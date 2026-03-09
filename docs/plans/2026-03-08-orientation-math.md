# Orientation Math Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace TF-based orientation computation with direct quaternion math, validated against full URDF transforms.

**Architecture:** Three files — (1) `urdf_to_dice_config.py` parses the URDF into a compact YAML config, (2) `orientation_math.py` computes all screen orientations from a single IMU quaternion using NumPy/SciPy, (3) `test_orientation_math.py` validates every computed frame against ground-truth URDF transforms for randomly sampled orientations.

**Tech Stack:** Python 3, NumPy, SciPy (`spatial.transform.Rotation`), PyYAML, xml.etree.ElementTree, pytest

---

### Task 1: URDF Config Extractor

**Files:**
- Create: `DiceMaster_Central/dicemaster_central/scripts/urdf_to_dice_config.py`
- Read: `DiceMaster_Central/dicemaster_central/resource/dice.urdf.xacro`
- Read: `DiceMaster_Central/dicemaster_central/resource/screen_component.urdf.xacro`
- Output: `DiceMaster_Central/dicemaster_central/resource/dice_geometry.yaml`

**Step 1: Write the URDF parser script**

Parse the processed URDF (`dice.urdf` — already generated from xacro) using `xml.etree.ElementTree`. Extract:

- `base_joint`: the `imu_link → base_link` joint's `rpy` converted to quaternion
- `canonical_edges`: the 4 edge offsets from `screen_component.urdf.xacro` (identical for all screens)
- `face_offset`: magnitude of screen joint `xyz` (should be 0.0508 for all screens)
- Per-screen `joint_quaternion`: each `screen_N_joint`'s `rpy` converted to quaternion

The script should:
1. Parse the URDF XML
2. Find `base_joint` and extract `origin rpy`
3. Find all `screen_N_joint` elements and extract `origin xyz` and `origin rpy`
4. Find edge joints from one screen (they're identical) and extract `origin xyz`
5. Validate: all screen offsets have the same magnitude, all edge offsets match the canonical set
6. Emit `dice_geometry.yaml`

```python
#!/usr/bin/env python3
"""Parse dice URDF and emit dice_geometry.yaml for the orientation math module."""

import argparse
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from scipy.spatial.transform import Rotation


def rpy_to_quaternion(rpy_str: str) -> list:
    """Convert 'roll pitch yaw' string to [x, y, z, w] quaternion."""
    rpy = [float(v) for v in rpy_str.split()]
    r = Rotation.from_euler('xyz', rpy)
    q = r.as_quat()  # [x, y, z, w]
    return [round(float(v), 10) for v in q]


def parse_urdf(urdf_path: str) -> dict:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = {j.get('name'): j for j in root.findall('joint')}

    # Base joint (imu_link → base_link)
    base_joint = joints['base_joint']
    base_rpy = base_joint.find('origin').get('rpy', '0 0 0')
    base_quat = rpy_to_quaternion(base_rpy)

    # Screen joints (base_link → screen_N_link)
    screens = {}
    offsets = []
    for i in range(1, 7):
        joint_name = f'screen_{i}_joint'
        joint = joints[joint_name]
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = origin.get('rpy', '0 0 0')
        joint_quat = rpy_to_quaternion(rpy)
        screens[i] = {'joint_quaternion': joint_quat}
        offsets.append(np.linalg.norm(xyz))

    # Validate all face offsets are equal
    face_offset = round(float(offsets[0]), 6)
    for i, off in enumerate(offsets):
        assert abs(off - face_offset) < 1e-6, \
            f"Screen {i+1} offset {off} != {face_offset}"

    # Canonical edges (from screen_1's edge joints — all screens are identical)
    edge_names = ['top', 'left', 'bottom', 'right']
    canonical_edges = {}
    for edge_name in edge_names:
        joint_name = f'screen_1_edge_{edge_name}_joint'
        if joint_name in joints:
            origin = joints[joint_name].find('origin')
            xyz = [round(float(v), 6) for v in origin.get('xyz', '0 0 0').split()]
            canonical_edges[edge_name] = xyz

    return {
        'base_joint': {'quaternion': base_quat},
        'face_offset': face_offset,
        'canonical_edges': canonical_edges,
        'screens': screens,
        'parameters': {
            'rotation_threshold': 0.7,
            'edge_detection_frames': 2,
            'orientation_rate': 10.0,
            'publish_tf': True,
        }
    }


def main():
    parser = argparse.ArgumentParser(description='Extract dice geometry from URDF')
    parser.add_argument('urdf_path', help='Path to dice.urdf')
    parser.add_argument('-o', '--output', default='dice_geometry.yaml',
                        help='Output YAML path')
    args = parser.parse_args()

    config = parse_urdf(args.urdf_path)

    with open(args.output, 'w') as f:
        yaml.dump(config, f, default_flow_style=None, sort_keys=False)

    print(f"Wrote {args.output}")
    print(f"  base_joint quaternion: {config['base_joint']['quaternion']}")
    print(f"  face_offset: {config['face_offset']}")
    print(f"  canonical_edges: {list(config['canonical_edges'].keys())}")
    print(f"  screens: {list(config['screens'].keys())}")


if __name__ == '__main__':
    main()
```

**Step 2: Run the script against the URDF**

```bash
cd DiceMaster_Central/dicemaster_central
python3 scripts/urdf_to_dice_config.py resource/dice.urdf -o resource/dice_geometry.yaml
```

Expected: generates `dice_geometry.yaml` with 6 screens, 4 canonical edges, face_offset=0.0508.

**Step 3: Commit**

```bash
git add scripts/urdf_to_dice_config.py resource/dice_geometry.yaml
git commit -m "Add URDF to dice geometry config extractor"
```

---

### Task 2: Orientation Math Module

**Files:**
- Create: `DiceMaster_Central/dicemaster_central/dicemaster_central/hw/orientation_math.py`

**Step 1: Write the orientation math module**

This module takes a YAML config (from Task 1) and an IMU quaternion, and computes:
- All 6 screen `up_alignment` values (z-component of rotated face normal)
- The top screen ID
- The edge rotation of the top screen (which edge is lowest → 0°/90°/180°/270°)

Uses vectorized NumPy operations: batch-rotate all 6 face normals in one `Rotation.apply()` call, batch-rotate 4 edges in one call.

```python
"""
Orientation math for DiceMaster dice.

Computes screen orientations from a single IMU quaternion and
precomputed dice geometry, replacing TF-based lookups.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Dict, Tuple, Optional
import yaml


class DiceOrientation:
    """Precomputes dice geometry and computes orientations from IMU quaternions."""

    EDGE_TO_ROTATION = {'bottom': 0, 'right': 1, 'top': 2, 'left': 3}

    def __init__(self, config_path: str):
        with open(config_path) as f:
            config = yaml.safe_load(f)
        self._load_geometry(config)

    def _load_geometry(self, config: dict):
        """Precompute face normals and edge offsets in base_link frame."""
        # Base joint: imu_link → base_link
        bq = config['base_joint']['quaternion']
        self.base_rotation = Rotation.from_quat(bq)  # [x, y, z, w]

        face_offset = config['face_offset']
        canonical_edges = config['canonical_edges']
        screens = config['screens']
        self.screen_ids = sorted(screens.keys())
        n = len(self.screen_ids)

        # Precompute face normals in base_link frame (Nx3)
        # The canonical face normal is [0, 0, 1] (pointing outward).
        # Each screen's joint_quaternion rotates it to the correct direction.
        self.face_normals = np.zeros((n, 3))
        self.face_offsets = np.zeros((n, 3))

        # Per-screen joint rotations (for edge computation)
        self.screen_joint_rotations = []

        for i, sid in enumerate(self.screen_ids):
            jq = screens[sid]['joint_quaternion']
            joint_rot = Rotation.from_quat(jq)
            self.screen_joint_rotations.append(joint_rot)

            # Face normal = joint_rotation applied to [0, 0, 1]
            self.face_normals[i] = joint_rot.apply([0, 0, 1])

            # Face center offset = joint_rotation applied to [0, 0, face_offset]
            self.face_offsets[i] = joint_rot.apply([0, 0, face_offset])

        # Canonical edge offsets in screen-local frame (4x3)
        edge_names = ['top', 'right', 'bottom', 'left']
        self.edge_names = edge_names
        self.canonical_edge_offsets = np.array(
            [canonical_edges[name] for name in edge_names]
        )

        # Precompute edge offsets in base_link frame for each screen (Nx4x3)
        self.screen_edge_offsets = np.zeros((n, 4, 3))
        for i, sid in enumerate(self.screen_ids):
            joint_rot = self.screen_joint_rotations[i]
            self.screen_edge_offsets[i] = joint_rot.apply(self.canonical_edge_offsets)

    def compute(self, imu_quat: np.ndarray) -> dict:
        """Compute all screen orientations from an IMU quaternion.

        Args:
            imu_quat: [x, y, z, w] quaternion from IMU (world → imu_link)

        Returns:
            dict with keys:
                'up_alignments': {screen_id: float} — [-1, 1] for each screen
                'top_screen': int — screen_id of the upward-facing screen
                'bottom_screen': int — screen_id of the downward-facing screen
                'top_rotation': int — 0/1/2/3 for the top screen's edge rotation
                'face_z': {screen_id: float} — z position of each face center
                'top_edge_z': {edge_name: float} — z positions of top screen's edges
        """
        # Composite rotation: world → imu_link → base_link
        imu_rot = Rotation.from_quat(imu_quat)
        world_rot = imu_rot * self.base_rotation

        # Phase 1: batch-rotate all face normals (Nx3 → Nx3)
        world_normals = world_rot.apply(self.face_normals)
        # up_alignment = z-component of rotated face normal
        z_components = world_normals[:, 2]

        # Clamp to [-1, 1]
        up_alignments = np.clip(z_components, -1.0, 1.0)

        # Top and bottom screen
        top_idx = int(np.argmax(up_alignments))
        bottom_idx = int(np.argmin(up_alignments))
        top_screen = self.screen_ids[top_idx]
        bottom_screen = self.screen_ids[bottom_idx]

        # Face center z positions (for validation)
        world_offsets = world_rot.apply(self.face_offsets)
        face_z = {self.screen_ids[i]: float(world_offsets[i, 2])
                  for i in range(len(self.screen_ids))}

        # Phase 2: edges for top screen only (4x3 → 4x3)
        top_edges_base = self.screen_edge_offsets[top_idx]  # (4, 3)
        world_edges = world_rot.apply(top_edges_base)
        edge_z_values = world_edges[:, 2] + world_offsets[top_idx, 2]

        top_edge_z = {self.edge_names[j]: float(edge_z_values[j])
                      for j in range(4)}

        # Lowest edge → rotation
        lowest_edge_idx = int(np.argmin(edge_z_values))
        lowest_edge = self.edge_names[lowest_edge_idx]
        top_rotation = self.EDGE_TO_ROTATION[lowest_edge]

        up_alignment_dict = {self.screen_ids[i]: float(up_alignments[i])
                             for i in range(len(self.screen_ids))}

        return {
            'up_alignments': up_alignment_dict,
            'top_screen': top_screen,
            'bottom_screen': bottom_screen,
            'top_rotation': top_rotation,
            'face_z': face_z,
            'top_edge_z': top_edge_z,
        }

    def compute_all_edges(self, imu_quat: np.ndarray) -> dict:
        """Compute edge z positions for ALL screens (for validation only).

        Args:
            imu_quat: [x, y, z, w] quaternion from IMU

        Returns:
            {screen_id: {edge_name: z_position}} for all 6 screens
        """
        imu_rot = Rotation.from_quat(imu_quat)
        world_rot = imu_rot * self.base_rotation

        world_offsets = world_rot.apply(self.face_offsets)

        all_edges = {}
        for i, sid in enumerate(self.screen_ids):
            top_edges_base = self.screen_edge_offsets[i]
            world_edges = world_rot.apply(top_edges_base)
            edge_z = world_edges[:, 2] + world_offsets[i, 2]
            all_edges[sid] = {self.edge_names[j]: float(edge_z[j])
                              for j in range(4)}
        return all_edges
```

**Step 2: Verify it imports cleanly**

```bash
cd DiceMaster_Central/dicemaster_central
python3 -c "from dicemaster_central.hw.orientation_math import DiceOrientation; print('OK')"
```

**Step 3: Commit**

```bash
git add dicemaster_central/hw/orientation_math.py
git commit -m "Add orientation math module with vectorized quaternion computation"
```

---

### Task 3: Ground Truth URDF Transform Module

**Files:**
- Create: `DiceMaster_Central/dicemaster_central/tests/test_orientation_math.py`

This is the test file. It contains:
1. A ground truth function that computes ALL transforms by walking the full URDF joint chain (equivalent to what `robot_state_publisher` + TF would produce)
2. Test cases that compare the ground truth against `DiceOrientation.compute()` for random IMU quaternions

**Step 1: Write the test**

```python
#!/usr/bin/env python3
"""
Validate orientation_math.py against ground-truth URDF transforms.

For each randomly sampled IMU quaternion, computes all screen face positions
and edge positions two ways:
  1. Ground truth: full URDF joint chain traversal (equivalent to robot_state_publisher + TF)
  2. Optimized: DiceOrientation.compute() (symmetry-based, vectorized)

Asserts numerical equality within floating-point tolerance.
"""

import os
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from dicemaster_central.hw.orientation_math import DiceOrientation

URDF_PATH = os.path.join(os.path.dirname(__file__), '..', 'resource', 'dice.urdf')
CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'resource', 'dice_geometry.yaml')

TOLERANCE = 1e-9
N_RANDOM_SAMPLES = 200


# ──────────────────────────────────────────────
# Ground truth: full URDF chain traversal
# ──────────────────────────────────────────────

class URDFGroundTruth:
    """Compute all frame transforms by walking the URDF joint chain.
    Equivalent to robot_state_publisher + tf2 lookups."""

    def __init__(self, urdf_path: str):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        self.joints = {}
        for j in root.findall('joint'):
            name = j.get('name')
            origin = j.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
            parent = j.find('parent').get('link')
            child = j.find('child').get('link')
            self.joints[name] = {
                'parent': parent, 'child': child,
                'xyz': np.array(xyz),
                'rotation': Rotation.from_euler('xyz', rpy),
            }

        # Build parent→child map for chain traversal
        self.child_to_joint = {j['child']: j for j in self.joints.values()}

    def _chain_to_root(self, frame_name: str):
        """Walk from frame_name up to the root (imu_link), collecting joints."""
        chain = []
        current = frame_name
        while current in self.child_to_joint:
            joint = self.child_to_joint[current]
            chain.append(joint)
            current = joint['parent']
        chain.reverse()  # root → leaf order
        return chain

    def get_world_transform(self, frame_name: str, imu_quat: np.ndarray):
        """Get world-frame position and rotation of a named frame.

        Args:
            frame_name: e.g. 'screen_1_link', 'screen_1_edge_top'
            imu_quat: [x, y, z, w] quaternion (world → imu_link)

        Returns:
            (position_xyz, rotation) in world frame
        """
        # Start at world frame: apply imu quaternion to get to imu_link
        current_rot = Rotation.from_quat(imu_quat)
        current_pos = np.zeros(3)

        # Walk the joint chain from imu_link to the target frame
        chain = self._chain_to_root(frame_name)
        for joint in chain:
            # Apply joint: translate then rotate
            current_pos = current_pos + current_rot.apply(joint['xyz'])
            current_rot = current_rot * joint['rotation']

        return current_pos, current_rot

    def get_frame_z(self, frame_name: str, imu_quat: np.ndarray) -> float:
        """Get the z-position of a frame in world coordinates."""
        pos, _ = self.get_world_transform(frame_name, imu_quat)
        return pos[2]


# ──────────────────────────────────────────────
# Random quaternion generation
# ──────────────────────────────────────────────

def random_unit_quaternions(n: int, seed: int = 42) -> np.ndarray:
    """Generate n random unit quaternions uniformly distributed on SO(3).
    Returns (n, 4) array with [x, y, z, w] convention."""
    rng = np.random.default_rng(seed)
    # Uniform random rotations via scipy
    rotations = Rotation.random(n, random_state=rng)
    return rotations.as_quat()  # [x, y, z, w]


# ──────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────

@pytest.fixture(scope='module')
def ground_truth():
    return URDFGroundTruth(URDF_PATH)


@pytest.fixture(scope='module')
def dice_orientation():
    return DiceOrientation(CONFIG_PATH)


@pytest.fixture(scope='module')
def random_quats():
    return random_unit_quaternions(N_RANDOM_SAMPLES)


class TestFaceZPositions:
    """Validate face center z-positions match URDF ground truth."""

    def test_all_faces_all_orientations(self, ground_truth, dice_orientation, random_quats):
        """For N random IMU quaternions, compare all 6 face z-positions."""
        max_error = 0.0
        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)

            for screen_id in range(1, 7):
                expected_z = ground_truth.get_frame_z(
                    f'screen_{screen_id}_link', q
                )
                actual_z = result['face_z'][screen_id]
                error = abs(expected_z - actual_z)
                max_error = max(max_error, error)
                assert error < TOLERANCE, (
                    f"Sample {i}, screen {screen_id}: "
                    f"expected z={expected_z:.12f}, got z={actual_z:.12f}, "
                    f"error={error:.2e}"
                )
        print(f"\nFace z-positions: max error = {max_error:.2e} across "
              f"{len(random_quats) * 6} comparisons")


class TestUpAlignment:
    """Validate up_alignment values match z-position-derived values."""

    def test_up_alignment_consistent_with_face_z(self, ground_truth,
                                                  dice_orientation, random_quats):
        """up_alignment should equal face_z / face_offset, clamped to [-1, 1]."""
        face_offset = 0.0508  # from URDF
        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)

            for screen_id in range(1, 7):
                z = result['face_z'][screen_id]
                expected_align = np.clip(z / face_offset, -1.0, 1.0)
                actual_align = result['up_alignments'][screen_id]
                error = abs(expected_align - actual_align)
                assert error < TOLERANCE, (
                    f"Sample {i}, screen {screen_id}: "
                    f"alignment mismatch: {expected_align:.12f} vs {actual_align:.12f}"
                )


class TestTopBottomScreen:
    """Validate top/bottom screen identification."""

    def test_top_screen_has_highest_z(self, ground_truth, dice_orientation,
                                      random_quats):
        """Top screen should have the highest face z-position."""
        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)

            top_z = result['face_z'][result['top_screen']]
            for screen_id, z in result['face_z'].items():
                assert z <= top_z + TOLERANCE, (
                    f"Sample {i}: screen {screen_id} z={z:.12f} > "
                    f"top screen {result['top_screen']} z={top_z:.12f}"
                )

    def test_bottom_screen_has_lowest_z(self, ground_truth, dice_orientation,
                                         random_quats):
        """Bottom screen should have the lowest face z-position."""
        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)

            bottom_z = result['face_z'][result['bottom_screen']]
            for screen_id, z in result['face_z'].items():
                assert z >= bottom_z - TOLERANCE, (
                    f"Sample {i}: screen {screen_id} z={z:.12f} < "
                    f"bottom screen {result['bottom_screen']} z={bottom_z:.12f}"
                )


class TestEdgeZPositions:
    """Validate edge z-positions match URDF ground truth for all screens."""

    def test_all_edges_all_screens(self, ground_truth, dice_orientation,
                                    random_quats):
        """For N random quaternions, compare all 24 edge z-positions."""
        max_error = 0.0
        edge_names = ['top', 'right', 'bottom', 'left']
        for i in range(len(random_quats)):
            q = random_quats[i]
            all_edges = dice_orientation.compute_all_edges(q)

            for screen_id in range(1, 7):
                for edge_name in edge_names:
                    frame_name = f'screen_{screen_id}_edge_{edge_name}'
                    expected_z = ground_truth.get_frame_z(frame_name, q)
                    actual_z = all_edges[screen_id][edge_name]
                    error = abs(expected_z - actual_z)
                    max_error = max(max_error, error)
                    assert error < TOLERANCE, (
                        f"Sample {i}, screen {screen_id}, edge {edge_name}: "
                        f"expected z={expected_z:.12f}, got z={actual_z:.12f}, "
                        f"error={error:.2e}"
                    )
        print(f"\nEdge z-positions: max error = {max_error:.2e} across "
              f"{len(random_quats) * 24} comparisons")


class TestEdgeRotation:
    """Validate edge-based rotation matches ground truth edge z ordering."""

    def test_top_rotation_matches_lowest_edge(self, ground_truth,
                                               dice_orientation, random_quats):
        """Top screen's rotation should match whichever edge has lowest z."""
        edge_to_rotation = {'bottom': 0, 'right': 1, 'top': 2, 'left': 3}
        edge_names = ['top', 'right', 'bottom', 'left']

        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)
            top_screen = result['top_screen']

            # Ground truth: compute all 4 edge z-positions for the top screen
            edge_z = {}
            for edge_name in edge_names:
                frame_name = f'screen_{top_screen}_edge_{edge_name}'
                edge_z[edge_name] = ground_truth.get_frame_z(frame_name, q)

            expected_lowest = min(edge_z, key=edge_z.get)
            expected_rotation = edge_to_rotation[expected_lowest]

            assert result['top_rotation'] == expected_rotation, (
                f"Sample {i}, top screen {top_screen}: "
                f"expected rotation {expected_rotation} "
                f"(edge '{expected_lowest}'), got {result['top_rotation']}. "
                f"Edge z: {edge_z}"
            )


class TestKnownOrientations:
    """Test specific known orientations for sanity."""

    def test_identity_quaternion(self, dice_orientation):
        """Identity quaternion — no rotation from IMU."""
        q = np.array([0.0, 0.0, 0.0, 1.0])  # identity
        result = dice_orientation.compute(q)

        # With the base_joint rotation (pi around X, pi around Z),
        # the result depends on the URDF. Just verify it returns valid data.
        assert result['top_screen'] in range(1, 7)
        assert result['bottom_screen'] in range(1, 7)
        assert result['top_screen'] != result['bottom_screen']
        assert len(result['up_alignments']) == 6
        assert len(result['face_z']) == 6
        assert len(result['top_edge_z']) == 4

    def test_opposite_screens_have_opposite_alignment(self, dice_orientation,
                                                       random_quats):
        """Opposite faces (1/6, 2/4, 3/5) should have opposite up_alignments."""
        opposite_pairs = [(1, 6), (2, 4), (3, 5)]
        for i in range(len(random_quats)):
            q = random_quats[i]
            result = dice_orientation.compute(q)
            for a, b in opposite_pairs:
                align_sum = result['up_alignments'][a] + result['up_alignments'][b]
                assert abs(align_sum) < TOLERANCE, (
                    f"Sample {i}: screens {a},{b} alignments "
                    f"{result['up_alignments'][a]:.12f} + "
                    f"{result['up_alignments'][b]:.12f} = {align_sum:.12f} "
                    f"(should be ~0)"
                )


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
```

**Step 2: Run tests — expect them to fail first (config file may not exist yet)**

```bash
cd DiceMaster_Central/dicemaster_central
python3 -m pytest tests/test_orientation_math.py -v --tb=short
```

If config doesn't exist, run the extractor first:
```bash
python3 scripts/urdf_to_dice_config.py resource/dice.urdf -o resource/dice_geometry.yaml
python3 -m pytest tests/test_orientation_math.py -v --tb=short
```

Expected: all tests pass with max error < 1e-9.

**Step 3: Commit**

```bash
git add tests/test_orientation_math.py
git commit -m "Add orientation math validation tests against URDF ground truth"
```

---

### Task 4: Validate and Fix

**Step 1: Run the full pipeline end-to-end**

```bash
cd DiceMaster_Central/dicemaster_central
python3 scripts/urdf_to_dice_config.py resource/dice.urdf -o resource/dice_geometry.yaml
python3 -m pytest tests/test_orientation_math.py -v --tb=long -s
```

**Step 2: Fix any failures**

Common issues to watch for:
- Quaternion convention mismatch (`[x,y,z,w]` vs `[w,x,y,z]`)
- Edge name mapping mismatch between URDF xacro (`edge_left` joint maps to `+Y` in screen frame) and the orientation code
- The `base_joint` rpy `"-3.14159 0 3.14159"` is a pi rotation around X then Z — verify this matches the existing chassis.py behavior

**Step 3: Final commit if fixes were needed**

```bash
git add -A
git commit -m "Fix orientation math validation issues"
```
