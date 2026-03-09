# USB Game Transfer — Design

## Overview

Users plug a USB drive into the dice's USB-A port to sync game assets. The drive becomes the single source of truth: all existing games on-device are replaced with those on the drive. No SSH, network, or technical knowledge required.

## System Modes

### Run Mode (default)
Full ROS2 stack: IMU → chassis → screens → game manager → active game.

### Transfer Mode (USB with valid games inserted)
Partial stack: IMU → chassis → screens remain running (needed to display status on top-facing screen). Game manager node is killed. USB copy service runs.

### Mode Transitions

```
USB inserted (udev)
  → validate USB has games/ directory
  → if no games/ directory: ignore, stay in run mode
  → stop current game via /game_control service
  → kill game manager node
  → run USB copy service
  → blank non-top screens, show status on top screen

USB removed (udev)
  → unmount USB
  → restart game manager node
  → resume default game
```

Random USB drives (no `games/` folder) are ignored entirely — no mode switch.

## Storage

### Image-Based Isolation

A 2GB ext4 image file at `/opt/dicemaster_storage.img`, permanently mounted to `/home/dice/.dicemaster/`. Entry in `/etc/fstab` as a loop device. The fixed image size guarantees the eMMC system partition cannot be filled by game assets.

### Image Contents

```
/home/dice/.dicemaster/
├── games/          ← managed by USB transfer (wiped + replaced)
│   ├── game_a/
│   │   ├── config.json
│   │   └── assets/
│   └── game_b/
│       ├── config.json
│       └── assets/
└── strategies/     ← managed by developers via SSH/git (never touched by USB transfer)
    └── shake_quizlet/
        └── shake_quizlet.py
```

### Provisioning (one-time setup per device)

```bash
dd if=/dev/zero of=/opt/dicemaster_storage.img bs=1M count=2048
mkfs.ext4 /opt/dicemaster_storage.img
mkdir -p /home/dice/.dicemaster
# Add to /etc/fstab:
# /opt/dicemaster_storage.img /home/dice/.dicemaster ext4 loop,defaults 0 2
mount /home/dice/.dicemaster
mkdir -p /home/dice/.dicemaster/{games,strategies}
chown -R dice:dice /home/dice/.dicemaster
```

## USB Detection

### udev Rule

```
# /etc/udev/rules.d/99-dicemaster-usb.rules
ACTION=="add", SUBSYSTEM=="block", ENV{ID_FS_USAGE}=="filesystem", \
  TAG+="systemd", ENV{SYSTEMD_WANTS}="dicemaster-usb-transfer@%k.service"

ACTION=="remove", SUBSYSTEM=="block", \
  RUN+="/usr/local/bin/dicemaster-usb-cleanup.sh"
```

### USB Mount

Auto-mount inserted drive to `/media/usb0`. The transfer service handles mount/unmount.

## USB Drive Structure (expected)

```
USB:/
└── games/
    ├── chinese_quizlet/
    │   ├── config.json
    │   └── assets/
    │       ├── question.json
    │       └── computer/
    │           ├── images/
    │           └── answer.json
    └── another_game/
        ├── config.json
        └── assets/
```

## Copy Flow

### State Machine

```
IDLE → USB_DETECTED → SCANNING → READY_TO_COPY → COPYING → DONE → USB_REMOVED → IDLE
                                      ↓
                                  ERROR (→ waits for USB removal → IDLE)
```

Currently all transitions are automatic. Future GPIO button support can gate the `READY_TO_COPY → COPYING` transition on a confirm press.

### Steps

1. **USB_DETECTED**: Mount USB drive to `/media/usb0`. Check for `games/` directory at USB root. If absent, ignore (no mode switch).
2. **SCANNING**: Stop current game via `/game_control` service. Kill game manager node. Iterate `games/*/config.json` on USB. Classify each subdirectory as valid (has `config.json`) or invalid (missing).
3. **READY_TO_COPY**: Check total size of valid games vs free space in storage image. If insufficient, enter ERROR state.
4. **COPYING**: Delete all contents of `/home/dice/.dicemaster/games/`. Copy valid games from USB using `rsync` or `cp`. Track per-game progress.
5. **DONE**: Verify copied files (count/size sanity check). Display completion message. Wait for USB removal.
6. **USB_REMOVED**: Unmount `/media/usb0`. Restart game manager node (resumes default game). Return to IDLE.

### Error Cases

| Error | Screen Message | Recovery |
|-------|---------------|----------|
| No `games/` on USB | (no mode switch, ignored) | N/A |
| No valid games found | "No valid games found (need config.json)" | Remove USB |
| Insufficient space | "Not enough space: need XMB, have YMB" | Remove USB |
| Copy failure | "Copy failed: [error]" | Remove USB |

## Screen Feedback

### During Transfer Mode

- **Top-facing screen** (determined by IMU + chassis): shows transfer status
- **Other 5 screens**: blanked (black)
- If dice is rotated, status follows the new top screen

### Status Messages (sequential)

```
USB detected...

Scanning games...

Found 3 games
  ✓ chinese_quizlet
  ✓ animal_quiz
  ✗ broken_game (no config.json)

Copying 2/3: animal_quiz
[████████░░░░░░] 62%

Transfer complete!
Safe to remove USB.

(after USB removal)
Starting games...
```

## Future: GPIO Button Integration

When 4 GPIO buttons are available:

- **Confirm/cancel** before deleting existing games and starting copy
- **Select games** to include/exclude (scroll + toggle)
- **Acknowledge errors** (dismiss error screen)

The state machine design supports this: `READY_TO_COPY` can pause for button confirmation instead of auto-proceeding.

## Implementation Components

### On the Pi (new files)

| Component | Purpose |
|-----------|---------|
| `/etc/udev/rules.d/99-dicemaster-usb.rules` | Detect USB insert/remove |
| `/usr/local/bin/dicemaster-usb-transfer.sh` | Entry point: mount, validate, trigger ROS2 |
| `/usr/local/bin/dicemaster-usb-cleanup.sh` | Entry point: unmount, signal restore |
| systemd service (templated) | Runs the transfer script with proper environment |

### In DiceMaster_Central (new ROS2 code)

| Component | Purpose |
|-----------|---------|
| `usb_transfer_node.py` | ROS2 node: manages copy state machine, publishes screen commands |
| `usb_transfer.launch.py` | Launch file for the transfer node |

### Modifications to Existing Code

| File | Change |
|------|--------|
| `game_manager.py` | No changes needed — stopped/started externally |
| `config.py` | Add storage image path, max size constants |
| `dicemaster.launch.py` | No changes — game manager started/stopped independently |

## Dependencies

- `udev` (already present)
- `rsync` (install if not present)
- No new Python packages required

## Out of Scope

- Strategy management via USB (developer-only, via SSH/git)
- Game validation beyond `config.json` existence
- USB write-back (dice never writes to the USB drive)
- Multiple USB drives simultaneously
