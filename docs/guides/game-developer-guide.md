# DiceMaster Game Developer Guide

This guide is for educators and developers who want to build games that run on the DiceMaster hardware. It assumes you are comfortable with Python and have read (or at least skimmed) the [Hello Dice walkthrough](../beginner_game_guide.md), which covers installation and first-run from scratch.

---

## 1. Introduction

### What is a DiceMaster game?

A DiceMaster game has two parts:

1. **Game directory** — a folder containing a `config.json` and an `assets/` subdirectory. The game directory is where you store all content the dice will display: text cards, images, and animated GIFs.
2. **Strategy** — a Python class (subclassing `BaseStrategy`) that wires together sensors and screens. It lives in a `strategies/` directory that is scanned at startup.

The game manager discovers both at launch. When it finds a pair whose `config.json` points to a known strategy, it instantiates the strategy node and calls `start_strategy()`.

### Two ways to develop

| Mode | When to use |
|------|------------|
| **DiceMaster Studio** (web simulator) | Rapid iteration — no hardware required. Runs in your browser and simulates all six screens, shake events, and orientation changes. |
| **On hardware** (Raspberry Pi) | Final testing and demos on the physical dice. Requires SSH access to the Pi. |

Both modes use the same Python code. See [Section 7](#7-testing-your-game) for setup instructions for each.

### First-time setup

If you have not yet created and run your first game, follow the [Hello Dice walkthrough](../beginner_game_guide.md) before continuing here.

---

## 2. Game Structure

### Directory layout

```
my_game/
├── config.json
└── assets/
    ├── card_1.json          # text card
    ├── banner.jpg           # static image  480×480 px
    └── sparkle.gif.d/       # animated GIF (directory of frames)
        ├── 0.jpg
        ├── 1.jpg
        └── 2.jpg
```

The `assets/` tree may contain subdirectories. The `dice.assets` module (see below) resolves paths relative to the assets root, so you never need to hard-code absolute paths.

### config.json fields

```json
{
    "game_name": "my_game",
    "strategy": "my_strategy",
    "strategy_config": {
        "rounds": 10,
        "show_hints": true
    }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `game_name` | string | Unique identifier for this game. Must match the directory name. |
| `strategy` | string | Name of the strategy to load (see naming rules below). |
| `strategy_config` | object | Arbitrary key-value pairs forwarded to the strategy constructor as keyword arguments. Use `{}` if you have no options. |

### Strategy file naming

The strategy loader uses a strict naming convention:

```
strategies/
└── my_strategy/
    └── my_strategy.py    ← file name matches directory name
```

Inside `my_strategy.py` your class must:

- inherit from `dice.strategy.BaseStrategy`
- set the class attribute `_strategy_name = "my_strategy"` (matching the directory and file name)

The game manager scans all strategy directories, finds the first `BaseStrategy` subclass in each file, and registers it by `_strategy_name`.

---

## 3. The `dice` SDK

The `dice` package is the recommended API for writing strategies. It is a thin wrapper over ROS2 that hides node lifecycle, publisher/subscriber management, and message construction behind simple function calls.

### Core modules

| Module | Import | Purpose |
|--------|--------|---------|
| `dice.screen` | `from dice import screen` | Send content to a screen face |
| `dice.motion` | `from dice import motion` | React to shake events |
| `dice.orientation` | `from dice import orientation` | Track which face is up/down |
| `dice.timer` | `from dice import timer` | Schedule repeating or one-shot callbacks |
| `dice.assets` | `from dice import assets` | Resolve asset file paths |
| `dice` (top-level) | `from dice import log` | Logging |

### `dice.screen`

```python
screen.set_text(screen_id: int, path: str)    # display a text JSON card
screen.set_image(screen_id: int, path: str)   # display a JPG or PNG
screen.set_gif(screen_id: int, path: str)     # play a *.gif.d directory
```

`screen_id` is an integer 1–6. Use `dice.assets.get()` to resolve paths:

```python
screen.set_text(1, assets.get("card_1.json"))
```

### `dice.motion`

```python
motion.on_shake(handler)         # register a callback: handler(intensity: float)
motion.is_shaking() -> bool      # poll current state
motion.shake_intensity() -> float
```

Handlers registered with `on_shake` are called every time the shake flag rises. Multiple handlers are supported.

### `dice.orientation`

```python
orientation.on_change(handler)   # callback: handler(top: int, bottom: int)
orientation.top() -> int         # screen ID currently facing up
orientation.bottom() -> int      # screen ID currently facing down
```

Side screens are any IDs not equal to `top()` or `bottom()`.

### `dice.timer`

```python
tid = timer.set(interval: float, callback)   # repeating, returns timer id
tid = timer.once(delay: float, callback)     # one-shot
timer.cancel(tid)                            # stop a timer
```

### `dice.assets`

```python
assets.get("card_1.json")        # -> "/abs/path/to/assets/card_1.json"
assets.list_all()                # -> sorted list of relative paths in assets/
```

### Key ROS2 topics

These are the underlying topics the `dice` SDK wraps. You do not need to publish or subscribe to them directly, but they are useful for debugging.

| Topic | Direction | Message type | Description |
|-------|-----------|-------------|-------------|
| `/screen_{1-6}_cmd` | Publish | `ScreenMediaCmd` | Send content to one screen |
| `/imu/motion` | Subscribe | `MotionDetection` | Shake and rotation events |
| `/chassis/orientation` | Subscribe | `ChassisOrientation` | Which face is up/down |

For full message field definitions see [DiceMaster_Central/docs/api/motion_detection.md](../../DiceMaster_Central/docs/api/motion_detection.md) and [DiceMaster_Central/docs/api/architecture.md](../../DiceMaster_Central/docs/api/architecture.md).

---

## 4. Introductory Example: "Flashcard Game"

This example displays vocabulary flashcards. Each card is a JSON text file in `assets/`. Shaking the dice advances to the next card. It is intentionally minimal — no orientation tracking, no images.

### File layout

```
examples/games/vocab_flash/
├── config.json
└── assets/
    ├── card_1.json
    ├── card_2.json
    └── card_3.json

examples/strategies/vocab_flash/
└── vocab_flash.py
```

### config.json

```json
{
    "game_name": "vocab_flash",
    "strategy": "vocab_flash",
    "strategy_config": {
        "display_screen": 1
    }
}
```

The `display_screen` key is passed to the constructor as a keyword argument. This makes it easy to point the game at any face without touching the strategy code.

### Sample asset: assets/card_1.json

```json
{
    "bg_color": "0x0000",
    "texts": [
        {
            "x_cursor": 80,
            "y_cursor": 160,
            "font_name": "tf",
            "font_color": "0x07FF",
            "text": "ephemeral"
        },
        {
            "x_cursor": 60,
            "y_cursor": 260,
            "font_name": "tf",
            "font_color": "0xFFFF",
            "text": "lasting for a very short time"
        }
    ]
}
```

Colors are RGB565 hex values. `0x0000` is black, `0xFFFF` is white, `0x07FF` is cyan. See [Section 6](#6-media-types-reference) for a color reference.

### examples/strategies/vocab_flash/vocab_flash.py

```python
"""
Vocab Flash — cycles vocabulary flashcards on a single screen.

Shake once to advance to the next card. The set of cards is discovered
automatically from all *.json files in assets/, so adding a new card
is as simple as dropping a new file into assets/.
"""

import time
from dice import screen, motion, assets, log, timer
from dice.strategy import BaseStrategy


class VocabFlashStrategy(BaseStrategy):
    # Must match config.json "strategy" field and the directory/file name.
    _strategy_name = "vocab_flash"

    def __init__(self, game_name: str, config: dict, assets_path: str,
                 display_screen: int = 1, **kwargs):
        super().__init__(game_name, config, assets_path, **kwargs)

        self._display_screen = display_screen  # which face shows the card

        # Discover all JSON card files in assets/ (sorted for stable ordering)
        all_files = [p for p in assets.list_all() if p.endswith(".json")]
        self._cards = sorted(all_files)        # e.g. ["card_1.json", "card_2.json", ...]
        self._index = 0                        # which card is currently shown

        # Cooldown: ignore shakes for 1.5 s after a card change
        self._last_advance = 0.0
        self._cooldown = 1.5

        log(f"VocabFlash: found {len(self._cards)} cards on screen {display_screen}")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start_strategy(self) -> None:
        """Called once when the game starts. Register callbacks and show card 0."""
        motion.on_shake(self._on_shake)    # wire up the shake handler
        self._show_current_card()

    def stop_strategy(self) -> None:
        """Called when the game stops or is switched out."""
        log("VocabFlash stopped")

    # ------------------------------------------------------------------
    # Display helpers
    # ------------------------------------------------------------------

    def _show_current_card(self) -> None:
        if not self._cards:
            log("VocabFlash: no cards found in assets/")
            return
        path = assets.get(self._cards[self._index])
        screen.set_text(self._display_screen, path)
        log(f"VocabFlash: showing {self._cards[self._index]}")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_shake(self, intensity: float) -> None:
        """Advance to the next card, respecting the cooldown window."""
        now = time.monotonic()
        if now - self._last_advance < self._cooldown:
            return                             # still in cooldown — ignore

        self._last_advance = now
        self._index = (self._index + 1) % len(self._cards)  # wrap around
        self._show_current_card()
```

Key things to notice:

- `assets.list_all()` discovers cards automatically — drop a new JSON file into `assets/` and it appears next run.
- The `display_screen` parameter comes from `strategy_config` in `config.json`, so teachers can configure which face shows the card without editing Python.
- The cooldown pattern (comparing `time.monotonic()` against a saved timestamp) prevents a single vigorous shake from skipping multiple cards.
- `stop_strategy()` does not need to cancel anything here because `motion.on_shake` handlers are cleaned up by the runtime. Only explicit `timer.set()` calls need a matching `timer.cancel()`.

---

## 5. Advanced Example: "Orientation Quiz"

This example demonstrates orientation-aware display, image hints, multiple-question cycling, and a cooldown timer. Questions appear on the TOP face, answers on the BOTTOM face, and four hint images on the SIDE faces. Shaking the dice three times in a row advances to the next question.

### File layout

```
examples/games/orient_quiz/
├── config.json
└── assets/
    ├── question.json              # shared question prompt shown on top
    ├── apple/
    │   ├── answer.json            # answer card (shown on bottom)
    │   └── images/
    │       ├── apple_1.jpg
    │       ├── apple_2.jpg
    │       ├── apple_3.jpg
    │       └── apple_spin.gif.d/  # optional animated hint
    │           ├── 0.jpg
    │           └── 1.jpg
    ├── bicycle/
    │   ├── answer.json
    │   └── images/
    │       ├── bicycle_1.jpg
    │       ├── bicycle_2.jpg
    │       ├── bicycle_3.jpg
    │       └── bicycle_4.jpg
    └── cloud/
        ├── answer.json
        └── images/
            ├── cloud_1.jpg
            ├── cloud_2.jpg
            ├── cloud_3.jpg
            └── cloud_4.jpg
```

### config.json

```json
{
    "game_name": "orient_quiz",
    "strategy": "orient_quiz",
    "strategy_config": {
        "shakes_to_advance": 3,
        "cooldown_seconds": 3.0
    }
}
```

### examples/strategies/orient_quiz/orient_quiz.py

```python
"""
Orientation Quiz — a multi-face vocabulary quiz.

Layout:
  TOP screen    → shared question prompt
  BOTTOM screen → answer for the current card
  SIDE screens  → four hint images (or GIFs) from the card's images/ folder

Interaction:
  - Shake the dice N times (default 3) in quick succession to advance.
  - A cooldown timer prevents accidental double-advances.
  - The game waits for the first orientation event before displaying
    anything, so the correct faces are used from the start.
"""

import os
import random
import time

from dice import screen, motion, orientation, assets, log
from dice.strategy import BaseStrategy


class OrientQuizStrategy(BaseStrategy):
    _strategy_name = "orient_quiz"

    def __init__(self, game_name: str, config: dict, assets_path: str,
                 shakes_to_advance: int = 3,
                 cooldown_seconds: float = 3.0,
                 **kwargs):
        super().__init__(game_name, config, assets_path, **kwargs)

        self._shakes_needed = shakes_to_advance
        self._cooldown = cooldown_seconds

        # Screen assignment — populated on the first orientation event
        self._top = None
        self._bottom = None
        self._sides = []
        self._all_ids = list(range(1, 7))   # screens 1-6

        # Shake tracking
        self._shake_times = []              # timestamps of recent shakes
        self._last_advance = 0.0            # time of last card change

        # Question data
        self._cards = []                    # list of card dicts
        self._index = 0
        self._ready = False                 # False until first orientation arrives

        self._load_cards()

    # ------------------------------------------------------------------
    # Asset loading
    # ------------------------------------------------------------------

    def _load_cards(self) -> None:
        """Discover quiz cards from subdirectories of assets/."""
        root = self._assets_path
        for item in sorted(os.listdir(root)):
            card_dir = os.path.join(root, item)
            if not os.path.isdir(card_dir):
                continue                    # skip files at the root level

            answer_path = os.path.join(card_dir, "answer.json")
            images_dir = os.path.join(card_dir, "images")

            if not os.path.exists(answer_path) or not os.path.isdir(images_dir):
                continue                    # not a valid card directory

            # Collect image and GIF files from images/
            image_paths = []
            for fname in sorted(os.listdir(images_dir)):
                fpath = os.path.join(images_dir, fname)
                if fname.lower().endswith((".jpg", ".jpeg", ".png")):
                    image_paths.append(fpath)
                elif fname.endswith(".gif.d") and os.path.isdir(fpath):
                    image_paths.append(fpath)

            if len(image_paths) < 4:
                log(f"orient_quiz: card '{item}' has only {len(image_paths)} images, need 4 — skipping")
                continue

            self._cards.append({
                "name": item,
                "answer_path": answer_path,
                "image_paths": image_paths,
            })

        random.shuffle(self._cards)
        log(f"orient_quiz: loaded {len(self._cards)} cards")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start_strategy(self) -> None:
        orientation.on_change(self._on_orientation)  # fires when dice rotates
        motion.on_shake(self._on_shake)              # fires on shake events
        log("orient_quiz: waiting for first orientation event…")

    def stop_strategy(self) -> None:
        self._shake_times.clear()
        log("orient_quiz: stopped")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_orientation(self, top: int, bottom: int) -> None:
        """Called every time the dice settles into a new orientation."""
        self._top = top
        self._bottom = bottom
        self._sides = [sid for sid in self._all_ids if sid != top and sid != bottom]

        if not self._ready:
            self._ready = True
            self._display_current_card()    # first display now that we know which face is up
        else:
            self._display_current_card()    # re-display with new face assignments

    def _on_shake(self, intensity: float) -> None:
        """Accumulate shakes; advance when N shakes arrive within the cooldown window."""
        now = time.monotonic()

        # Drop shakes that are outside the cooldown window
        if now - self._last_advance < self._cooldown:
            return                          # still cooling down from last advance

        # Keep only shakes within the last 2 seconds
        window = 2.0
        self._shake_times = [t for t in self._shake_times if now - t < window]
        self._shake_times.append(now)

        if len(self._shake_times) >= self._shakes_needed:
            self._shake_times.clear()
            self._last_advance = now
            self._advance()

    # ------------------------------------------------------------------
    # Display helpers
    # ------------------------------------------------------------------

    def _advance(self) -> None:
        if not self._cards:
            return
        self._index = (self._index + 1) % len(self._cards)
        log(f"orient_quiz: advancing to card {self._index}")
        self._display_current_card()

    def _display_current_card(self) -> None:
        """Push content to all six faces for the current card."""
        if not self._ready or not self._cards:
            return

        card = self._cards[self._index]
        question_path = assets.get("question.json")

        # TOP: shared question prompt
        screen.set_text(self._top, question_path)

        # BOTTOM: answer for this card
        screen.set_text(self._bottom, card["answer_path"])

        # SIDES: four hint images in random order
        chosen = random.sample(card["image_paths"], min(4, len(card["image_paths"])))
        sides = self._sides.copy()
        random.shuffle(sides)
        for i, img_path in enumerate(chosen):
            if i >= len(sides):
                break
            if img_path.endswith(".gif.d"):
                screen.set_gif(sides[i], img_path)
            else:
                screen.set_image(sides[i], img_path)

        log(f"orient_quiz: displaying '{card['name']}'")
```

Key patterns demonstrated:

- **Deferred first display** — the strategy stores `_ready = False` and only calls `_display_current_card()` once the first orientation event has arrived. This prevents sending content to the wrong face.
- **Re-display on rotation** — `_on_orientation` is called every time the dice settles into a new orientation. Calling `_display_current_card()` inside it means the content always appears on the correct faces without extra bookkeeping.
- **Shake accumulation window** — instead of counting consecutive shakes with a simple counter, the handler keeps timestamps and purges entries older than two seconds. This is more robust than counting because it handles variable shake timing.
- **GIF detection** — the loader checks for `.gif.d` directory entries alongside image files, and routes them to `screen.set_gif()` automatically.

---

## 6. Media Types Reference

### Content types

| Type | File format | How to display | Notes |
|------|------------|----------------|-------|
| Text card | `.json` | `screen.set_text(id, path)` | Rendered on-device. RGB565 colors. Max 255 bytes of text per entry. |
| Static image | `.jpg` / `.png` | `screen.set_image(id, path)` | 480×480 px recommended. Larger files are slower to transfer. |
| Animated GIF | directory `*.gif.d/` containing `0.jpg`, `1.jpg`, … | `screen.set_gif(id, path)` | Playback at ~12 fps. Pass the directory path, not a frame path. |

### Text card JSON format

```json
{
    "bg_color": "0x0000",
    "texts": [
        {
            "x_cursor": 60,
            "y_cursor": 200,
            "font_name": "tf",
            "font_color": "0xFFFF",
            "text": "Hello"
        }
    ]
}
```

| Field | Description |
|-------|-------------|
| `bg_color` | Background fill in RGB565 hex (e.g. `"0x0000"` = black) |
| `x_cursor`, `y_cursor` | Top-left origin of the text, in pixels. Screen is 480×480, (0,0) is top-left. |
| `font_name` | Font to use (see table below) |
| `font_color` | Text color in RGB565 hex |
| `text` | The string to display |

### Font names

| `font_name` value | Script | Example text |
|-------------------|--------|-------------|
| `tf` | Latin / default | `Hello, World!` |
| `chinese` | Simplified Chinese | `你好世界` |
| `arabic` | Arabic (RTL) | `مرحبا بالعالم` |
| `cyrillic` | Cyrillic | `Привет мир` |
| `devanagari` | Devanagari | `नमस्ते दुनिया` |

Multiple font entries can appear in the same `texts` array, so a card can mix Latin labels with Chinese characters.

### RGB565 quick-reference

| Hex | Color |
|-----|-------|
| `0x0000` | Black |
| `0xFFFF` | White |
| `0xF800` | Red |
| `0x07E0` | Green |
| `0x001F` | Blue |
| `0xFFE0` | Yellow |
| `0x07FF` | Cyan |
| `0xF81F` | Magenta |

---

## 7. Testing Your Game

### In the web simulator (DiceMaster Studio)

DiceMaster Studio lets you run and interact with games in a browser without hardware. It simulates all six screens, injects shake events via a button, and lets you rotate the virtual dice.

See [DiceMaster_Studio/docs/guides/running-studio.md](../../DiceMaster_Studio/docs/guides/running-studio.md) for setup and usage.

### On hardware (Raspberry Pi)

Follow [DiceMaster_Central/docs/runbooks/deploy.md](../../DiceMaster_Central/docs/runbooks/deploy.md) to push code to the Pi and start the system.

Quick reference:

```bash
# On your development machine — push changes
git push

# On the Pi — pull and restart
ssh dice1 'cd ~/DiceMaster/DiceMaster_Central && git pull'
ssh dice1 'sudo systemctl restart dicemaster'
```

Because the workspace is built with `--symlink-install`, Python file edits take effect immediately after a restart. A full rebuild is only needed when you change `setup.py`, `package.xml`, or message definitions.

### ROS2 debugging

These commands run on the Pi after SSHing in:

```bash
# List all active topics
ros2 topic list

# Watch motion events in real-time (shake the dice to see output)
ros2 topic echo /imu/motion

# Watch orientation changes (rotate the dice to see output)
ros2 topic echo /chassis/orientation

# Watch what is being sent to screen 1
ros2 topic echo /screen_1_cmd

# Check which nodes are running
ros2 node list
```

If your strategy is not loading:

- Confirm the file is at `strategies/my_strategy/my_strategy.py` (directory and filename match).
- Confirm the class has `_strategy_name = "my_strategy"`.
- Confirm `config.json` has `"strategy": "my_strategy"`.
- Look for load errors in the game manager logs: `ros2 topic echo /rosout`.

---

## 8. Where to Go Next

| Resource | Path |
|----------|------|
| Full API reference | `DiceMaster_Central/docs/api/` |
| Game creator deep-dive | `DiceMaster_Central/docs/creator/` |
| Architecture overview | `DiceMaster_Central/docs/api/architecture.md` |
| Web simulator docs | `DiceMaster_Studio/docs/guides/running-studio.md` |
| Deploy runbook | `DiceMaster_Central/docs/runbooks/deploy.md` |
| Example games source | `DiceMaster_Central/src/dicemaster_central/examples/` |
| Hello Dice walkthrough | `docs/beginner_game_guide.md` |

The `examples/` directory contains two fully-worked games worth reading:

- **`chinese_quizlet`** — a Chinese vocabulary quiz using the same orientation+shake pattern as the advanced example in this guide. Good to compare against your own implementation.
- **`test`** — a minimal timer-based game that cycles through all six screens. Useful as a smoke-test when debugging hardware.
