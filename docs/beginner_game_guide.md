# Beginner's Guide to Creating Your First DiceMaster Game

> **Prerequisites:** This guide assumes you're familiar with Python and ROS2. If you need a refresher, check out [Python Basics](https://docs.python.org/3/tutorial/) and [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html).

## What is a DiceMaster Game?

A DiceMaster game consists of two parts:
1. **Game Directory**: Contains a `config.json` file and an `assets/` folder with your media (images, text files)
2. **Strategy**: A Python class that defines your game's behavior (what happens when the dice is shaken, rotated, etc.)

When the DiceMaster starts, it discovers all available games and strategies, then launches your game automatically.

## Quick Start: "Hello Dice" Game

Let's create a simple game that displays a greeting on all screens and changes the message when you shake the dice.

### Step 1: Create the Game Directory

Create the following structure in the examples directory:

```bash
mkdir -p DiceMaster_Central/dicemaster_central/examples/games/hello_dice/assets
```

Your structure should look like:
```
examples/games/hello_dice/
├── config.json
└── assets/
    ├── greeting.json
    └── shaken.json
```

### Step 2: Create the config.json

Create `examples/games/hello_dice/config.json`:

```json
{
    "game_name": "hello_dice",
    "strategy": "hello_strategy",
    "strategy_config": {}
}
```

This tells the game manager:
- The game is named "hello_dice"
- It uses the strategy called "hello_strategy" (we'll create this next)
- No additional configuration needed (empty `strategy_config`)

### Step 3: Create Text Assets

Text assets are JSON files that define what appears on the screen.

Create `examples/games/hello_dice/assets/greeting.json`:

```json
{
    "bg_color": "0x0000",
    "texts": [
        {
            "x_cursor": 100,
            "y_cursor": 200,
            "font_name": "tf",
            "font_color": "0xFFFF",
            "text": "Hello, DiceMaster!"
        },
        {
            "x_cursor": 80,
            "y_cursor": 280,
            "font_name": "tf",
            "font_color": "0x07FF",
            "text": "Shake me to continue..."
        }
    ]
}
```

Create `examples/games/hello_dice/assets/shaken.json`:

```json
{
    "bg_color": "0x001F",
    "texts": [
        {
            "x_cursor": 120,
            "y_cursor": 220,
            "font_name": "tf",
            "font_color": "0xFFE0",
            "text": "You shook me!"
        }
    ]
}
```

**Understanding the format:**
- `bg_color`: Background color in RGB565 hex format (0x0000 = black, 0xFFFF = white)
- `texts`: Array of text elements to display
- `x_cursor`, `y_cursor`: Position on 480x480 screen (0,0 is top-left)
- `font_name`: Font to use ("tf" is the default font, others: "chinese", "arabic", "cyrillic", "devanagari")
- `font_color`: Text color in RGB565 hex format
- `text`: The actual text to display

**RGB565 Color Quick Reference:**
- `0x0000`: Black
- `0xFFFF`: White
- `0xF800`: Red
- `0x07E0`: Green
- `0x001F`: Blue
- `0xFFE0`: Yellow
- `0x07FF`: Cyan
- `0xF81F`: Magenta

### Step 4: Create the Strategy

Create the strategy directory:

```bash
mkdir -p DiceMaster_Central/dicemaster_central/examples/strategies/hello_strategy
```

Create `examples/strategies/hello_strategy/hello_strategy.py`:

```python
"""
Hello Strategy - A simple introductory game for DiceMaster.

This strategy displays a greeting on all screens and changes the message
when the dice is shaken.
"""

import os
from dicemaster_central.games.strategy import BaseStrategy
from dicemaster_central.config import dice_config
from dicemaster_central.constants import ContentType
from dicemaster_central_msgs.msg import ScreenMediaCmd, MotionDetection


class HelloStrategy(BaseStrategy):
    """
    A simple strategy that demonstrates basic DiceMaster concepts:
    - Publishing text to screens
    - Subscribing to motion detection
    - Managing game state
    """

    # This must match the strategy name in your game's config.json
    _strategy_name = "hello_strategy"

    def __init__(self, game_name: str, config_file: str, assets_path: str, verbose: bool = False):
        super().__init__(game_name, config_file, assets_path, verbose)

        # Get all available screen IDs (1-6)
        self.available_screen_ids = list(dice_config.screen_configs.keys())

        # Track whether we've been shaken
        self.has_been_shaken = False

        # Publishers dictionary (created as needed)
        self.screen_publishers = {}

        # Will hold our motion subscription
        self.motion_subscription = None

        self.get_logger().info(f"HelloStrategy initialized for game '{game_name}'")

    def _get_screen_publisher(self, screen_id: int):
        """Get or create a publisher for a specific screen."""
        if screen_id not in self.screen_publishers:
            topic_name = f'/screen_{screen_id}_cmd'
            self.screen_publishers[screen_id] = self.create_publisher(
                ScreenMediaCmd,
                topic_name,
                10
            )
            self.get_logger().info(f"Created publisher for {topic_name}")
        return self.screen_publishers[screen_id]

    def _display_on_all_screens(self, file_name: str):
        """Display the same content on all screens."""
        file_path = os.path.join(self._assets_path, file_name)

        if not os.path.exists(file_path):
            self.get_logger().error(f"Asset file not found: {file_path}")
            return

        for screen_id in self.available_screen_ids:
            msg = ScreenMediaCmd()
            msg.screen_id = screen_id
            msg.media_type = ContentType.TEXT
            msg.file_path = file_path

            publisher = self._get_screen_publisher(screen_id)
            publisher.publish(msg)

        self.get_logger().info(f"Displayed '{file_name}' on all screens")

    def _motion_callback(self, msg: MotionDetection):
        """Handle motion detection messages."""
        if msg.shaking and not self.has_been_shaken:
            self.has_been_shaken = True
            self.get_logger().info("Shake detected! Changing display...")
            self._display_on_all_screens('shaken.json')

    def start_strategy(self):
        """
        Called when the strategy starts.
        Create subscriptions, publishers, and display initial content.
        """
        # Subscribe to motion detection
        self.motion_subscription = self.create_subscription(
            MotionDetection,
            '/imu/motion',
            self._motion_callback,
            10
        )

        # Display initial greeting
        self._display_on_all_screens('greeting.json')

        self.get_logger().info("HelloStrategy started - waiting for shake!")

    def stop_strategy(self):
        """
        Called when the strategy stops.
        Clean up subscriptions and publishers.
        """
        # Destroy subscription
        if self.motion_subscription:
            self.motion_subscription.destroy()
            self.motion_subscription = None

        # Destroy all publishers
        for publisher in self.screen_publishers.values():
            if publisher:
                publisher.destroy()
        self.screen_publishers = {}

        self.get_logger().info("HelloStrategy stopped")
```

**Understanding the Strategy Code:**

1. **Class Definition**: Your strategy must inherit from `BaseStrategy` and set `_strategy_name` to match config.json

2. **`__init__`**: Initialize variables, get screen IDs from config, but don't create ROS subscriptions yet

3. **`_get_screen_publisher`**: Helper to lazily create publishers for each screen topic

4. **`_display_on_all_screens`**: Helper to show the same content on all 6 screens

5. **`_motion_callback`**: Handles messages from the IMU - detects shaking

6. **`start_strategy`**: Called automatically when your game starts - create subscriptions and show initial content

7. **`stop_strategy`**: Called when your game stops - clean up resources

### Step 5: Test Your Game

1. **Set your game as the default** by modifying `DiceMaster_Central/dicemaster_central/dicemaster_central/config.py`:

```python
class GameConfig:
    # ... other settings ...
    default_game = "hello_dice"  # Change this line
```

2. **Build the ROS workspace**:

```bash
cd DiceMaster_ROS_workspace
source prepare.sh
colcon build --symlink-install
```

3. **Launch the system**:

```bash
ros2 launch dicemaster_central dicemaster.launch.py
```

4. **Test it**: You should see "Hello, DiceMaster!" on all screens. Shake the dice and watch the message change!

**NOTE**: the dice will start this default game on startup if auto-start is enabled!

## Understanding Key Concepts

### Screen Management

DiceMaster has 6 screens (IDs 1-6). Each screen has its own ROS topic:
- `/screen_1_cmd`
- `/screen_2_cmd`
- ... and so on

To display content on a screen, publish a `ScreenMediaCmd` message to its topic.

### Media Types

The `ContentType` enum defines what you can display:
- `ContentType.TEXT` (0): JSON text files
- `ContentType.IMAGE` (1): JPG/PNG images
- `ContentType.GIF` (2): Animated GIF directories

### Available Sensor Topics

Your strategy can subscribe to:
- `/imu/motion`: Motion detection (shaking, movement)
  - Message type: `MotionDetection`
  - Fields: `shaking` (bool), `moving` (bool)

- `/chassis/orientation`: Current dice orientation
  - Message type: `ChassisOrientation`
  - Fields: `top_screen_id`, `bottom_screen_id`, and screen poses

### Asset Loading

When your strategy initializes, `self._assets` contains a dictionary of all files in your assets folder (created by `load_directory`). You can use this to discover available content programmatically.

## Next Steps: More Complex Patterns

### Pattern 1: Timer-Based Updates

Add a timer to change content periodically:

```python
def start_strategy(self):
    # Create a timer that fires every 2 seconds
    self.update_timer = self.create_timer(2.0, self._timer_callback)

def _timer_callback(self):
    # Do something every 2 seconds
    self.get_logger().info("Timer fired!")

def stop_strategy(self):
    if self.update_timer:
        self.destroy_timer(self.update_timer)
```

### Pattern 2: Orientation-Aware Displays

Use different content for top/bottom/side screens:

```python
def __init__(self, ...):
    super().__init__(...)
    self.top_screen_id = None
    self.bottom_screen_id = None
    self.side_screen_ids = []

def _chassis_callback(self, msg: ChassisOrientation):
    """Update screen assignments when dice rotates."""
    self.top_screen_id = msg.top_screen_id
    self.bottom_screen_id = msg.bottom_screen_id

    # Side screens are all except top and bottom
    self.side_screen_ids = [
        sid for sid in self.available_screen_ids
        if sid not in [self.top_screen_id, self.bottom_screen_id]
    ]

    # Display different content based on position
    self._update_displays()

def start_strategy(self):
    self.chassis_subscription = self.create_subscription(
        ChassisOrientation,
        '/chassis/orientation',
        self._chassis_callback,
        10
    )
```

### Pattern 3: Using Images

Display an image instead of text:

```python
# In your assets folder, add an image: assets/my_image.jpg

def _display_image(self, screen_id: int, image_name: str):
    """Display an image on a specific screen."""
    image_path = os.path.join(self._assets_path, image_name)

    msg = ScreenMediaCmd()
    msg.screen_id = screen_id
    msg.media_type = ContentType.IMAGE
    msg.file_path = image_path

    publisher = self._get_screen_publisher(screen_id)
    publisher.publish(msg)
```

### Pattern 4: Animated GIFs

For GIFs, create a directory with the `.gif.d` extension containing numbered frames:

```
assets/
└── animation.gif.d/
    ├── 0.jpg
    ├── 1.jpg
    ├── 2.jpg
    └── 3.jpg
```

Then display it:

```python
def _display_animation(self, screen_id: int):
    gif_path = os.path.join(self._assets_path, 'animation.gif.d')

    msg = ScreenMediaCmd()
    msg.screen_id = screen_id
    msg.media_type = ContentType.GIF
    msg.file_path = gif_path

    publisher = self._get_screen_publisher(screen_id)
    publisher.publish(msg)
```

## Learning from Examples

The `examples/` directory contains working games you can study:

### chinese_quizlet Game

A language learning game that demonstrates:
- Loading multiple "cards" (subdirectories with answer.json and images/)
- Subscribing to both motion and orientation
- Displaying questions on top, answers on bottom, hint images on sides
- Implementing shake detection with cooldown period
- Randomizing and cycling through content

**Key files to study:**
- `examples/games/chinese_quizlet/config.json`
- `examples/games/chinese_quizlet/assets/` (structure with multiple cards)
- `examples/strategies/shake_quizlet/shake_quizlet.py`

### test Game

A minimal testing game that demonstrates:
- Simple timer-based strategy
- Publishing to screens in sequence
- Using the notification builder utility

**Key files to study:**
- `examples/games/test/config.json`
- `examples/strategies/pipeline_test/pipeline_test.py`

## Debugging Tips

### Check ROS Logs

Monitor your strategy's output:
```bash
ros2 topic echo /rosout
```

### Verify Topics

List active topics to ensure your publishers are working:
```bash
ros2 topic list
```

### Test Motion Detection

Monitor the motion topic:
```bash
ros2 topic echo /imu/motion
```

### Common Issues

1. **Strategy not loading**:
   - Ensure your file is named `{strategy_name}/{strategy_name}.py`
   - Verify `_strategy_name` matches the strategy name
   - Check that your class inherits from `BaseStrategy`

2. **Assets not found**:
   - Use `os.path.join(self._assets_path, filename)` for file paths
   - Verify files exist with correct names
   - Check `self._assets` dictionary to see what was loaded

3. **Nothing displays on screens**:
   - Verify screen IDs are correct (1-6)
   - Check topic names: `/screen_{id}_cmd`
   - Ensure `file_path` in your message is absolute and exists
   - Look for error messages in logs

4. **Motion detection not working**:
   - Verify IMU node is running: `ros2 node list | grep imu`
   - Check motion topic has data: `ros2 topic echo /imu/motion`
   - Ensure you subscribed to the correct topic name

## Helper Utilities

### Notification Builder

For quick text messages, use the notification builder:

```python
from dicemaster_central.utils import build_info_notification

def _show_message(self, screen_id: int):
    msg = build_info_notification("Hello from helper!", screen_id)
    publisher = self._get_screen_publisher(screen_id)
    publisher.publish(msg)
```

Available functions:
- `build_info_notification(content, screen_id)` - White background, black text
- `build_warning_notification(content, screen_id)` - White background, orange text
- `build_error_notification(content, screen_id)` - White background, red text

### Data Loader

Your strategy automatically has `self._assets` populated with all valid files in your assets directory. The structure mirrors your file system:

```python
def start_strategy(self):
    # Print all discovered assets
    self.get_logger().info(f"Available assets: {self._assets}")

    # Access nested structure
    if 'images' in self._assets:
        for img_name, img_path in self._assets['images'].items():
            self.get_logger().info(f"Found image: {img_name} at {img_path}")
```

## Where to Go From Here

1. **Experiment**: Modify the hello_dice game to display different text or images
2. **Explore**: Study the chinese_quizlet strategy to see advanced patterns
3. **Create**: Build your own educational game (vocabulary, math facts, trivia, etc.)
4. **Share**: Contribute your game back to the examples directory!

## Additional Resources

- Protocol documentation: `docs/protocol.md`
- Architecture overview: `DiceMaster_Central/dicemaster_central/docs/architecture.md`
- CLAUDE.md: Development reference for this codebase
- ROS2 Message definitions: `DiceMaster_Central/dicemaster_central_msgs/`

Happy coding! 🎲