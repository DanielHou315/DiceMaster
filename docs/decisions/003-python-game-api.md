# ADR-003: Strategy Pattern with BaseStrategy for the Game API

## Status

Accepted

## Context

DiceMaster's primary target audience includes educators — university instructors, language teachers, and learning designers — who want to create interactive classroom games without deep knowledge of robotics middleware. At the same time, the system is built on ROS2, which exposes a Node-based API that requires understanding of executors, topic subscriptions, lifecycle states, and message types.

The design challenge is to expose enough power for educators to write meaningful games (react to shaking, display content on specific screens, handle orientation changes) while hiding the ROS2 plumbing entirely from their code. The API must also be safe to swap at runtime: the game manager needs to start and stop strategies reliably, and a broken strategy should not crash the screen managers or IMU pipeline.

A secondary concern is that strategies are discovered and loaded dynamically from a configured directory at runtime via Python's `importlib`, so the loading mechanism needs a stable base class to introspect.

## Decision

Use the **Strategy design pattern** with an abstract base class `BaseStrategy` that inherits from `rclpy.node.Node`. Game authors subclass `BaseStrategy` and implement two abstract methods:

- `start_strategy()` — called automatically at construction time. The author sets up subscriptions, timers, and initial screen content here.
- `stop_strategy()` — called by the game manager when switching games. The author tears down timers and any ongoing work here.

The base class handles:
- Constructing the ROS2 node with a guaranteed-unique name (`strategy_{game_name}`) to avoid node name collisions
- Loading the game's JSON config file and assets directory before `start_strategy()` is called, so both are available as `self._config` and `self._assets` when the author's code runs
- The `_running` flag and lifecycle sequencing

The game manager discovers strategy classes by scanning a configured directory for `.py` files, importing them with `importlib.util`, and finding the first class that is a subclass of `BaseStrategy` but is not `BaseStrategy` itself.

A minimal working strategy looks like:

```python
from dicemaster_central.games.strategy import BaseStrategy
from dicemaster_central_msgs.msg import MotionDetection

class VocabQuiz(BaseStrategy):
    _strategy_name = "vocab_quiz"

    def start_strategy(self):
        self.create_subscription(
            MotionDetection, '/imu/motion',
            self._on_motion, 10
        )
        self._show_question(0)

    def stop_strategy(self):
        pass  # subscriptions are cleaned up with the node

    def _on_motion(self, msg):
        if msg.shaking:
            self._next_question()
```

## Consequences

**Positive:**
- Educators never import `rclpy` directly. The fact that a strategy is a ROS2 node is an implementation detail they need not know.
- The two-method lifecycle (`start_strategy` / `stop_strategy`) maps naturally to "game is active" and "game is being switched out," which is conceptually intuitive for non-engineers.
- Asset loading and config parsing happen before the author's code runs, so `self._assets` and `self._config` are always ready when `start_strategy()` is entered.
- The dynamic discovery mechanism (`importlib` scan) means adding a new game requires only dropping a `.py` file in the games directory; no registration step or restart is needed.
- The class introspection approach (`issubclass(attr, BaseStrategy)`) is robust: it does not rely on naming conventions or explicit registration.

**Negative:**
- Because `BaseStrategy` inherits from `Node`, it carries the full ROS2 node overhead. A strategy with no sensors and only a simple timer still instantiates a DDS participant. For the current hardware target this is acceptable.
- The lifecycle is simple (no explicit pause/resume states). A strategy that needs to suspend mid-game must manage its own internal state flags.
- Errors in `start_strategy()` (e.g., a missing asset) propagate as exceptions that the game manager must catch. An uncaught exception in a strategy will prevent the game from loading but will not affect the rest of the ROS2 graph.

## Alternatives Considered

**Raw ROS2 Node subclassing**
Requiring educators to subclass `rclpy.node.Node` directly would expose executor management, callback group configuration, and message type imports. This is the correct approach for ROS2 engineers but is a significant barrier for the educator audience.

**Lua scripting**
A Lua-scripted game layer (similar to how some game engines expose scripting) would provide true isolation between game logic and the host process, enabling sandboxing and hot-reload without Python import machinery. However, it would require a Lua runtime, a Python-Lua bridge, and custom bindings for all DiceMaster-specific APIs. The development and maintenance cost was judged disproportionate to the current scale of the project.
