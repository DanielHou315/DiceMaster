---
applyTo: '**'
---
To build projects using ROS, go into DiceMaster_ROS_Workspace and run the following commands:

```bash
source prepare.sh
colcon build --symlink-install
```

Then you can run testing either with ros2 run or with python.

For DiceMaster_Central, you should put all test files in tests/