# DiceMaster

Master repository for the DiceMaster project: a programmable multi-screen dice prototype designed for language learning. 

**Note** This repository is open for alpha access! We are hard at work on getting the documentation ready and hardware models so you can build your own. Stay tuned! 

**SeriousPlay 2025** We attended the SeriousPlay 2025 conference to present our prototype! We will be back in the future with our pedagogical experiments, student feedback, and more thoughts!

## Submodules 

**DiceMaster_Central**: home to code and resources running on the Raspberry Pi.
**DiceMaster_ESPScreen**: as the name suggests, this is home to code that runs on the ESP32 screen driver.
**DiceMaster_ROS_workspace**: this is a git-tracked ROS2 workspace for building the code on the Raspberry Pi.
**DiceMaster_Studio**: (not maintained anymore) home to a GUI tool for processing files and designing games. 


## Miscellaneous

1. `scripts/hardware_optimizer.py` was used to minimize the internal size of the dice with a quadratic program. Unfortunately, our naive stacking design is very close to the optimal that it does not make sense to optimize any further. 