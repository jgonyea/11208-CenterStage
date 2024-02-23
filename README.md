# FTC Center Stage 
## Kardia Robotics, 2023

This branch may contain the RoadRunner code and autonomous OpMode(s) at some time in the future.

## Setup
* Create a new empty directory (for the whole "project")
* Open a terminal or git bash
* `cd` into that directory
* Run the following command:
  ```
  git clone -b quickstart1 --depth 1 --single-branch https://github.com/acmerobotics/road-runner-quickstart.git . && rm -rf README.md .git* && cd TeamCode/src/main/java/org/firstinspires/ftc && rm -r teamcode && git clone -b auto https://github.com/jgonyea/11208-CenterStage.git teamcode && cd ../../../../../../..
  ```
* The directory is now an FTC SDK with RoadRunner installed, and with this branch cloned to teamcode.
