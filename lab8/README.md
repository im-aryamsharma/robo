# Overview
This repo is for a smart car for the CSCI 1062U Ontario Tech University class. This robot, based on the lab goals, will be able to solve a simple maze, make use of most sensors aboard (ultrasonic, line, gyro senosrs and servos). The code is spilt up into multiple files, labN.ino, setup.hpp, setup.cpp. setup.hpp contains the declarations of functions used within the setup.cpp and has all the helper functions used to run the code in order to not over populate the labN.ino.

# How to install
```
git clone https://github.com/im-aryamsharma/robo.git
```
All the updated code is in the latest lab folder

# Prerequisites
1. A completed [ELEGOO Smart Robot Car Kit V4.0](https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial)
2. Any IDE that can compile to the Arduino Uno board (i.e. [Arduino IDE](https://docs.arduino.cc/software/ide/))


# How to run
1. Compile & upload the code into the Arduino UNO.
2. (Opt.) Make sure the robot is on a dark floor with white tape.
3. After waiting 5 seconds, press the button to start the program.
4. If you want to stop the program midway, press the red button to reset the robot. If you wish to start the same program again, go back to step 2.

# Explanation
In it's current state the robot follows a state based design pattern where state is as follows  

### Lab 5  
0. Tells the robot to go in a straight line until the ultrasonic sensors detect a wall, if the  distance to the wall is slightly futher than the MIN_DISTANCE constant it will back up, turn to a random direction and continue on state 0, if the wall is closer than MIN_DISTANCE than the robot will switch to state 1.
1. State 1 will allow the robot to follow the object directly in front of it and it's speed will be based on the distance to the object. If it loses sight of the object it will then switch to state 2.
2. State 2 will make the robot "check" it's surroundings and revert back to state 0 once it's sequence of move's is complete.
3. State 3 is a line following mode and will revert to state 0 once the line has disappeared on all 3 line sensors. 

### Lab 7+  
0. Tells the robot to go in a straight line until the line sensors detect a white line after which it switches to state 1.
1. Turn 90 degrees clockwise and return back to state 1.

### NOTES
In every lab state -1 is to test functions and the robot will never automatically switch to state -1.

# Future fixes
[  ] Make the Gyro assisted driving work \
[  ] Re-write verbose sections of the code \
[  ] Following one style guide \
