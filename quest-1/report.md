# Quest 1: Four Legged Walker
Authors: Qi Luo, Yihe Bi, Zhenglei Jiang

Date: 2022-09-23
-----

## Summary

In this skill quest, our team mounted 2 separate servos and 1 alphanumeric display on ESP32 board and created a moving bug walker with the components. The two servos were tuned to move synchronously and a countdown is displayed to signify the time until the next move. Besides, we manage to create an console interface for the unser to interact with our servo walker.

## Self-Assessment
We put a lot of effort into this assignment. We believe that we've learned everything the skill and quest intended to teach us. If I am the grader, I would give us a full grade. :) please~

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Must Keep track of time (no time loss) | 1 |  1     | 
| Must report time every second | 1 |  1     | 
| Must use two servos | 1 |  1     | 
| *(optional)*  incorporate battery | 0 |  1     | 



### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design
Our servo walker uses a bread board as its body, and it has two servo attached to it. There’s also an display installed on the top of it to show the time till next step and a counter to show how many steps it has taken.

Display: We use the modulus we built in the bitmap skill and slightly modified it. We are using I2C the communicate between the ESP32 and the bitmap. The pin in use is SCL, SDA, 3.3V and GND on ESP32. The leftmost digit on the bitmap we use it to display the number of steps. The right two digit we use as a count down.

Timer: We use the modulus we built in the Stopwatch skill and add two variables to it. One of them (time_counter) is counting down on time, and the other one (period) is measuring if the the walker should take a step.

Servo: We build a servo_task() function to tune the servo. We use the function mcpwm_set_duty_in_us() to control the servoe to rotate to a certain postion. Then, we determine when to rotate the servo (to take a step) using the information from Timer.

Legs: Details are in our video.

User Interface: We use the UART ports, namely the function gets, to obtain an time (in char) from the user. Then we convert it into a int, and input it to our system. It is stored in the variable period, which is mentioned above.

## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>

![Image](images/IMG_0423.PNG)


## Supporting Artifacts
- [Google Drive Video Demo](https://drive.google.com/file/d/1cUpGdLC1NAZQgrequCBTbGhFNz1tXkDn/view?usp=sharing).


## Modules, Tools, Source Used Including Attribution
I2C display Driver: https://github.com/BU-EC444/code-examples/tree/Fall-2022/i2c-display

## References
https://github.com/BU-EC444/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control

-----