# Quest Name
Authors: Qi Luo, Yihe Bi

Date: 2022-12-9
-----

## Summary
In this quest, we made a fob which can control our buggy remotely. There is a button on the fob. With one click it will active the led. Double click the button, the buggy will turn left. 3 clicks it will right, and 4 clicks it will do a trick: go forward and backward, just like the dancing mode on a tesla model X.

## Self-Assessment
This is the quest! We are finally here!

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design
We wired everything up exactly the same as we did in the quest. However, this time we count the number of clicks in 2s, and the number of clicks to determine which message should be send out by the IR LED (as specified in the summary). Also, removed a ESP32 from one of the fob we build, and connect everything on that board to the ESP32 on our buggy. That way, this board works as a receiver, and the other board will work as the fob to control the buggy.

As the receive gets the message for IR LED, it will read the second char in the message. This char determines wheather to active the LEDs on the buggy, or to make it turn left and right, or to make it do a trick. This is implemented by several switch statements within the while loop in the led task, the servo task, and the steering task. In each of the tasks, the switch statement will have 5 cases. 4 of them is in response to the led, turns, and clicks, the 5th one is default. Thus, we make sure that the input of a signal will immediatly trigger a certain behavior of the buggy.

Investigative question (2 ways to hack the system and how to prevent it):

1. Have another fob with a similar set up, and send out infinite(this means many) messages, and see which message gets the buggy to response. I can prevent this by controlling the behavior of this buggy using all 4 char, and I'll encrypt the 4 char using a bloom filter, so that each char is used as a 8 bits binary number. Combine all 4 gives me a 32 bits password, which is not that easy to hack using the way provided above.

2. Have a strong IR signal presented the enviroment to defect the fob. I could not think of a perfect way to solve this. However, I believe adding more IR led to the fob, or just increase the power of the IR transmitter in the fob will definitely mitigate this issue.

## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to video demo](https://drive.google.com/file/d/1kC_7x7a2MK0svQuI30b9-5rpFJxmQR2p/view?usp=sharing). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References
https://github.com/BU-EC444/bu-ec444-whizzer/blob/Fall-2022/quests/primary/carfob.md
-----

