# CarND-Controls-PID - WriteUp

## Implementation
First of all I have implemented in the "PID::UpdateError()" and the "PID::TotalError()" the basic functionality of a PID controller.
Then I tried manual tuning. As this is very time consuming I have implemented the twiddle algorithm to find fitting PID gains. Then I tried to extend the twiddle algorithm by stepwise decreasing the lateral limit.

## Manual Tuning
Starting with Kp = 1 the car was able to control the vehicle for some distance, but started to oscillate a lot. (see ./doc/Kp_1_Kd_0_Ki_0.mp4) The proportional part corrected high CTEs, but was not able keep the car stable in the center.

Adding the Kd=1 the oscillation could be reduced.(see ./doc/Kp_1_Kd_1_Ki_0.mp4) It decompensates overshooting when the CTE get's smaller.

The Ki did not helped at all (see ./doc/Kp_1_Kd_1_Ki_1.mp4) as expected as we have no bias in the simulation model.

So PD controller would be the most beneficial in regards to manual tuning. I tried out some values and ended up with:
- Kp = 0.5
- Kd = 0.5
- Ki = 0

With that the car was able to travel one lap, but was very much oscillating. (see ./doc/Kp_0_5_Kd_0_5_Ki_0.mp4)

## Twiddle
So as in the lecture showed I did implement the twiddle algorithm here as well. In the main.cpp I have a check if the CTE>pid.latLimit to see if the car has left the lane. In such cases a reset of the simulation and twiddling the PID controller gains are necessary. This is implemented in PID.cpp-PID::Twiddle(). Instead of calculating the RMS of the CTEs I have used a counter to see how far the car has travelled as the goal was to be able to drive at least one lap.

First the lateral limit was set to 2.5m. The output of the twiddle is following:

|counter| Kp| Kd| Ki| Dp|  Dd|  Di| maxCounter|
|-|-|-|-|-|-|-|-|
| 382 | 0.1 | 0.0 | 0 | 0.1 | 0.1 | 0.1 | 382|
| 440 | 0.1 | 0.1 | 0 | 0.11 | 0.1 | 0.1| 440|
| 54 | 0.1 | 0.1 | 0.1 | 0.11 | 0.11 | 0.1 | 440 |
| 376 | 0.21 | 0.1 | 0 | 0.11 | 0.11 | 0.09 | 440|
| 755 | 0.1 | 0.21 | 0 | 0.099 | 0.11 | 0.09 | 755|
| 54 | 0.1 | 0.21 | 0.09 | 0.099 | 0.121 | 0.09 | 755|
| 551 | 0.199 | 0.21 | 0 | 0.099 | 0.121 | 0.081| 755|
| 776 | 0.1 | 0.331 | 0 | 0.0891 | 0.121 | 0.081| 776|
| 56 | 0.1 | 0.331 | 0.081 | 0.0891 | 0.1331 | 0.081 | 776|
| 1062| 0.1891 | 0.331 | 0 | 0.0891 | 0.1331 | 0.0729| 1062|
| 12056 | 0.1891 | 0.4641 | 0 | 0.09801 | 0.1331 | 0.0729 | 12056|

Also in the twiddle algorithm we can see that the Ki gain is not required. Now the car is controlled safely within the lane. (see ./doc/twiddle.mp4)

## extended Twiddle
Although the car is already driving safely in the lane I wanted to see how far the twiddle algorithm could optimize the gains if the lateral limit is stepwise reduced each time we are able to travel one lap. (see main.cpp lines 110-121). By that I was expecting to get more and more smoother controlling. But starting from the values gathered above is was only able to to tune parameter up to 2.2m max CTE.

The output of the extended twiddle is following:

Kp| Kd| Ki| Dp|  Dd|  Di| latLimit|
-|-|-|-|-|-|-|
0.1891 |0.4641 |0 |0.09801 |0.1331 |0.001 |2.2
0.09109 |0.5972 |0 |0.088209 |0.1331 |0.001 |2.2
0.09109 |0.4641 |0.001 |0.088209 |0.11979 |0.001 |2.2
0.179299 |0.4641 |0 |0.088209 |0.11979 |0.0009 |2.2
0.09109 |0.58389 |0 |0.0793881 |0.11979 |0.0009 |2.2
0.09109 |0.4641 |0.0009 |0.0793881 |0.107811 |0.0009 |2.2
0.170478 |0.4641 |0 |0.0793881 |0.107811 |0.00081 |2.2 

That are my best tuned parameters:
- Kp = 0.170478
- Kd = 0.4641
- Ki = 0

Now the simulation looks like this: see ./doc/extendedTwiddle.mp4