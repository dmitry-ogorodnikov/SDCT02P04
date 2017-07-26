# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID components

1. 'P' - proportional factor. The output of the proportional factor is the product of gain and measured error (crosstrack error). Hence, larger proportional gain or error makes for greater output from the proportional factor. Setting the proportional gain too high causes a controller to repeatedly overshoot the setpoint, leading to oscillation.

2. 'I' - integral factor. The output of integral factor is the product of gain and integrated error (the sums of all the previous measured errors). Due to limitation of P-factor where there always exists an offset between the process variable and set point, I-factor is needed, which provides necessary action to eliminate the steady state error. It integrates the error over a period of time until error value reaches to zero.

3. 'D' - differential factor. The output of differential factor is the product of gain and the rate of change of the measured error. The derivative component causes the output to decrease if the process variable is increasing rapidly.


## Tuning hyperparameters

I used combination of manual tuning and twiddle. First I set approximate values of components so that the vehicle does not leave the track for 2,000 measurements in case when the throttle value equal 0.3. Then I used twiddle algorithm (which was described in the class) for components. When I applied twiddle algorithm I used 6000 measurements and the throttle value was 0.5.

I added two modes for driving: `FAST` and `NORMAL`. In `NORMAL` mode, the throttle value is equal 0.4. In `FAST` mode, the throttle value is equal 0.5. 
