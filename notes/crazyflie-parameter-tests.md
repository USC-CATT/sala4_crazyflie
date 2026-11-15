# experiments to evaluate crazyflie parameters

we need to estimate a few things

1. crazyflie rotational inertia
2. the pwm to thrust function

## rotational inertia

we're going to use the [bifilar suspension 
method](https://myphysicsclassroom.in/to-determine-the-moment-of-inertia-of-a-body-using-bifilar-suspension-method-with-parallel-strings-threads-lab-manual/)
to estimate the rotational inertia of the system. 

we need to be able to hang the drone from two strings that are parallel

three orientations to test all three primary axes
three configurations:
full setup
without blade guards
without propellers

we will use the method without propellers to estimate the rotational inertia of 
the propellers using the [parallel axis 
theorem](https://en.wikipedia.org/wiki/Parallel_axis_theorem)

### equipment needed

- string to hang the drone
- a level
- a stool?
- tape measure for long measurements
- calipers for short measurements
- timer/stopwatch
- the drone

## PWM to Thrust

The way this is calculated in the code is a little weird. It acknowledges there 
is a delay between pwm and thrust due to motor dynamics, but it still back 
calculates from thrust all the way to pwm... we would like to also map 
rotational speed $\omega$ to thrust, but we can at least make an attempt at 
mapping pwm to thrust

## Propeller Speed time constants

The gazebo simulator treats the motor speed as a simple first order time 
constant, but there's a different time constant to speed up as there is to slow 
down. We want to design an experiment to estimate these time constants. 

This sort of requires that we have access to the ESC data, since we need to 
actually measure the $\omega$ of the rotor to see this change. We can estimate 
it via acceleration if we can measure the position over time, though, if we make 
some assumptions about the relationship between speed and thrust, which we're 
already doing above, so that seems like a fair step.












