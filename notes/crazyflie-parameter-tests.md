# experiments to evaluate crazyflie parameters

we need to estimate a few things

1. crazyflie rotational inertia
2. the pwm to thrust function

## rotational inertia

we're going to use the [bifilar suspension method](https://myphysicsclassroom.in/to-determine-the-moment-of-inertia-of-a-body-using-bifilar-suspension-method-with-parallel-strings-threads-lab-manual/)
to estimate the rotational inertia of the system. 

we need to be able to hang the drone from two strings that are parallel

three orientations to test all three primary axes
three configurations:
full setup
without blade guards
without propellers

we will use the method without propellers to estimate the rotational inertia of the propellers using the [parallel axis theorem](https://en.wikipedia.org/wiki/Parallel_axis_theorem)














