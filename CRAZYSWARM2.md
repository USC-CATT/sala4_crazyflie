# Crazyswarm2

We've found that we will need to modify Crazyswarm2 to fit our custom controller based needs. Since Crazyswarm2 offers a python backend, which is simply a thin shim between the official crazyflie library and the Crazyswarm2 python library, it's easiest to modify that to suit our needs.


> [!IMPORTANT]  
> It is important to note that these modifications do NOT work in simulation, only on the actual drone. Do not go on wild goose chases figuring out why the drone isnt working like you think it should in RViz without trying on the actual crazyflie drone.

## Data pipeline
The data pipeline of the Crazyswarm2 cflib backend, including its ROS2 python api and controlling on the actual drone looks like this:

 `Crazyswarm2 (ROS2 API) -> crazyswarm2/crazyflie_py/crazyflie.py -> crazyswarm2/crazyflie_sim/crazyflie_server.py -> cflib -> Drone`

> [!NOTE]
> The `crazyflie_server.py` file does not seem to run on the `sim` backend, even though it is located inside of the `crazyflie_sim` library. It does seem to work on the actual drone.

### Crazyflie.py
The crazyflie.py file is the actual ROS2 API used within ROS2 python files. It is responsible for sending data to the `crazyflie_server` via ROS2 topics.

### Crazyflie_server.py
`crazyflie_server.py` is responsible for subscribing to all of the various ROS2 topics that `crazyflie.py` could send on, and converting them into Crazyflie commands via `cflib` directly.