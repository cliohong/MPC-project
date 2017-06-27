
# CarND-Controls-MPC




In this project we'll implement Model Predictive Control to drive the car around the track. 
This simple kinematic model consists of four variables, namely x coordinate, y coordinate, heading direction and velocity. The actuators consist of the steering angle and the acceleration (negative or positive). 

>there's a 100 millisecond latency between actuations commands on top of the connection latency.


The following animation shows the result of how mpc works on to keep the car running on the track all the time.

![](mpc.gif)

---

###  MPC algorithm:

**Setup** :

     1. Define the length of the trajectory,N, and duration of each timestep, dt.
     2. Define vehicle dynamics and actuator limitations along with other constraints.
     
**Loop** :

     1. We pass the current state as the initial state to the model predictive controller.
     2. We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called Ipopt.
     4. Back to 1.


### THE STATE VARIABLES:

* px:  the current x position of vehicle based on global map coordinate system.
    
* py:  the current y position of vehicle based on global map coordinate system.
    
* psi: the orientation of the vehicle in radians.
    
* v: current velocity of vehicle in mph.
    

this given to us by the simulation everytime we ask for. In addition, udacity provided the lack_track_waypoints.csv file which has waypoints of the lack track, we can use this to fit a polynomial and see how well our model track curves perform.  It is known that a 3rd degree polynomial can be good enough to estimate most road curves. 

----

### ACTUATIONS  :  *STEERING,  THROTTLE,  and BRAKE*

### Here are two modes of acutation we can used to control our vehicle.

`delta`:

 * this is the steering angle which I restricted it to be between +- 25
   
    
`a` :

  * this is a throttle/brake value which represents the acceleration or decceleration of vehicle. In my code, I restricted it to be between +- 1 which is as same as the simulator exptected. 
  

## Cost Function and Penalty Weights

Given the current state of vehicle and the reference trajectory we want to follow, we need to minimize the cost function, by constantly optimize actuations,  to find the lowest cost predictive trajectory. 

Here are some factors we put into account:

   *  minimize the cross track error `cte` and the heading error `epsi` to penalize the vehicle not maintaining the reference state.
   
   *  minimize the velocity error `v` to penalize the vehicle not maitaining the reference velocity.
   
   *  minimize the difference between the next actuator state ( `delta`,  `a` ) and current one to maitain a smooth driving condition.
  

The penalty weights are determined through trial-and-error, taking into consideration that tuning the mpc wieghts will result in a smoother moving and steering transitions. When I increased the weight cost of absolute steering angle large enough and noticed that the path that the vehicle was taking is less jagged. Also I increased the weight cost for the consecutive acceleration difference to consider not letting the vehicle turn too late at road curves, especially when at the sharp turn. 


**Here are the weights I ended up with**


```python
cte_cost_weight=1

epsi_cost_weight=1

v_cost_weight=1

delta_cost_weight=10

a_cost_weight=10

delta_chaneg_cost_weight=500

a_change_cost_weight=20


```


## Tuning timesteps `( N )` and timesepts duration`( dt )`

I originally used `N=15, dt=0.2`, because I thought `3s (15*0.2)` would be a good prediction span. However I found that the time span was too slow to react,and the car ran out of the track when it was at a sharp turn. Then I eventually settled on `10` for `N `,and `dt=0.12` which meant that I was only predict for 1.2 seconds essentially, and the result looked so good.  



## Model Predictive Control with Latency
as we have to take the **100ms** latency into consideration, thus the approach here we take is by using the **kinematic model** starting from the current state for the duration of the latency. 

---

## Kinematic Model

here is the simplified version of how our vehicle works in the real world based on physics. How the variables update based on time elapse`(dt)`, the current state (`px` ,`py`, `psi`, `v`), and the actuations (`delta`, `a`).

**Lf**:
   *  this is the length from the front of vehicle to its center of gravity. Based on this simulator, it is set to the value of 2.67.

current_px= v*dt*cos(delta)

current_py= 0.0 

current_psi= v/Lf *(-delta)*dt

current_v= v+a*dt

current_cte=cte+ v*sin(epsi)*dt

current_epsi=epsi+v*(-delta)/Lf *dt


state<<current_px,current_py,current_psi,current_cte,current_epsi


```python

```
