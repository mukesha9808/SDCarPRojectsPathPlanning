## Highway Driving Path planning Project

The goals / steps of this project are the following:

* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
* Max Acceleration and Jerk are not Exceeded. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* The car must not come into contact with any of the other cars on the road.
* The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
* The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.


---
[//]: # (Image References)

[image1]: ./output_images/Result.jpg "Higway driving performance"



## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points

---

## Designing path planner
In this section, I will explain my implementation for path planning algorithm step by step

###Code Files organisation
Most of implementation of path planning algorithm is implemented in pathplanner.h. I have also included header file spline.h for spline method for jerk minimisation.

### Simulation Performance
![alt text][image1]

### Design Implementation
I have designed my implementation is following steps:
* Determine all the possible trajectories
* Determine cost of each trajectory
* Pick the trajectory with least cost
* Determine future action
* Create smooth path for chosen trajectory

#### 1. Determine possible trajectories
To determine, possible trajectories, first we need to find the lane on which currently being driven. My function `getlanenumber()` gives line number with input d coordinate. When Car is on middle lane it can go to any lane of three lane. But, if car is on left most lane then car can either stay on left most lane or can go to middle lane. Similarly, if car is on right most lane then then car can either stay on right most lane or can go to middle lane. This implementation can be found on function `determinefutureTrajectory()`.

#### 2. Determine Cost 
We need to establish cost of each possible path. To determine cost we need to loop through each possible trajectory and determine the cost. I have defined two functions for that. Function `keepLane_cost()`, determine cost of staying on current lane while, function `laneChangeCost()`, determine cost of changing lane either left or right.

##### 2.1 Cost of staying on lane
For staying on same lane, all the vehicles on same lane is analysed and cost is updated based on sensed measurement from sensor fusion data. I have used used remaining points in my current trajectory pipeline for cost. I am using car_s0 as current s position of ego car while car_s is last point in my current pipeline. As function inputs gives only car_s, I am estimating car_s0 using car_a and ego car velocity. Similarly s position of car in that iteration is defined as s_check0 and I have extrapolated this s point to time when ego car would be end position of  pipeline and define as s_check. After estimating cost for each sensed vehicle in lane, highest cost is considered as cost for staying on same lane.
* Cost of vehicle cutting into lane
There is chances that car could cut into lane and lead to collision. So my logic always look if  vehicle current s-position in between ego car's current s-position and at end of trajectory pipeline. When this condition is true, coliision risk is high and cost is set to maximum (i.e. 0.8)

* Cost for vehicle outside region 
If car would be far ahead at end of current path ( >100m) or behind the car then cost would be 0

* Cost for vehicle coming closer
If gap between cars at end path is narrow (20m or less) then collision risk is very high. Cost is set to very high (0.75). Cost here is not set to max because this difference is used to edetrmine whether car needs be decelerated or emergency braking needed

* Cost for vehicle is region
I have defined cost function for car in horizon based on gap between sensed cars and ego car at end path. Cost function can be defined as
$$ Cost_{KeepLane} = 1 - e^{-(\frac{25}{Gap})^2} $$

###### 2.1.2 Bias cost for lateral collision
I have taken into consideration that cost should be higher if any vehicle from adjacent lane is coming closer to change lane. I defined this as bias cost and add into cost estmeted above. This cost is estimated using gap between d-positions.
$$ BiasCost_{KeepLane} = 1 - e^{-(\frac{3}{\abs{d-Gap}})^2} $$

##### 2.1 Cost for changing lane
Function `laneChangeCost()` is defined to estimate cost for changing lane in either direction. Lane number of final lane is given as input to determine cost desired side. To determine cost for changing lane, first cost of staying on that lane is determined using above function `keepLane_cost()`. Then additional cost for transition is determined using sensor fusion data.
* Cost for vehicle outside region
If vehicle is too behind from ego car(100m or more or ahead of ego car (10m or more) then transition cost is minimal (0.05). I have not used 0 cost just to discourgae lane change is not really needed.

* Cost for vehicle really close
If car is really close (10m either ahead or behind) then lane chane is very risky and cost is really high. I have assigned cost 1.

*Cost for vehicle in region
I have defined cost function for car in horizon based on gap between sensed cars and ego car at end path. Cost function can be defined as
$$ Cost_{LaneTransition} = 0.5 *(1 - e^{-(\frac{15}{Gap})^2} )$$

#### 3 Choose best trajectory
After iterating through all possible trajectory, trajectory with least cost is choosen and next state is defined based on selection. This is impleted in function `target_state()` which will be explained in next section.

#### 4 Determine future action
Function `target_state()` defines next action taken by ego car to create trajectory. This function iterate though tractories and find outputs future state using least cost of trajectory. Future state/target state is defined with following values
* 0- Keep on same lane and slow down
* 1- Keep on same lane and maintain speed
* 2- Change lane to right 
* 3- Change lane to left
* 9- Emergency braking

##### 3.1 Change Lane to Left
If most desirable path is left to current lane then target state outputs is 3 and this output selects final lane number as current lane -1 in main function for trajectory creation

##### 3.1 Change Lane to Right
If most desirable path is right to current lane then target state outputs is 2 and this output selects final lane number as current lane +1 in main function for trajectory creation

##### 3.1 Stay on cuurent lane
When it is determined that staying on same lane is most efficient then there could be three possible scenario
* Speed limit could be maintained 
* Slow down as vehicle in front is slowing down and changing lane is not safe
* Brake hard to avoid collison

This desired behavior is achieved using cost of staying on lane. Cost is less (<0.7) then target state is 1 to instruct path planner to maintain speed and reference speed in main function is accelerated if speed limit is not reached. If cost is greater than 0.7 then first as bias 0.25 is added so that if lane change is feasible then it could be selected. Please note that cost for staying lane is saturated at 0.9.
If cost is greater than 0.75 then target state is 9  for emergency braking and reference velocity is reduced quickly else target state is 0 which will reduce speed slowly. Reference valocity is saturated between 0.01 and speed limit to avoid divide by zero is trajectory point creation.


#### 4 Create smooth path for chosen trajectory
Once we know our target lane for next instant, we create points in that lane add to pipeline. To make path smooth with minimum jerk, spline method is used. Spline method is uses defined waypoints and create a smooth path to fit those points.

##### 4.1 Choose waypoints
To create spline path, 5 spaced points are taken. First two points are taken from last two points in current pipeline. If current pipeline does not have points then current car position is considered and point in past along the direction is determined. Another three points taken at distance of 30m spacing in target lane to feed into spline.

##### 4.2 Transform points on vehicle frame
Points are transformed using shift and rotation to dtermine equally spaced points

##### 4.3 Craete spline
Trasformed points are fed into spline function and x,y coordinates are determined at equal spacing in 30m distance. Number of points will depend on vehicle speed. If speed is high points will be less and number of points will be higher when speed is less

##### 4.4 Revert to global frame
All the output points are transformed to global coordinate frame using rotation and shift

##### 4.5 Add points to the pipeline
Pipeline has constant 50 points. Reamining points in pipeline is used and points from new trajectory added to remaining space in pipeline. Other points are ignored once pipeline is full.
