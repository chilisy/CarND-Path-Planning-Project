# Model Documentation

## Overview
My approach for this project is based on the approach from Aaron and David in the walkthrough, so it contains a spline path generator and a finite state maschine to change lane with a cost function to determine which lane should be driven.
The model has mainly three parts:

1. Sensor data processing
2. Behavior planning
3. Trajectory generation

### Sensor data processing
The date provided from the simulator contains following values for each object: 
- id
- x
- y
- vx
- vy
- s
- d

From all objects found, only the vehicles on track around ego vehicle are of interest.

If an object found in lane 0 to 2, they will be added to lane cost objects and processed later.
```c++
void add_vehicle(sensor_obj obj)
```

The planner object also search after vehicles
ahead, left and right of the ego vehicle and mark them for further processing.
```c++
void getSensorData(vector<vector<double>> sensor_data)
```


### Behavior planning

#### Finite State Machine
For this project, a finite state machine with following five states is used to control the ego vehicle:
- Keep Lane (KL)
- Prepare Change Lane Left (PCLL)
- Prepare Change Lane Right (PCLR)
- Change Lane Left (CLL)
- Change Lane Right (CLR)

KL can transit to PCLL or PCLR, if the cost of left or right lane to the ego vehicle is less than the ego lane cost.
PCLL and PCLR can transit to CCL or CCR, if no objects left or right are dectected. If cost function of left or right lane exceed the ego lane cost in this state, PCLL and PCLR transit back to KL.
CLL and CLR can only transit to KL whenever the change lane maneuver is completed.

#### Cost function
There are 2 costs modeled for each lane of the highway:

- cost of inefficiency
If the car drives lower speed than max allowed speed, the cost of inefficiency increases. If the speed is greater than max allowed speed, cost goes to maximum. 
```math
cost = 1-exp(-(max_v-lane_speed)/max_v)
```

- cost of prefered lane
The middle lane is defined as prefered lane and if ego lane is either left or right, the cost increases. Since we only have three lanes, the cost is modeled as:
```math
cost = 1-exp(-abs(current lane - prefered lane))
```

### Trajectory generation
The trajectory generation is basicly the same as Aaron and David sugguested in walkthrough video. The spline lib from Tino Kluge was used to calculate a spline using anchor points we provide. After the spline is calculated, we obtain the y-value for each x-value to generate the trajectory. 




