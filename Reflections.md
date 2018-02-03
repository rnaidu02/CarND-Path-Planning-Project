# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
## Compilation
Code compiles without errors.

## Valid trajectories
1. Car drives more than 4.3 miles without any incident.
2. Car drives little less than 50  mph most of the time. It slows down when it is stalled by another car in the front or at the time of changing the lanes.
3. Car's acceleration and jerk are within the limits.
4. Car didn't hit any other cars.
5. Car stays in the lane and only moves from one lane to the other when it needs. The transitions from one to the other didn't take more than 3 sec.
6. Car is changing the lanes when there is a slow moving car in the front and at the same time there is a lane that is clear of traffic (for about 30m front and behind) for safe transition.

## Reflections

This project code consists of three important steps to generate the path trajectory for the car. Here are the steps: Prediction, Behavior planning, and Trajectory generation.

### Prediction
The function `predictCarsInLanes()` inside `main.cpp (lines 165 - 230)` has an implementation that predicts if there will be any cars in the lanes within 30m distance into the future at the end of projected steps (usually at the end of 50 steps - that is approximately for an frenet s of 30 m). This may not be very accurate way of predicting the vehicles into the future but an approximation that works.

Here is the pseudo code for the logic:
```
Input:  sensor_fusion: sensor fusion data from the Simulator
        current_lane: Current lane position (0, 1, 2)
        prev_path_size: the size of prev_path that is not used by the vehicle from prev generated path trajectory
        car_s: s position of the car at the end of the prev_path list
        range: How many meters safe distance the car has to maintain to avoid collision

Output: returns a vector of size 3 with each element represents the lane and tells if there is a vehicle within safe distance
```

```
  For each vehicle in sensor fusion list
    Find velocity, frenet s, d values foe each vehicle
    Estimate the future Frenet s values for the vehicle based on the current s value and speed.
    For each lane on the road
      If there is a vehicle in front of ego vehicle within given range
        tag that there is a vehicle in the current lane
      If there is a vehicle in the front or behind ego vehicle within a given range in other two lanes
        tag that there is a vehicle within safe Distance

    Return the vector of size 3 with lane occupancy information.

```


### Behavior planning
Once the prediction of other vehicles is done wrt occupancy in the lanes, the next step would be to find out what would be the right move for the ego vehicle. In this planning the ego vehicle will decide on whether to be in the same lane and travel at speed limit , or slow down if there is a vehicle in the front and no lane change is possible, or move to either left or right lanes to maintain the speed.

This logic is implemented inside `returnChangedLanewithUpdatedSpeed()` function in `main.cpp (lines 233 - 296)`.

Here is the pseudo code for the function:

```
 This function takes the current lane position of the car along with lane occupancy of other cars within the lanes, reference velocity, target velocity, acceleration details and it returns the new lane position if it make any lane change. If the lane is not changed it essentially either increases the ref_velocity or decreases the ref_velocity.

  Input:
           current_lane: current lane position
           lane_occupancy: lane occupancy information
           ref_velocity: the current car_speed (mutable)
           target_velocity: the speed limits
           increment_Step: the increment/decrement in speed
  Output:
           returns the updated lane position
```

```

  If there is a vehicle in the same lane as the ego vehicle
    If the current lane is left lane
      If there is no vehicle in middle lane within 30 m Distance
        Change the current lane to middle lane
      else
        Slowdown the ego vehicle

    IF the current lane is middle lane (either lane change possible)
      If there is no vehicle in left lane within 30 m Distance
        Change the current lane to left lane
      else If there is no vehicle in right lane within 30 m Distance
        Change the current lane to right lane
      else
        Slowdown the ego vehicle

      If the current lane is Right lane
        If there is no vehicle in Middle lane within 30 m Distance
          Change the current lane to middle lane
        else
          Slowdown the ego vehicle
  else (If no vehicle in the front of ego vehicle)
    increase the speed of ego vehicle as long as it is within speed limit

    return the updated lane

```

### Trajectory generation
Once the lane is identified and the speed at which ego vehicle should go, trajectory generation is the next step for the ego vehicle. Using the prev path x and y points, and Fernet way points - create a list of x and y nodes. Convert these nodes from global to car co-ordinate system. Using these nodes create a spline that can predict y given x (Frenet s) (main.cpp - lines 428-490).

Now add all of the prev_x and prev_y values to the next_x and next_y values for the ego vehicle trajectory. The overall trajectory points for the ego vehicle is set to 50 and in this step there is a need to generate (50 - length of prev x/y) values. The remaining way points are generated based on the current speed of the ego vehicle and the distance that need to be covered (30 m). Once these points are generated, these are sent to the simulator.

The code for this logic is in `main.cpp (lines 428 - 530)`.

Here is the pseudo code for the Trajectory generation:

```
  // Ref ref_velocity
  target_velocity = 50.0;
  // Increment step of the velocity (0.224 mph -> results in 5 m/sec^s acceleration)
  increment_Step = 0.224*2;
  waypoint_spacing = 30;

  If prev_path_size is less than 2
    generate two data points based on the current and the car_yaw
  else
    generate two data points based on the last two prev_path_points

  Generate the way points for the next 30, 60, and 90 m from the current position

  Convert these data points to car co-ordinate system

  Use these data points to generate a spline that can be used to determine the next d (y) for a given s (x).

  Taking an s of 30m. Find the the d value.

  For this distance, generate N data points, where N depends on current velocity, distance it travels for once cycle (20 msec).

  Divide the 30 m into N equal parts (s_prime).
  Find out how many more data points are required in addition to prev_path_points to get to 50. Let us say this is K.
  Generate S, d points for the K steps (with each step increase by s_prime, and generate equivalent d). Once K data points are generated, convert them to global co-ordinate system

  Add the prev_points and newly generated K points to the next_x and next_y list.

  Send this list to the simulator.

```
