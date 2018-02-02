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


### Behavior planning

### Trajectory generation
