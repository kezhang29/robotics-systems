# Robotics Systems
This repository is practice for implementing classic control algs and motion planning. The goal is to understand the math behind it
and also learn how to write the sim for each.

## PID
PID is a feedback based control system that calculates the error between the setpoint and goal position. 
The controller then applies three components:
### Proportional (P)
This constant reacts to the current error. A larger error means a stronger correction. 
### Integral (I)
Accumulates past error over time. Mainly used to help the system when it undershoots.
### Derivative (D)
Predicts future error by looking at rate of change. This constant reduces overshoot.

After running the PID sim, export the logs as a CSV file and plot the graphs in matlab:
```
data = readtable('pid_data.csv');

% Create figure with subplots
figure('Position', [100, 100, 1200, 800]);

% Position vs Time
subplot(2,2,1);
plot(data.Time, data.Position, 'b-', 'LineWidth', 2);
hold on;
yline(100, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position');
title('Position vs Time');
legend({'Position', 'Target'});
grid on;

% Error vs Time
subplot(2,2,2);
plot(data.Time, data.Error, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Error');
title('Error vs Time');
grid on;

% Control Output vs Time
subplot(2,2,3);
plot(data.Time, data.Output, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('PID Output');
title('PID Output vs Time');
grid on;
```

Graph with default PID params:

<img width="1196" height="826" alt="Screenshot 2026-02-16 at 4 35 09 PM" src="https://github.com/user-attachments/assets/bb4f8d45-9550-438e-ba80-ebff73bf1ad1" /> 

This is one with a high kP that caused oscillation:

<img width="1144" height="758" alt="Screenshot 2026-02-18 at 2 32 08 PM" src="https://github.com/user-attachments/assets/08bbe3bc-453e-4fb9-8c71-b0ebd37823a4" />


## Bang-bang control

A bang-bang control scheme is a feedback cntrol strategy that switches adruptly between two discrete states, based upon a setpoint rather than output. This is most commonly used in flywheel control. Bang-bang control is inadequate for tasks like chassis control and lift mechanims, but can be found in real life in mechanisdms such as a thermostat. The idea here in the thermostat example is when the room is below a target temperature, the heating system will try to heat the room until the room is at target temperature. If the room is above a target temperature, the heating system will try to cool the room down. The target temperature can be defined as a target area (a range) to prevent oscillation. 




## Odometry
Odometry is a way to track the robot's position and orientation on the field using tracking wheels. The position tracking works 
by modeling the robot's motion as arcs over infinitely small time intervals. 
<img width="698" height="313" alt="Screenshot 2026-02-22 at 3 11 34 PM" src="https://github.com/user-attachments/assets/13b87be6-d37d-494b-90f0-69a53ea94ba8" />

Consider the following robot. 

<img width="724" height="358" alt="Screenshot 2026-02-22 at 3 12 12 PM" src="https://github.com/user-attachments/assets/790bf9cd-a3cb-410e-bf79-51c926182ad9" />

Let the left arc and right arc traveled by the tracking wheels be concentric with a circle with radius $r_a$. Let $s_L$ and $s_R$ denote the distances from
the left and right tracking wheels to the center of the robot respectively. Then,
$\Delta{L} = (r_a + s_L) \theta$ and $\Delta{R} = (r_a - s_R) \theta$
where $\theta$ is in radians. From here, we can get two equations for $r_a$, which are $r_a = \frac{\Delta{L}}{\theta} - s_L$ and $r_a = \frac{\Delta{R}}{\theta} + s_R$. 
Setting these equal gives $\boxed{\theta = \frac{\Delta{L}-\Delta{R}}{s_L + s_R}}$. This calculation is optional, most robots use an IMU sensor.

Now we calculate the position vector of the robot. 

<img width="352" height="366" alt="Screenshot 2026-02-22 at 3 37 42 PM" src="https://github.com/user-attachments/assets/4c1be6c7-5c25-49a4-ab42-f1961c5c8199" /> <img width="439" height="326" alt="Screenshot 2026-02-22 at 4 02 28 PM" src="https://github.com/user-attachments/assets/af28e7e4-a682-4fe4-9e25-3096e7296968" />

Consider the chord between the center of the robot before moving and after the movement. Let this be the y axis of our local coordinate system. Notice this is rotated
from the robot's initial "forward" by $\theta/2$. Thus the local y axis translation is $2 \sin(\theta/2) (\frac{\Delta{R}}{\theta} + s_R)$. We can calculate translation along the x axis in a similar way. 
Doing so gives our position vector as $2 \sin(\theta/2) (\frac{\Delta{S}}{\theta} + s_S, \frac{\Delta{R}}{\theta} + s_R).$ However, this is offset by $\theta_0 + \theta/2$ from the global coordinate system, where $\theta_0$ is the pervious robot orientation. Thus we rotate our local position vector by that amount and the global translation vector can be calculated.

### Usage
Given a coordinate system that we can base our robot on, the robot doesn't have to drive in straight lines anymore. Essentially the idea now is we have a target point and a point that our robot is at currently (from our odom)
and we calculate the distance and the angle needed to get there. Then PID does the rest of the work. Here is a simulation of this **drive to point function**:

<img width="300" height="300" alt="Screenshot 2026-02-28 at 1 21 57 PM" src="https://github.com/user-attachments/assets/d485e077-b325-4999-b06d-a803e1e611cc" /> 

<img width="300" height="300" alt="Screenshot 2026-02-28 at 1 24 32 PM" src="https://github.com/user-attachments/assets/7c1e488e-0827-454d-b611-9cb93e3b34e3" />

You can see that in the second image, the position that odometry is reading is offset from the robot's real coordinates. 

## Pure Pursuit
Pure pursuit is an automatic steering method for differential drive robots. Given a path represented as a series of waypoints, the controller calculates the angular velocity necessary to keeo the robot on the path, while keeping linear velocity constant or controlled by a PID algorithim. 

The main idea is to project a circle around a robot and the point it intersects along the path is the goal point, or carrot, the the robot chases. This circle's radius is known as the ```lookAheadDistance```.

### Line-Circle Intersection

If we offset the system to the origin, we can use the discriminant to determine whether or not intersections exist.

<img width="652" height="474" alt="Screenshot 2026-04-04 at 4 51 42 PM" src="https://github.com/user-attachments/assets/4f3b2264-2ac8-4b72-8881-761f64736bee" />

Here if the ```discriminant >= 0```  then there exists valid solutions. Intersections are only valid if they fall within the bounding box of the segment, i.e.:
```
min(x1,x2) <= sol_x <= max(x1,x2)
min(y1,y2) <= sol_y <= max(y1,y2)
```

### Goal Point Selection

In the case of two valid segments on a path, we prefer the one closer to the next waypoint on the path. To prevent the robot from backtracking, a ```lastFoundIndex``` variable keeps track of the previous found point. This prevents the robot from following previous segments on the path. Also, an additional safegaurd is to only select the goal point if it's closer to the next waypoint than the robot's current position. 


## Resources
Purdue Sigbots Wiki     
[https://wiki.purduesigbots.com/software/odometry]        
[https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit]




