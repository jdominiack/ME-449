This directory contains all of the code and results of my ME 449 capstone project. This software 
controls a simulated youBot robot to pick and place a small cube and was developed in 
accordance with the process outlined in the assignment page. The software is split in three 
primary sections: robot kinematics, reference trajectory generation, and feedback control. 

The robot kinematics section (represented by the “NextState” function) takes in the current 
configuration of the robot as well as an array of commanded velocities to calculate the state of 
the robot after a given time step. In the case of this project we assume that the robot’s velocities 
stay constant over the small (dt =0.01) timestep. This function also implements velocity limiting, 
so that the angular velocity of the robot arm joints and the drive wheels never surpass a user 
defined boundary. 

The reference trajectory generation section creates point-to-point trajectories that define the 
robot end effector’s desired motion. These trajectories are generated between user defined 
transformation matrices that dictate the desired spatial configuration of the end effector at key 
points such as initialization, cube grabbing, cube lifting, and cube placing. 

The feedback control section uses a feedforward + PI control loop to calculate robot velocities 
with which to command the robot. The FeedbackControl function finds the error between the 
end effector’s current position and its current desired position (given by the trajectory generation 
function) to generate a twist that will control the end effector towards the next desired 
configuration within a user defined time step. Users can tune the behavior of this feedback 
control system by changing the user-defined Kp and Ki gain matrices to alter the proportional 
and integral components of the controller.

Results:
I collected results for 3 different conditions: a best case feedback response, an overshoot 
feedback response, and a new task with altered cube positions. For the best case feedback 
response I found that with Kp and Ki gains of 3 and 0.6 respectively, the system achieves a near 
optimal behavior which avoids overshoot and causes the error in the end effector’s position to 
quickly fall to 0 within the first couple seconds. For the overshoot feedback response I found 
that for Kp and Ki gains of 1.25 and 1.35 respectively, I could get the system to exhibit fairly 
significant and undesirable overshoot, clearly visible on the plot of the components of the end 
effector error. Finally, for my new task I moved the initial configuration of the cube to be further 
from the robot and moved the final configuration to be just behind the robot’s initial position. This 
task mimicked “retrieving” the cube and bringing it back to the robot’s starting location. I 
encountered no issues with the code’s response to altered cube configurations. 
Note: I did not attempt to implement joint limits on the youBot within my software.
