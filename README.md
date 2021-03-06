                   
# Warmup Project
Eamon O'Brien and Emma Pan

## Teleop
Our teleop code uses the keys w, a, s, d, and space to drive the robot. To stop the neato, we used the space key. W is forward, a turns the neato counterclockwise, s is backward, and d is clockwise. The neato will continue in the direction of a pressed key until a different key is pressed. To drive the neato, we created a publisher with the topic `/cmd_vel`. We used this publisher to publish an angular velocity for neato rotation, and to publish a linear velocity to drive the neato forward or backward. Below is a diagram of our teleop command key mapping.

![teleop-image](https://github.com/epan547/warmup_project/blob/master/media/teleop.png)


## Drive Square
![drive-square-video](https://github.com/epan547/warmup_project/blob/master/media/drive_square.gif)

Our drive square implementation utilizes the `rospy.sleep()` functionality to time different behaviors. To continuously drive in a square, the script runs a `while` loop, in which  the neato is instructed to drive forwards 1 meter, and then rotate 90 degrees counterclockwise. To rotate the neato in a right angle in the two seconds that this command is sleeping, we divided the radians of a 90 degree turn (pi/2 radians) by 2 (seconds), resulting in an angular velocity of pi/4. Since the forward velocity and time spent moving forward is always the same, the neato always moves forwards the same distance between right-angle rotations, thus creating a square. 

## Wall Following

<img src="https://github.com/epan547/warmup_project/blob/master/media/wall_follow.jpeg" width="300">

Our wall following is achieved with a proportional controller. We detect the angle between the neato and the wall by measuring at two distances, 90 degrees away from each other, and calculating the error between them. If d1 is greater, the neato will rotate clockwise, so that d1 and d2 become equal with each other, and the neato becomes parallel with the wall. Conversely, if d2 is greater, then the neato will rotate counter-clockwise to compensate. The linear velocity of the neato is kept at a constant value. In this way, as a neato drives forwards along a wall, it rotates until one side is parallel with the wall. We experimented with a PID controller, but have not successfully implemented it yet. Our current implementation works well when the neato starts at an angle, but breaks down in circumstances where the wall is on another side of the neato, or when the neato is in an empty world. For example, if a neato is heading towards a wall head-on, it will not be able to detect it.

## Person Following

![person-follow](https://github.com/epan547/warmup_project/blob/master/media/person_follow.gif)

For person-following, we were able to get surprisingly robust performance from a very simple method. Our code takes a full 360° laser scan, replaces any values of infinity with 1000, and passes the data through a gaussian filter. If plotted, this filtered data looks like a line at 1000 which dips down like a bell curve at each angle where the neato detected an object. The closer or larger the object, the lower the value. In a non-simulated environment, the gaussian filter would help mitigate the effect of noise in the laserscan. From the minimum of this filtered data, we obtain a distance and angle for where the neato predicts a person to be.

Once the neato has a prediction of the person's location, we calculate an angular and a linear error, which give us the difference between where the neato currently is and where we want it to be. The angular error is the difference between the desired angle for the neato to face a person, zero, and the actual angle between it and the desired orientation. The linear error is the difference between the predicted distance from the person, and our desired distance, which we set to 0.5 meters. If both errors were 0, our neato would be exactly half a meter from the nearest object, and facing directly toward it.

We use a proportional controller to drive the angular and linear velocity of the neato based on these two errors. This makes the neato move efficiently and quickly towards an object when it is placed anywhere in its range. When it reaches its target distance, it jitters back and forth a bit, because there is currently no damping in the system. If we had more time, our first step to fix that would be to put a derivative term into our controller.

## Obstacle Avoidance

For our implementation of obstacle avoidance, we have written a script for avoiding obstacles (`obstacle_avoid_1.py`), and a script for navigating towards a given target point (`obstacle_avoid.py`). Unfortunately, due to a bug in our GPU for gazebo, we lost a significant chunk of testing time on Saturday and Sunday, and dind't get a chance to integrate these two behaviors. 

Our script for navigating the neato away from obstacles is the same as our person following script, except that our error is based on the maximum value of the filtered scan instead of the minimum. This way, the neato is instructed to move towards the emptiest direction in its view. Objects repel it more as they get nearer, larger, and more closely grouped.

Our script for navigating the neato towards a given target point in the global reference frame utlizes odometry data. By subscribing to the neato's `/odom` topic we can find the position and angle of the neato relative to the world’s coordinate frame. This is enough information to set an arbitrary target in space, and navigate towards it.

In order to get the orientation of the neato in the global reference frame, we used the odometry orientation data `msg.pose.pose.orientation`, and converted it from quaternion to euler, for simplicity of calculations. By printing these values, we discovered that the orientation coordinate frame is 180 degrees rotated from the world frame. We fixed this by simply subtracting the odom orientation angle value from 180 degrees: 
  `self.rotation = 180 -             math.degrees(euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0])`
  
To find the angle between the neato’s position and the origin, we used `math.atan2()`, a python function which accounts for the limited range of the arc-tangent function. We added 180 to that angle so that this heading would point in the direction that the neato should travel to get to the target, instead of from target to neato.
   `self.vector_to_target = 180 + int(math.degrees(math.atan2(self.y,self.x)))
       if self.vector_to_target > 360:
           self.vector_to_target = self.vector_to_target - 360`


Finally,to calculate the difference between the neato's heading and the heading that would direct it towards the target, we used simple subtraction of these two angles. The resulting 'angular error' is what we input to proportional control, to get the neato's angular velocity:
  `self.angular_error = -(self.rotation - self.vector_to_target)`

To calculate the distance from the neato to the target point, we first calculated the difference between the neato's and the target's current x and y positions: 
   `diff_x = self.target_x - global_x
    diff_y = self.target_y - global_y`

Next, we used the pythagorean theorem, to get the actual distance between the neato and its target. This 'linear error' is what we input to proportional control, to calculate the neato's linear velocity: 
  `self.linear_error = math.sqrt((diff_x**2) + (diff_y**2))`
  
Similarly to the person following code, we put this angular error and a linear error into two proportional controllers and were pleasantly surprised by the performance.

In order to integrate these two functionalities, our plan was to use the target heading to influence the gaussian-filtered data that the neato uses to avoid obstacles. If the neato is using this data to identify the maximum point (farthest away), and the target angle's distance is increased by a sizeable amount, then the neato should navigate towards the target, unless an obstacle is in its way, subtracting from the gaussian curve, and causing the neato to steer away.



## Finite State Behavior

In our finite state controller, we created a class for the neato which encapsulates multiple behaviors. On each interaction of the loop, it checks a parameter called self.state, which tells it which behavior to follow. Each behavior is a separate function.
The included behaviors are teleop, go to origin, and drive in square. It begins in teleop by default, and returns to it when it completes the other two behaviors, as shown in the diagram:

![finite-state-diagram](https://github.com/epan547/warmup_project/blob/master/media/rsz_finite_state_diagram.jpg)

In this structure, we could easily incorporate additional states by assigning them to the numbers 3-9.

## Code Structure
  
The code was mostly organized in an object-oriented manner. Each node is initialized in a class, and each subscriber is defined in its own method. The publisher for velocity is updated and published in a method called “run”, wrapped in a while loop that continues until the user inputs `ctrl + C`.
For all object-oriented scripts, the class object is initialized and its functionality is run in an `if __name__ == “__main__”` function. We have commented the code and used descriptive variable names where possible.

## Challenges

The learning curve for this project was extremely steep for both of us, since neither of us had much ROS experience before this class. Initially, it was a challenge to figure out each stage of the project. The aspect of the project that was most difficult, and took more than half our time, was getting odometry information, and translating it between coordinate frames. At first, we were running up against a bug in the gazebo environment, but after that was fixed, the information was still challenging to use. It took a while to realize the orientation was stored as a quaternion, and another while to figure out how to use that.

Once we were able to establish the angle between a target and our robot, the distance between them, and the orientation of the neato relative to the world, we had to do some fairly tricky geometry to make our controller guide it to the target.

## Improvements

We like the way that we implemented the finite state controller, with command keys for each behavior. The next thing we would do is add the rest of the behaviors we programmed to the finite state script. The big advantage this gives us is the ability to use our teleop to get the robot into position for debugging, without resetting gazebo or dragging the robot around and confusing the odometry.

We also never managed to avoid obstacles very well. When we’re able to get the laser scan working again, we will have odometry and scanning working at the same time, and should be able to combine them to make something effective. So far, we have never had both of those working at the same time.
Lastly, if given more time, we would have converted the proportional controller in our person-following and wall-following scripts to be a PD or PID controller. The script in which we attempted this before moving on for the sake of time can be found in our ‘scripts’ folder, named ‘wall_follow_pid.py’.

# Key Takeaways   

**Find the documentation**: For a few of our challenges, we struggled because we had small syntactical errors that let the program run (ie. not providing the marker with enough information, or using the wrong subscriber name), and couldn’t figure out what was wrong. If we had found the documentation for these objects earlier, we likely would have had a smoother debugging process later.

**Program more incrementally**: There are a lot of things that can go wrong in this situation, with multiple environments and scripts speaking to each other asynchronously to accomplish a complex task. We were writing our behaviors somewhat incrementally, but if we had written even smaller blocks of code and tested more frequently, we probably would have made faster progress.
