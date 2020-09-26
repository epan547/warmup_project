# Warmup Project
Eamon O'Brien and Emma Pan

## Teleop

Our teleop code uses the keys w, a, s, d, and space to drive the robot. W is forward, a turns the neato left on the spot, s is backward, and d is right. The neato will continue in the direction of a pressed key until it is told ot stop. The stop key is the space bar.

## Wall Following

Our wall following is acheived with a proportional controller. We detect the angle between the neato and the wall by measuring at two angles, and calculate our error. We experimented with a PID controller, but have not successfully implemented it yet.

## Person Following

For person following, we were able to get surprisingly robust performance from a very simple method. Our code takes a full 360° laser scan, replaces any values of infinity with 1000, and passes the scan through a gaussian filter. If plotted, this filtered data looks like a line at 1000 which dips down like a bellcurve at each angle where the neato detected an object. The closer or larger the object, the lower the value.

The neato then calculates an angular error and a linear error. The angular error is the difference between the desired angle between the neato and the nearest object, zero, and the actual angle between it and the minimum value in our filtered data. The linear error is the difference between the distance detected at the angle with the lowest value, and our desired distance from the nearest object, 0.5. If both errors were 0, our neato would be exactly half a meter from the nearest object, and facing directly toward it.

We use a proportional controller to drive the anglular and linear velocity of the neato based on these two errors. This make the neato move efficiently and quickly toward an object when it is placed anywhere in its range

## Obstacle Avoidance

The portion of our obstacle avoidance code that actually directs the neato away from obstacles is the same as our person following script, except for that our error is based on the maximum value of the filtered scan instead of the minimum. This way, the neato is encouraged to move in the emptiest direction in its view. Objects repell it more as they get nearer, larger, and more closely grouped.

To set desired headings...

## Finite State Behavior
