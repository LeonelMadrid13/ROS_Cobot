# ROS/Python

## Links

1. <https://stackoverflow.com/questions/26656943/how-to-get-data-in-python-from-ros-in-real-time>
2. <https://stackoverflow.com/questions/71514363/ros-python-how-to-access-data-from-a-rostopic-in-python>
3. <https://www.geeksforgeeks.org/ros-publishers-using-python>
4. <http://wiki.ros.org/pid>
5. <http://wiki.ros.org/rosserial_arduino/Tutorials>

## How to get data in python from ROS in real-time?

Take a closer look at the tutorials. Especially
<http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>

```BASH
cd into youd source folder
```

and create new package:

```BASH
catkin_create_pkg playground
```

cd in that directory

```BASH
cd playground
```

create a file listener.py and and add following content:

```Python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

make executable:

```Bash
chmod +x listener.py
```

start core:

```Bash
roscore
```

new terminal, run listener node:

```Bash
rosrun playground listener.py
```

new terminal publis some odometry data:

```Bash
rostopic pub -r1 /odom nav_msgs/Odometry {}
```

Now you should see some output on your terminal where your listener was started

## ROS / Python: How to access data from a rostopic in python?

### Question

I'm trying to write a python code, that controls a drone. I receive the position from a Rigid-body trough a rostopic, and I want to use that data in my code. How can i access it in my python code?

### Answer

Yes, you can access the ROS topic data in your Python code. Take the following example:

```Python
#!/usr/bin/env python

import numpy as np
import rospy
from pycrazyswarm import *
from geometry_msgs.msg import Pose


Z = 1.0

def some_callback(msg):
    rospy.loginfo('Got the message: ' + str(msg))

if __name__ == "__main__":
    swarm = Crazyswarm()
 
    # get initial position for maneuver
    swarm.allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    swarm.timeHelper.sleep(1.5+Z)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()
    
    #start ros node
    rospy.init_node('my_node')
    #Create a subscriber
    rospy.Subscriber('/vrpn_client_ros/RigidBody/pose', Pose, some_callback)

    # finished following the rigid body. Get back landing
    swarm.allcfs.land(targetHeight=0.02, duration=1.0+Z)
    swarm.timeHelper.sleep(1.0+Z)
```

This will create a ROS node, listen to the data coming from your topic, and print it out much like rostopic echo would.
