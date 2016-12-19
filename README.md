# moveit_python_tools
Python tools to ease usage of MoveIt! with any robot.

It contains classes to use easier/faster MoveIt! modules
and command line tools to use them too.

# Command line tools

* `tf_for_me.py`: Give it a *[pose/point/quaternion]* and a *from_frame* and a *to_frame* and it will transform it for you.
* `pr2_get_ik.py`: Give it a *pose* and it will get the IK for you.
* `pr2_get_ik_and_go.py`: Give it a *pose* and it will get an IK and move the arm there.
* `go_to_joint_cfg.py`: Give it a list of *joint_names* and a list of *positions* and it will move
the arm there.

# Classes

* TF:
```python
from moveit_python_tools.transformer import Transformer

tfer = Transformer()
# You can transform from any of:
# PoseStamped, Pose, PointStamped, Point,
# QuaternionStamped, Quaternion,
# Vector3 (with type_data: xyz, rpydeg, rpyrad),
# list or tuple (with type_data: xyz, rpydeg, rpyrad, quat)
print tfer.transform(thing)
```

* IK:
```python
from moveit_python_tools.get_ik import GetIK
from geometry_msgs.msg import PoseStamped

gik = GetIK("left_arm")
ps = PoseStamped()
ps.header.frame_id = 'l_wrist__roll_link'
ps.pose.position.x = 0.1
ps.pose.orientation.w = 1.0
print gik.get_ik(ps)
```

* Configuration execution:
```python
from moveit_python_tools.go_to_configuration import GoToConfiguration

gtc = GoToConfiguration()
joint_names = ['head_pan_joint', 'head_tilt_joint']
positions = [0.0, 0.5]
print gtc.go_to_configuration(joint_names, positions)
```
