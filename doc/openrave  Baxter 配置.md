openrave  Baxter 配置

 

https://sdk.rethinkrobotics.com/wiki/Custom_IKFast_for_your_Baxter



https://github.com/rdiankov/openrave/issues/568



https://github.com/rdiankov/openrave/pull/408/files



https://blog.csdn.net/u011573853/article/details/105312499



rosrun moveit_kinematics round_collada_numbers.py  baxter_arm.right.dae baxter_arm.right.rounded.dae 6

 rosrun moveit_kinematics round_collada_numbers.py  baxter_arm.left.dae baxter_arm.left.rounded.dae 6





python /opt/ros/kinetic/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/ikfast.py  --robot=baxter_arm.right.rounded.dae --iktype=transform6d --baselink=1 --eelink=10 --freeindex=5 --savefile=baxter_right_arm_ikfast_solver.cpp



python /opt/ros/kinetic/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/ikfast.py  --robot=baxter_arm.left.rounded.dae --iktype=transform6d --baselink=1 --eelink=10 --freeindex=5 --savefile=baxter_left_arm_ikfast_solver.cpp



