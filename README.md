# Trilateration ROS package.

Implementation of different trilateration localization algorithms:
- Linear trilateration
- Nonlinear trilateration
- RANSAC trilateration (TODO)

# Modified to be compatible with the GTEC solution:

- Ranging messages are now gtec_msgs::Ranging ([https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs))
- There is a new subscription to receive the anchor positions using a publisher like the node *anchors* included in:[https://github.com/valentinbarral/rostoa](https://github.com/valentinbarral/rostoa).
- The output is now a *PoseWithCovarianceStamped*.
- Tested and working in simulation using [https://github.com/valentinbarral/gazebosensorplugins](https://github.com/valentinbarral/gazebosensorplugins). 

