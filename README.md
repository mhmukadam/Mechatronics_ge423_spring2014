# [Mechatronics Project](http://coecsl.ece.illinois.edu/ge423/spring14/Group9_MustafaNathanBrad/Robobuddies_website/GE423_Webpage_Robobuddies.htm) (GE423) at UIUC, Spring 2014 #

The Mobile Robot platform runs on a OMAPL138 TI DSP (with ARM9) and a F28335 TI controller board. This robot uses an onboard LADAR to map the area in 2D instantaneously and travels between checkpoints in the maze by using real-time A* path planning and re-planning.

Kalman filtering is applied to achieve sensor fusion between dead reckoned odometry measurements and measurements obtained from optitrack - a motion capture system, to get accurate positioning of the robot. Computer Vision is used with an onboard camera and a servo mechanism to detect, collect, sort and deposit colored golf balls from within the maze to the drop off areas.
