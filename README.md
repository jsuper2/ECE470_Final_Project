# ECE470_Final_Project
This is the central repository for Joshua Super's ECE470 final project. 

To push to git
git add *
git commit -m "{commit message}"
git push origin master

The Test Code directory hold example files provided by CoppeliaSim and ECE470 for the simulation software.

vrep_example directory holds the example from the ECE470 lab manual and is mainly for the UR3 robotic arm. All important parts of this example have been ported to update_2

update_2_OLD is the directory with scripts to control the hexapod as well as FK and IK scripts. However these kinematic scripts were written before learning how to use screw motion and exponentials to calculate transformations

update_2 is the directory with scripts to control the hexapod as well as FK and IK scripts. These kinematic scripts were written after learning how to use screw motion and exponentials to calculate transformations and therefore utalize those techniques for the forward kinematics but not inverse

update_3 is the most up to date directory containing all the files used in the most recent project update. The step_3.py file now generates interpolated step paths and can handle steps in multiple directions. The hextest file shows the robot walking in a circle by rotating the step direction each step until the robot has made a full circle.
