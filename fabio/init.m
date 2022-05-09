clear all;
addpath("../Marco") 
ros_ip = "192.168.167.129";
rosshutdown; % shut down existing connection to ROS
rosinit(ros_ip,11311);
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
global jointSub trajAct trajGoal gripAct gripGoal
jointSub = rossubscriber('/my_gen3/joint_states');
[gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');