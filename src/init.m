load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
global jointSub trajAct trajGoal gripAct gripGoal
jointSub = rossubscriber('/my_gen3/joint_states');
[gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');