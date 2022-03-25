if not(exist('started','var') && started)
    started = StartSimulation("192.168.1.104");
end

jointSub = rossubscriber('/my_gen3/joint_states');
[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');

gripperX = 0.3;
gripperY = 0.0;
gripperZ = 0.5;
gripperRotationX = 0; % radians
gripperRotationY = pi; % radians
gripperRotationZ = pi/2; % radians

done = moveGripperTo(robot, trajAct, trajGoal, jointSub, [gripperX,gripperY,gripperZ], [gripperRotationX,gripperRotationY,gripperRotationZ]);

% gripperTranslation = [gripperX gripperY gripperZ];
% desiredGripperPose = trvec2tform(gripperTranslation)*axang2tform([1 0 0 gripperRotationX])*axang2tform([0 1 0 gripperRotationY])*axang2tform([0 0 1 gripperRotationZ]);
% jointMsg = receive(jointSub,2);
% currentRobotJConfig =  jointMsg.Position(2:8);
% [q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory(currentRobotJConfig, desiredGripperPose, robot, 'gripper', 4);
% 
% trajGoal = RoboCupManipulation_packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
% 
% server_response = waitForServer(trajAct,5);
% if(server_response)
%     sendGoal(trajAct,trajGoal);
% else
%     print('no connection to server');
% end



