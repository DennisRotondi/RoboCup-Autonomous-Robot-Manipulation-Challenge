function [resultMsg,resultState] = moveGripperTo(robot, trajectoryAction, trajectoryGoal, jointSubscriber, gripperPosition, gripperRotation)
%MOVEGRIPPERTO Moves gripper to specified location
%   Arguments are:
%       robot: rigidBodyTree of the model
%       trajectoryAction: SimpleActionClient
%       trajectoryGoal: The empty goal return when building action client
%       jointSubscriber: subscriber to robot joint_states
%       gripperPosition: desired [x y z] for robot end-effector
%       gripperPosition: desired RPY angles for robot end-effector
%   Uses RoboCupManipulation_computeTrajectory() to compute a trajectory
%   and executes it using the SimpleActionClient.
%   Return true when the goal is reached.

    desiredGripperPose = trvec2tform(gripperPosition)*axang2tform([1 0 0 gripperRotation(1)])*axang2tform([0 1 0 gripperRotation(2)])*axang2tform([0 0 1 gripperRotation(3)]);
    jointMsg = receive(jointSubscriber,2);
    currentRobotJConfig =  jointMsg.Position(2:8);
    [q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory(currentRobotJConfig, desiredGripperPose, robot, 'gripper', 4);
    trajectoryGoal = RoboCupManipulation_packageJointTrajectory(trajectoryGoal,q,qd,qdd,trajTimes);
    
    server_response = waitForServer(trajectoryAction,5);
    if(server_response)
        [resultMsg,resultState] = sendGoalAndWait(trajectoryAction,trajectoryGoal);
        done = true;
    else
        print('no connection to server');
        resultMsg = 0;
        resultState = 0;
    end
    
    end