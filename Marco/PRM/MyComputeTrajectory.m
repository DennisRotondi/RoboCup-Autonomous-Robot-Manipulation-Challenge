function [q,qd,qdd,trajTimes] = MyComputeTrajectory(currentRobotJConfig, finalPosition, finalOrientation, robot, endEffector, trajDuration, mapRobot)
%MYCOMPUTETRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    timestep = trajDuration/98; %100 are the points returned by MyCartesianPath
    ik = inverseKinematics('RigidBodyTree',robot);
    ik.SolverParameters.AllowRandomRestart = false;
    weights = [1 1 1 1 1 1];

    trajTimes = 0:timestep:trajDuration;
    trajTimes = trajTimes(1:end);
    timeInterval = [0;trajDuration];

    %Initial task config
    jointInit = wrapToPi(currentRobotJConfig');
    %Final task config
    taskFinal = trvec2tform(finalPosition)*axang2tform([1 0 0 finalOrientation(1)])*axang2tform([0 1 0 finalOrientation(2)])*axang2tform([0 0 1 finalOrientation(3)]);
    jointFinal = ik(endEffector,taskFinal,weights,jointInit);
    % compute cartesian path
    q = myGetTrajectory(mapRobot,jointInit,jointFinal)';
    qd = [zeros(length(jointInit),1),diff(q,1,2)];
    qdd = [zeros(length(jointInit),1),diff(qd,1,2)];
    
end



% prova con jacobiana analitica per massimizzare distanza dalle singolarità

% puoi provare con metodo repulsivo dalle singolatrità