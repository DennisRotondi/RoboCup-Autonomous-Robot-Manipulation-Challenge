function [q,qd,qdd,trajTimes,path,velocities,orientations,dets] = MyComputeTrajectory(currentRobotJConfig, finalPosition, finalOrientation, robot, endEffector, trajDuration, detEstimator)
%MYCOMPUTETRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    timestep = trajDuration/98; %100 are the points returned by MyCartesianPath
%     ik = inverseKinematics('RigidBodyTree',robot);
%     ik.SolverParameters.AllowRandomRestart = false;

    trajTimes = 0:timestep:trajDuration;
    trajTimes = trajTimes(1:end);

    %Initial task config
    jointInit = wrapToPi(currentRobotJConfig');
    % Initial end-effector pose
    taskInit = getTransform(robot, jointInit, endEffector);

    % compute cartesian path
    path = MyCartesianPath(tform2trvec(taskInit)',finalPosition);
    % compute desired angle path
    initialOrientation = rot2ang(taskInit(1:3,1:3),'xyz',[0,pi,pi/2]); %<----------- initial guess needs fixing
    orientations = MyOrientationPlanner(initialOrientation,finalOrientation);
    
    q = zeros(length(jointInit),0);
    qd = zeros(length(jointInit),0);
    velocities = zeros(3,0);
    dets = zeros(1,0);
    robotPos = jointInit;
    angleCurrent = initialOrientation;
    omega = [0;0;0];
    T = getTransform(robot, robotPos, endEffector);
    idx = 1;
    while norm(finalPosition-tform2trvec(T))>1e-3 && idx<=200
        i = min(idx,length(path));
        T = getTransform(robot, robotPos, endEffector);
        % feedback velocities as difference between desired position and
        % current one
        velocities(:,idx) = (path(:,i)-tform2trvec(T)')/timestep;

        % feedback omega
        angleTarget = orientations(:,i);
        angleCurrent = rot2ang(T(1:3,1:3),'xyz',angleCurrent);
        % angleCurrent = angleCurrent + omega*timestep;
        omega = (angleTarget - angleCurrent)/timestep*0.8; %<------------------change it to reduce oscillations
        %omega = [0; 0; 0];

        Ji = geometricJacobian(robot,robotPos,endEffector);
        dets(:,idx) = det(Ji(1:6,1:6));
        % task priority
        gradH = getGradientNet(detEstimator,robotPos');
        %qd(:,idx) = InvKinTp_optim({velocities(:,idx),omega},{Ji(4:6,:),Ji(1:3,:)},gradH);
        qd(:,idx) = InvKinPinv_optim([omega;velocities(:,i)],Ji(1:6,:),gradH);
%         qd(:,idx) = InvKinTp({velocities(:,idx),omega},{Ji(4:6,:),Ji(1:3,:)});
        % dampened least squares
        %qd(:,idx) = InvKinDls([omega;velocities(:,i)],Ji(1:6,:),0.002);
        
        % dampened least squares with no angle constrains
        % qd(:,idx) = InvKinDls(velocities(:,i),Ji(4:6,:),0.002);
        
        q(:,idx) = robotPos;
        robotPos = robotPos+qd(:,idx).'*timestep;
        idx = idx+1;
    end

    qdd = [zeros(length(jointInit),1),diff(qd,1,2)];
    
end



% prova con jacobiana analitica per massimizzare distanza dalle singolarità

% puoi provare con metodo repulsivo dalle singolatrità