function [q,qd,qdd,trajTimes,path,velocities,orientations,dets] = MyComputeTrajectory(currentRobotJConfig, finalPosition, finalOrientation, robot, endEffector, trajDuration, detEstimator)
%MYCOMPUTETRAJECTORY This function returns q,qd,qdd,trajTimes ready to be passed to a
%trajectory packager and then to the ros controller
%   The parameters are:
%       - currentRobotJConfig: the current values for the q of the robot
%       - finalPosition: 3d desired position to be reached
%       - finalOrientation: 'XYZ' angles for desired e-e orientation
%       - robot: rigidBodyTree of the robot
%       - endEffector: name of the link to control
%       - trajDuration: desired duration of the trajectory
%       - detEstimator: neural network used to estimate determinant of the
%       Geometric Jacobian of the robot

    timestep = trajDuration/98; 

    trajTimes = 0:timestep:trajDuration;
    trajTimes = trajTimes(1:end);

    %Initial task config
    jointInit = wrapToPi(currentRobotJConfig');
    % Initial end-effector pose
    taskInit = getTransform(robot, jointInit, endEffector);

    % compute cartesian path
    % this can be changed to a straight line path, probably more intuitive
    % to use
    path = MyCartesianPath(tform2trvec(taskInit)',finalPosition);
    % compute desired angle path
    initialOrientation = rot2ang(taskInit(1:3,1:3),'xyz',[0,pi,pi/2]);
    orientations = MyOrientationPlanner(initialOrientation,finalOrientation);
    
    q = zeros(length(jointInit),0);
    qd = zeros(length(jointInit),0);
    velocities = zeros(3,0);
    dets = zeros(1,0);
    robotPos = jointInit;
    angleCurrent = initialOrientation;
    T = getTransform(robot, robotPos, endEffector);
    idx = 1;
    while norm(finalPosition-tform2trvec(T))>1e-3 && idx<=200
        i = min(idx,length(path));
        T = getTransform(robot, robotPos, endEffector);
        % feedback velocities as difference between desired position and
        % current one
        velocities(:,idx) = (path(:,i)-tform2trvec(T)')/timestep;

        % feedback angular velocity
        angleTarget = orientations(:,i);
        angleCurrent = rot2ang(T(1:3,1:3),'xyz',angleCurrent);
        % angleCurrent = angleCurrent + omega*timestep;
        omega = (angleTarget - angleCurrent)/timestep*0.8; % 0.8 used to reduce oscillations

        Ji = geometricJacobian(robot,robotPos,endEffector);
        dets(:,idx) = det(Ji(1:6,1:6));
        % task priority
        gradH = getGradientNet(detEstimator,robotPos');
        % Task priority method with two task
        qd(:,idx) = InvKinTp_optim({velocities(:,idx),omega},{Ji(4:6,:),Ji(1:3,:)},gradH);
        % Pseudoinverse method
        %qd(:,idx) = InvKinPinv_optim([omega;velocities(:,i)],Ji(1:6,:),gradH);
        
        % Task priority method with no gradient optimization 
        %qd(:,idx) = InvKinTp({velocities(:,idx),omega},{Ji(4:6,:),Ji(1:3,:)});
        % dampened least squares with no gradient optimization
        %qd(:,idx) = InvKinDls([omega;velocities(:,i)],Ji(1:6,:),0.002);
        
        % dampened least squares with no angle constrains
        % qd(:,idx) = InvKinDls(velocities(:,i),Ji(4:6,:),0.002);
        
        q(:,idx) = robotPos;
        robotPos = robotPos+qd(:,idx).'*timestep;
        idx = idx+1;
    end

    qdd = [zeros(length(jointInit),1),diff(qd,1,2)];
    
end