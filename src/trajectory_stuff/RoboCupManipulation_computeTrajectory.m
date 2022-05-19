function [q,qd,qdd,trajTimes] = MWRoboCupChallenge_computeTrajectory(currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)
% Copyright 2021 MathWorks, Inc
%
% This function use features available in the Robotics System Toolbox to
% compute a smooth trajectory between the desired end effector position and
% the current robot configuration
%
% For more information on the features implemented here visit:
%   https://www.mathworks.com/help/robotics/referencelist.html?type=function&category=manipulators


        timestep = 0.05;
        ik = inverseKinematics('RigidBodyTree',robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        %Initial task config
        jointInit = wrapToPi(currentRobotJConfig');
        % Initial end-effector pose
        taskInit = getTransform(robot, jointInit, endEffector);

        % Time intervals
        timeInterval = [0;trajDuration];
        trajTimes = timeInterval(1):timestep:timeInterval(end)-timestep*20;

        % Retrieve task configurations between initial and final
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes));
        [T, ~, ~] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes, 'TimeScaling',[s;sd;sdd]/timeInterval(end));

        % Compute corresponding joint configurations
        robotPos = zeros(size(T,3),numel(jointInit));
        initialGuess = wrapToPi(jointInit); 
        for i=1:size(T,3)            
            robotPos(i,:) = ik(endEffector,T(:,:,i),weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %%  Compute joint velocities and accelerations at required rate for execution by the robot
        % disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = timestep;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,numel(jointInit));robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,numel(jointInit));robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    
        
        [q,qd,qdd,trajTimes] = fixHardBounds(q,qd,qdd,trajTimes);
end

function [qNew,qdNew,qddNew,trajTimesNew] = fixHardBounds(q,qd,qdd,trajTimes)
%FIXHARDBOUNDS fixing velocities to respect maximum velocities values, this
%will change total trajectory time
qd_max = 3;

timestep = trajTimes(2)-trajTimes(1);

%fixing velocities following hard bound qd_max
qdNew = zeros([size(qd(:,1))]);
idxNew=1;
for idx=1:length(qd)
    qdNorm = abs(qd(:,idx)/qd_max);
    biggest = max(qdNorm);
    if biggest>1
        additionalSteps = ceil(biggest);
        for i=0:additionalSteps-1
            qdNew(:,idxNew)=qd(:,idx)/additionalSteps;
            idxNew = idxNew+1;
        end
    else
        qdNew(:,idxNew)=qd(:,idx);
        idxNew = idxNew+1;
    end
end

%getting new q trajectory
trajTimesNew = 0:timestep:timestep*(length(qdNew)-1);
qNew = q(:,1);

for idx=2:length(qdNew)
    qNew(:,idx) = qNew(:,idx-1)+qdNew(:,idx)*(trajTimesNew(idx)-trajTimesNew(idx-1));
end
%applying gaussian filter to q trajectory to smooth out discontinuities
window = 10;
qNew = smoothdata([ones(length(q(:,1)),window).*qNew(:,1), qNew, ones(length(q(:,1)),window).*qNew(:,end)],2,'gaussian',int8(1.5*window));

trajTimesNew = 0:timestep:timestep*(length(qNew)-1);

%getting new velocities and accellerations
qdTemp = diff(qNew');
qdNew = [zeros(1,length(q(:,1)));qdTemp]'./[timestep,diff(trajTimesNew)];

qddTemp = diff(qdNew');
qddNew = [zeros(1,length(q(:,1)));qddTemp]'./[timestep,diff(trajTimesNew)];

end