% This script runs the MyComputeTrajectory function. It takes the initial
% robot coonfiguration and a desired position and orientation and returns
% q,qd,qdd for the robot to follow.
% (For now it returns other stuff used for debugging)

clc;clear;close all;
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
load('my_net.mat');

% initial robot configuration
% currentRobotJConfig = randomConfiguration(robot).';
% currentRobotJConfig = [-0.1625;1.1614;0.5558;-2.0513;-1.5327;-0.5755;-1.4158];

%initial config
currentRobotJConfig = [-1.9494;-0.0346;-1.1962;-1.0550;0.0367;-2.0500;1.5847];

syms q1 q2 q3 q4 q5 q6 q7

% goal end effector position and orientation
gripperPosition = [0.5,-0.5,0.3];
gripperRotation = [0;pi;pi/2]; % radians

[q,qd,qdd,trajTimes,path,v,orient,dets] = MyComputeTrajectory(currentRobotJConfig, gripperPosition, gripperRotation, robot, 'gripper', 10, net);

% from q calculates the positions of the end effector 
p = zeros(3,length(q));
v_real = zeros(3,length(q));
angles = zeros(3,length(q));
guess = [0;pi;pi/2];
for i = 1:length(q)
    T = getTransform(robot, q(:,i).', 'gripper');
    p(:,i) = T(1:3,4);
    if i>1
        v_real(:,i) = p(:,i)-p(:,i-1);
    end
    angles(:,i) = rot2ang(T,'xyz',guess);
    guess = angles(:,i);
end


T = getTransform(robot, currentRobotJConfig.', 'gripper');
start = T(1:3,4);
goal = gripperPosition;
hold on;
plot3(start(1),start(2),start(3),'x');
plot3(goal(1),goal(2),goal(3),'o');
plot3(p(1,:),p(2,:),p(3,:),'-x');
plot3(path(1,:),path(2,:),path(3,:),'--','color',[0,0,0]);
title('Plot of end effector positions')
legend('start','goal','actual path','desired path')

figure;
hold on;
plot(dets);
vnorm = vecnorm(v);
plot(vnorm);
vnorm2 = vecnorm(v_real);
plot(vnorm2);
legend('determinant','desired velocity norm','real velocity norm');
title('Plot determinant')

figure;
hold on;
plot(angles(1,:),'Color',[1,0,0]);
plot(orient(1,:),'--','Color',[1,0,0]);
plot(angles(2,:),'Color',[0,1,0]);
plot(orient(2,:),'--','Color',[0,1,0]);
plot(angles(3,:),'Color',[0,0,1]);
plot(orient(3,:),'--','Color',[0,0,1]);
legend('x','','y','','z','')
title('Plot of end effector angles')

legend('x','y','z')
title('Plot of desired end effector angles')

figure;
hold on
plotted_q = qd;
plot(plotted_q(1,:))
plot(plotted_q(2,:))
plot(plotted_q(3,:))
plot(plotted_q(4,:))
plot(plotted_q(5,:))
plot(plotted_q(6,:))
plot(plotted_q(7,:))
title('Plot of q velocities')
