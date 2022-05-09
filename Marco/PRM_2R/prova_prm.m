clc;clear;close all;

robot = rigidBodyTree("DataFormat",'row');

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform2 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

bodyEndEffector = rigidBody('endeffector');
tform3 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform3);
addBody(robot,bodyEndEffector,'body2');

G = getPRMStar(robot,200);
plot(G)