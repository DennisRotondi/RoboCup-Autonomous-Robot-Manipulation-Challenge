clc;clear;close all;
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');

mapKinova = getPRMStar(robot,5000);
plot(mapKinova);

save('mapKinova.mat','mapKinova');