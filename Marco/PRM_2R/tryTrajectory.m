clc;clear;close all;

load('map2R.mat');

startP = [1,2];
endP = [-2,-1];
traj = myGetTrajectory(myMap,startP,endP);

my_plot_graph(myMap);
plot(traj(:,1),traj(:,2),'Color',[1,0,0],'LineWidth',3);

figure;
hold on;
plot(traj(:,1));
plot(traj(:,2));
hold off;