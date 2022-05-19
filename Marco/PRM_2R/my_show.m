function plotted = my_show(robot,config)
%MY_SHOW Summary of this function goes here
%   Detailed explanation goes here
base = [0,0];
T1 = getTransform(robot,config,'body1','body2');
p1 = T1(1:2,4);
T2 = getTransform(robot,config,'body2','endeffector');
p2 = p1+T2(1:2,4);
x = [base(1),p1(1),p2(1)];
y = [base(2),p1(2),p2(2)];
plot(x,y,'x-');
end

