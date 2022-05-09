function rpy_angles = MyOrientationPlanner(initial_angles,target_angles,linear_time)
%MYORIENTATIONPLANNER Linearly reaches desired orientation from starting
%one.
%   The parameter linear_time specifies the part of the total time of the
%   trajectory that should be used to reach the target_angles, then the
%   angles should be constant.
    arguments
        initial_angles (3,1)
        target_angles (3,1)
        linear_time = 0.2
    end

    total_steps = 100;
    
    ds=0:1/(total_steps*linear_time):1;
    rpy_angles = (1-ds).*initial_angles+ds.*target_angles;
    rpy_angles = [rpy_angles(:,1:end-1),ones(3,int8(total_steps*(1-linear_time))).*rpy_angles(:,end)];
end

