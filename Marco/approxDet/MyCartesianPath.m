function path = MyCartesianPath(start,goal,cruising_altitude)
%MYCARTESIANPATH Finds a smooth path from start to goal with maximum 
% possible distance kept from the origin.
%   The minimum distance from the origin is defined as the radius of the
%   closest point between start and goal. The path will start on start, go
%   to the cruising altitude and follow a path that is formed by the tangent
%   from one point to the circumference with minimum radius and an arc of
%   said circumference. Then from cruising altitude it will reach the 
%   altitude of the goal. The path is passed through a filter to make it
%   smooth so if does not follow perfectly the minimum radius circumference

    arguments
        start (3,1)
        goal (3,1)
        cruising_altitude = max(start(3),goal(3))
    end

    % lengths of the paths from start and goal to cruising altitude
    l_path1 = abs(cruising_altitude-start(3));
    l_path3 = abs(cruising_altitude-goal(3));

    % path is parametrized with s from 0 to 1
    n_points = 70; % 30 are added at the end
    ds = 0:1/n_points:1;
    
    r_i = norm(start(1:2));
    r_g = norm(goal(1:2));
    
    % check which point is closer to the origin
    if r_i > r_g
        r_close = r_g;
        far_point = start;
        far_point(3) = cruising_altitude;
        close_point = goal;
        close_point(3) = cruising_altitude;
        invert_path = false;
    else
        r_close = r_i;
        far_point = goal;
        far_point(3) = cruising_altitude;
        close_point = start;
        close_point(3) = cruising_altitude;
        invert_path = true;
    end
    
    % finds the two tangent points
    syms tx ty
    eq1 = ty*far_point(2)+tx*far_point(1) == r_close^2;
    eq2 = tx^2+ty^2 == r_close^2;
    
    [tx,ty] = solve([eq1;eq2],[tx;ty]);
    
    % The two tangent points
    t1 = double([tx(1);ty(1)]);

    % if we already are on the circle there's only 1 tangent point
    try
        t2 = double([tx(2);ty(2)]);
    catch
        t2=t1;
    end

    % check if I need tangent points or not. If segment between start and
    % goal does not cross the minimum radius circle then I only need that
    % segment
    if norm(t1-far_point(1:2))<norm(far_point(1:2)-close_point(1:2))
        % check which tangent point is closer to close_point
        a1 = acos(dot(close_point(1:2),t1)/(norm(close_point(1:2))*norm(t1)));
        a2 = acos(dot(close_point(1:2),t2)/(norm(close_point(1:2))*norm(t2)));
        if a1 < a2
            a_min = a1;
            t_close = t1;
            t_close(3) = cruising_altitude;
        else
            a_min = a2;
            t_close = t2;
            t_close(3) = cruising_altitude;
        end
        l1 = norm(far_point(1:2)-t_close(1:2));
        l2 = a_min*r_close;
        l_tot = l_path1+l1+l2+l_path3;
        idx_start = int8(double(l_path1/l_tot*n_points))+1;
        idx_middle = int8(double((l_path1+l1)/l_tot*n_points))+1;
        idx_end = int8(double((l_path1+l1+l2)/l_tot*n_points))+1;
        ds_start = ds(idx_start);
        ds_middle = ds(idx_middle);
        ds_end = ds(idx_end);
        if ds_start == ds_middle
            first_segment = [];
        else
            first_segment = (1-(ds(idx_start:idx_middle)-ds_start)/(ds_middle-ds_start)).*far_point+(ds(idx_start:idx_middle)-ds_start)/(ds_middle-ds_start).*t_close;
        end
        theta1 = atan2(t_close(2),t_close(1));
        theta2 = atan2(close_point(2),close_point(1));
        
        if theta1<0
            theta1 = theta1 + 2*pi;
        end
        if theta2<0
            theta2 = theta2 + 2*pi;
        end
        if theta2-theta1>pi
            theta2 = theta2 - 2*pi;
        elseif theta1-theta2>pi
            theta2 = theta2 + 2*pi;
        end
        theta_t = (1-(ds(idx_middle:idx_end)-ds_middle)/(ds_end-ds_middle)).*theta1+(ds(idx_middle:idx_end)-ds_middle)/(ds_end-ds_middle).*theta2;
        if ds_end == ds_middle
            second_segment = [];
        else
            second_segment = [cos(theta_t)*r_close;sin(theta_t)*r_close;ones(1,length(theta_t))*cruising_altitude];
        end
        path2 = [first_segment second_segment];
    else
        l_path2 = norm(far_point-close_point);
        l_tot = l_path1+l_path2+l_path3;
        idx_start = int8(double(l_path1/l_tot*n_points))+1;
        idx_end = int8(double((l_path1+l_path2)/l_tot*n_points))+1;
        ds_start = ds(idx_start);
        ds_end = ds(idx_end);
        path2 = (1-(ds(idx_start:idx_end)-ds_start)/(ds_end-ds_start)).*far_point+(ds(idx_start:idx_end)-ds_start)/(ds_end-ds_start).*close_point;
    end
    
    if invert_path
        path2 = flip(path2,2);
    end
    
    if ds_start==0
        path1 = [];
    else
        path1 = (1-ds(1:idx_start)/ds_start).*start+ds(1:idx_start)/ds_start.*[start(1);start(2);cruising_altitude];
    end

    if ds_end==1
        path3 = [];
    else
        path3 = (1-(ds(idx_end:end)-ds_end)/(1-ds_end)).*[goal(1);goal(2);cruising_altitude]+(ds(idx_end:end)-ds_end)/(1-ds_end).*goal;
    end

    path_not_smooth = [path1, path2, path3];
    % smoothing the path.
    % before smoothing points are added at the beginning and end to make
    % the path start with zero velocity
    window = 14;
    path = smoothdata([ones(3,window).*path_not_smooth(:,1), path_not_smooth, ones(3,window).*path_not_smooth(:,end)],2,'gaussian',int8(1.5*window));
    path = path(:,1:100);
end

