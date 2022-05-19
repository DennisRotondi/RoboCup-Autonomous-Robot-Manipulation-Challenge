function theta = rot2ang(R,str,initial_guesses)
%

if not(str=='xyz')
    disp("Error, only xyz order implemented");
    return;
end
theta = zeros([3,1]);
if R(1,3) < 1
    if R(1,3) > -1
        t1 = asin(R(1,3));
        t3 = 1e10;
        if t1 > 0
            t2 = pi-t1;
        elseif t1 < 0
            t2 = -pi-t1;
        else
            t2 = -pi;
            t3 = pi;
        end
        diff1 = abs(initial_guesses(2)-t1);
        diff2 = abs(initial_guesses(2)-t2);
        diff3 = abs(initial_guesses(2)-t3);
        if diff1 <= diff2 && diff1 <= diff3
            theta(2) = t1;
        elseif diff2 <= diff3
            theta(2) = t2;
        else
            theta(2) = t3;
        end

        
        t1 = atan2(-R(2,3),R(3,3));
        t2 = t1-pi;
        t3 = t1+pi;
        diff1 = abs(initial_guesses(1)-t1);
        diff2 = abs(initial_guesses(1)-t2);
        diff3 = abs(initial_guesses(1)-t3);
        if diff1 <= diff2 && diff1 <= diff3
            theta(1) = t1;
        elseif diff2 <= diff3
            theta(1) = t2;
        else
            theta(1) = t3;
        end

        t1 = atan2(-R(1,2),R(1,1));
        t2 = t1-pi;
        t3 = t1+pi;
        diff1 = abs(initial_guesses(3)-t1);
        diff2 = abs(initial_guesses(3)-t2);
        diff3 = abs(initial_guesses(3)-t3);
        if diff1 <= diff2 && diff1 <= diff3
            theta(3) = t1;
        elseif diff2 <= diff3
            theta(3) = t2;
        else
            theta(3) = t3;
        end

    else
        theta(2) = -pi/2;
        % the diff between z and x needs to be atan2(R(1,0),R(1,1))
        error_zx = atan2(R(2,1),R(2,2))-(initial_guesses(3)-initial_guesses(1));
        theta(3) = initial_guesses(3)+error_zx/2;
        theta(1) = initial_guesses(1)-error_zx/2;
    end
else
    theta(2) = pi/2;
    % the sum between z and x needs to be atan2(R(1,0),R(1,1))
    error_zx = atan2(R(2,1),R(2,2))-(initial_guesses(3)+initial_guesses(1));
    theta(3) = initial_guesses(3)-error_zx/2;
    theta(1) = initial_guesses(1)-error_zx/2;
end