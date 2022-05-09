function Ree = grasp_orientation(v,alpha) % v is my y-axis

    % find a non collinear vector to v
    p = [1;0;0];
    if abs(v - p ) < 10^-6 % if v == [1;0;0]
        p = [0;1;0];
    end
    
    l = cross(v,p); % orthogonal vector to v: my x-axis
    l = l/norm(l);
    m = cross(l,v); % z-axis
    
    
    C = [l v m];
    
    % choose alpha0 such that if alpha == 0 approach direction is vertical
    syms a
    alpha0_ = eval(vpa(solve(l(3)*cos(a)-m(3)*sin(a) == 0,a)));
    if size(alpha0_,1)>1
        alpha0_
        if l(3)*sin(alpha0_(1)) + m(3)*sin(alpha0_(1)) < 0
            alpha0 = alpha0_(1);
        else 
            alpha0 = alpha0_(2);
        end
    else
        alpha0 = 0;
    end
    Ry = [ cos(alpha+alpha0),  0, sin(alpha+alpha0);
            0,     1,  0;
         -sin(alpha+alpha0),  0, cos(alpha+alpha0)];
    
    Ree = C*Ry;

end

