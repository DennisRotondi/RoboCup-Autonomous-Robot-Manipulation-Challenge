function q_d = InvKinDls(r_d,J, mu2)
%INV_KIN_DLS Returns dampened least squares solution to inverse kinematics problem
%   Finds q_d from r_d = J * q_d even if J is not invertible using
%   dampened least square method. mu2 is the squared damping parameter. This solution
%   always has an associated task error, but task velocities don't explode
%   near singularities.
    J_size = size(J);
    q_d = J.'*inv(mu2*eye(J_size(1))+J*J.')*r_d;
end

