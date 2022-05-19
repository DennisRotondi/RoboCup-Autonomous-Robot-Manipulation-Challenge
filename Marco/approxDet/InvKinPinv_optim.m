function q_d = InvKinPinv_optim(r_d,J, gradH)
%INV_KIN_PINV_OPTIM
    size_J = size(J);
    n_q = size_J(2);
    q_d = pinv(J)*r_d+(eye(n_q)-pinv(J)*J)*gradH*10;
end



