function q_d = InvKinTp_optim(r_ds,Js,gradH)
%INV_KIN_TP Returns solution to inverse kinematics problem using task
%priorities
%   Finds q_d from r_d = J * q_d for each task separately, starting from
%   the first and solving the others after. This approach tries to solve
%   the first tasks without erorrs and "sends" the error to the later
%   tasks.
%   Takes as input a cell array with a list of tasks and a cell array with
%   the respective jacobians. The arrays should be ordered base on
%   priority.

    size_Js = size(Js);
    n_tasks = size_Js(2);
    size_J1 = size(Js{1});
    n_q = size_J1(2);
    q_d = zeros([n_q,1]);
    Pai = eye(n_q);
    for i=1:n_tasks
        Ji_Pai1 = Js{i}*Pai;
        pinv_Ji_Pai1 = pinv(Ji_Pai1);
        Pai = Pai-pinv_Ji_Pai1*Ji_Pai1;
        
        delta_q_d = pinv_Ji_Pai1*(r_ds{i}-Js{i}*q_d);
        % for last task I project gradH in the null space of the last task
        if i==n_tasks
            delta_q_d = delta_q_d + (eye(n_q)-pinv_Ji_Pai1*Ji_Pai1)*(gradH);
        end
        q_d = q_d + delta_q_d;
    end
    
end