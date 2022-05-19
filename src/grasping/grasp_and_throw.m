function desired_pose = grasp_and_throw(dir,point,label,robot)
    global trajAct trajGoal jointSub
    % end effector orientation
    Ree = grasp_orientation(dir,0); % TODO set alpha
    % end effector position
    tee = point;

    desired_pose = [tee; rotm2eul(Ree,"xyz")'];
    
    goto = @(pose) (moveGripperTo(robot,trajAct,trajGoal,jointSub,pose(1:3)',pose(4:6)'));

    approach_pose = desired_pose + [Ree*[0;0;-0.2]; [0;0;0]];
    
    goto(approach_pose);
    goto(desired_pose);
    SLActivateGripper(1);
    goto(approach_pose);
    if label == "can"
        goto(can_bin);
    else
        goto(bottle_bin)
    end
    SLActivateGripper(0);

end

