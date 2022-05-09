function desiredPose = grasp_and_throw(dir,point,label)
    % end effector orientation
    Ree = grasp_orientation(dir,0); % TODO set alpha
    % end effector position
    tee = point;

    desiredPose = [tee; rotm2eul(Ree,"xyz")'];

    % goto ...
end

