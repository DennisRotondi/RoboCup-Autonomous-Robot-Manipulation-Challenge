function [P] = get_camera_matrix
    sub = rossubscriber("/camera/color/camera_info");
    msg = receive(sub,1);
    P = reshape(msg.P,3,4);
    
end

