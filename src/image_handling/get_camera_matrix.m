function [K,P] = get_camera_matrix
    sub = rossubscriber("/camera/depth/camera_info");
    msg = receive(sub,1);
    K = reshape(msg.K,3,3);
    K=K';
    P = reshape(msg.P,3,4);
end

