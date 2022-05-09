function p3D = p3D_from_p2D(point,depth_img,frame_id)

    [K,~] = get_camera_matrix;
    fx=K(1,1);
    fy=K(2,2);
    ox=K(1,3);
    oy=K(2,3);

    depth=depth_img(point(2),point(1));
        
    fixedRotation = eul2rotm([0 pi 0],"XYZ"); % fixed rotation between gazebo camera and urdf camera link
        
    tmp_p=[(point(1)-ox)*depth/fx;
           (point(2)-oy)*depth/fy;
            depth;];
        
    tmp_p= fixedRotation*tmp_p;

    pt = rosmessage('geometry_msgs/PointStamped');
    pt.Header.FrameId = 'camera_link';
    pt.Point.X = tmp_p(1);
    pt.Point.Y = tmp_p(2);
    pt.Point.Z = tmp_p(3);
        
    tftree = rostf;
    tr = getTransform(tftree,frame_id,'camera_link');
    attemptn=0;
    while(size(tr) == [0,1])
        pause(1)
        tr = getTransform(tftree,frame_id,'camera_link');
        attemptn=attemptn+1;
        if attemptn > 5
            p3D = "fail";
            return
        end
    end
    tfpt = apply(tr,pt); %Transform the ROS message to the 'frame_id' frame 
    p3D=[tfpt.Point.X;
         tfpt.Point.Y;
         tfpt.Point.Z];
end

