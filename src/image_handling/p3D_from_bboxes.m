function points = p3D_from_bboxes(bboxes,depth_img,frame_id)
    numb=size(bboxes,1);
    points=zeros(3,numb);
    [K,~] = get_camera_matrix
    fx=K(1,1);
    fy=K(2,2);
    ox=K(1,3);
    oy=K(2,3);
    for i=1:numb
        bbox=bboxes(i,:);
        centroid = round([bbox(1) + bbox(3) * 0.5, bbox(2) + bbox(4) * 0.5]);
        depth=depth_img(centroid(2),centroid(1));
    
        pt = rosmessage('geometry_msgs/PointStamped');
        pt.Header.FrameId = 'camera_link';
        pt.Point.X = (centroid(1)-ox)*depth/fx;
        pt.Point.Y = (centroid(2)-oy)*depth/fy;
        pt.Point.Z = depth;
        tfpt = transform(rostf,frame_id,pt); %Transform the ROS message to the 'frame_id' frame 

        points(:,i)=[tfpt.Point.X;
                     tfpt.Point.Y;
                     tfpt.Point.Z];
    end
end

