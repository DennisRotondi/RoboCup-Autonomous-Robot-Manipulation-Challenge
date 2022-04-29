function points = p3D_from_bboxes(bboxes,depth_img,frame_id)
    numb=size(bboxes,1);
    points=zeros(3,numb);
    [K,~] = get_camera_matrix;
    fx=K(1,1);
    fy=K(2,2);
    ox=K(1,3);
    oy=K(2,3);
    for i=1:numb
        bbox=bboxes(i,:);
        centroid = round([bbox(1) + bbox(3) * 0.5, bbox(2) + bbox(4) * 0.5]);
        depth=depth_img(centroid(2),centroid(1));
        
        fixedRotation = eul2rotm([0 pi 0],"XYZ"); % fixed rotation between gazebo camera and urdf camera link
        
        tmp_p=[(centroid(1)-ox)*depth/fx;
               (centroid(2)-oy)*depth/fy;
               depth;];
        
        tmp_p= fixedRotation*tmp_p;

        pt = rosmessage('geometry_msgs/PointStamped');
        pt.Header.FrameId = 'camera_link';
        pt.Point.X = tmp_p(1);
        pt.Point.Y = tmp_p(2);
        pt.Point.Z = tmp_p(3);
        
        tr = getTransform(rostf,frame_id,'camera_link');
        attemptn=0;
        while(size(tr) == [0,1])
            pause(1)
            tr = getTransform(rostf,frame_id,'camera_link');
            attemptn=attemptn+1;
            if attemptn > 5
                points = "fail";
                return
            end
        end
        tfpt = apply(tr,pt); %Transform the ROS message to the 'frame_id' frame 

        points(:,i)=[tfpt.Point.X;
                     tfpt.Point.Y;
                     tfpt.Point.Z];
    end
end

