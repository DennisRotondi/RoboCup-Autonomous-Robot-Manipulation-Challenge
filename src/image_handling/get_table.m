%% Step 1: Move to scanning positions and capture point cloud data and camera poses
tftree = rostf; %finds TransformationTree directly from ros
% from z=0.5 camera sees +-0.25 in x direction, +-0.4 in y direction
n_rows = 7; % 7 in theory
n_cols = 4;
scan_positions = {};
for row = 1:n_rows
    for col = 1:n_cols
        scan_positions{end+1} = [0.25*row,1-0.4*col,0.5];
    end
end
scan_orientation = [0,pi,pi/2];
camera_quaternions = cell(numel(scan_positions),1);
camera_translations = cell(numel(scan_positions),1); 
pointclouds = cell(numel(scan_positions),1);
for scanIter = 1: numel(scan_positions)
    % Move to next scanning pose
    disp(['Moving to scanning pose ' num2str(scanIter)]);
    done = moveGripperTo(robot, trajAct, trajGoal, jointSub, scan_positions{scanIter}, scan_orientation);
%     pause(10);
    % Capture point cloud
    disp(['Capturing point cloud ' num2str(scanIter)]);
    pointCloudSub = rossubscriber('/camera/depth/points');
    pointclouds{scanIter} = receive(pointCloudSub);
    % Get camera pose
    disp(['Getting camera pose ' num2str(scanIter)]);
    camera_transf = getTransform(tftree, 'world', 'camera_link');        
    camera_transl = camera_transf.Transform.Translation;
    camera_rotation = camera_transf.Transform.Rotation;
    camera_quaternions{scanIter} = [camera_rotation.W, camera_rotation.X,...
        camera_rotation.Y,camera_rotation.Z];
    camera_translations{scanIter} = [camera_transl.X,...
        camera_transl.Y,camera_transl.Z];
end

%% Steps 2 and 3: Transform point clouds to world frame and merge to single point cloud
pointclouds = pointclouds';
for i=1:size(pointclouds,2)
    % Extract xyz locations of point cloud
    xyz = readXYZ(pointclouds{1,i});
    % Remove NaNs
    xyzLess = double(rmmissing(xyz));
    % Create point cloud object
    ptCloud = pointCloud(xyzLess);
    % Create affinite transformation from camera quaternions and
    % translations
    quat = camera_quaternions{i};
    rotm = quat2rotm(quat);
    fixedRotation = eul2rotm([0 pi 0],"XYZ"); % fixed rotation between gazebo camera and urdf camera link
    rotm = rotm*fixedRotation' ;
    translVect = camera_translations{i};
    tform = rigid3d(rotm,[translVect(1),translVect(2),translVect(3)]);
    % Transform point cloud to world frame
    ptCloudWorld = pctransform(ptCloud,tform);
%     figure(i);
%     pcshow(ptCloudWorld);
    % Merge with other point clouds
    if i == 1
        pcMerged = ptCloudWorld;
    else
        mergeSize = 0.01;
        pcMerged = pcmerge(pcMerged, ptCloudWorld, mergeSize);
    end
end
disp("fine")
% pcshow(pcMerged);
% pause(3);
% Remove points below table height
indxPlane = find(pcMerged.Location(:,3) > -0.09 & pcMerged.Location(:,3) < -0.07);
plane = select(pcMerged,indxPlane); % table   
indx = find(pcMerged.Location(:,3) > -0.08);
pcMergedTop = select(pcMerged,indx); %FINAL RESULT
% pcshow(pcMergedTop);
% pause(3);


