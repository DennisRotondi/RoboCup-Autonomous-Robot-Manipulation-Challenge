%% Steps 2 and 3: Transform point clouds to world frame and merge to single point cloud
% pointclouds = pointclouds';
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
    indx = find(ptCloudWorld.Location(:,3) > -0.2);
    ptCloudWorld = select(ptCloudWorld,indx);
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
% pcshow(pcMerged);
pause(3);

% Remove points below table height
indxPlane = find(pcMerged.Location(:,3) > -0.09 & pcMerged.Location(:,3) < -0.07 );
plane = select(pcMerged,indxPlane); % table   
indx = find(pcMerged.Location(:,3) > -0.07);
pcMergedTop = select(pcMerged,indx);
indx2 = find(pcMergedTop.Location(:,2) > 0.2 | pcMergedTop.Location(:,2) < -0.2 | pcMergedTop.Location(:,1) > 0.14 );
pcMergedTop = select(pcMergedTop,indx2);
% pcshow(pcMergedTop);
pause(3);

% Initialize list of obstacles with table
tableMesh = collisionMesh(plane.Location);
tablePointCloud = plane;
hold on
[~,patchObj] = show(tableMesh);
patchObj.LineStyle = 'none';
patches = {patchObj}; 
obstacles = {tableMesh};
segments = {tablePointCloud};
drawnow;
pause(1);

%% Step 4: Segment point cloud
[labels, numClusters] = pcsegdist(pcMergedTop,0.02);

%% Step 5: Create collision meshes for segmented point clouds
for i=1:numClusters
    labelIdx = find(labels==i);
    obstacle = select(pcMergedTop,labelIdx);
    if obstacle.Count > 100
        obstacleMesh = collisionMesh(obstacle.Location);
        hold on
        [~,patchObj] =  show(obstacleMesh);
        patchObj.FaceColor = rand(1,3);
        patchObj.LineStyle = 'none';
        obstacles{end+1} = copy(obstacleMesh);
        segments{end+1} = copy(obstacle);
        patches{end+1} =  patchObj; 
        drawnow;
    end        
end