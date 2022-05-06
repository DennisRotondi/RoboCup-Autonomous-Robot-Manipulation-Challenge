function pc_filtered = get_pointcloud(ptcloud,point)
%GOTO_PLACE Summary of this function goes here
[indices,~] = findNeighborsInRadius(ptCloud,point,0.5);
pc_filtered = select(ptcloud,indices);
end

%object_2 = [0.205+0.14,-0.102+0.11,-0.03,0, pi, 0]';