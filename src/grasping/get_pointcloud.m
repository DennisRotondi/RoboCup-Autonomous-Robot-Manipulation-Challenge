function item = get_pointcloud(ptcloud,point)
%GOTO_PLACE Summary of this function goes here
[indices,~] = findNeighborsInRadius(ptcloud,point,0.13);
pc_filtered = select(ptcloud,indices);
[labels,numClusters] = pcsegdist(pc_filtered,0.05);
cmax=-1; %current num max of points in pcloud
for i=1:numClusters
    temp=find(labels==i);
    obj=select(pc_filtered,temp);
    if obj.Count>cmax
        cmax=obj.Count;
        item = obj;
    end
end
for i=1:4
    pcomp = get_direction(item);
    [model,inlierIndices] = pcfitcylinder(item,0.02,pcomp);
    item = select(pc_filtered,inlierIndices);
end
end

%object_2 = [0.205+0.14,-0.102+0.11,-0.03,0, pi, 0]';