% go to photo_location
goto(init_pose);

% get highest score bounding box + label
rgb=get_image('r');
di=get_image('d');
[prediction, bboxes, scores, labels] = classify(net,rgb);
[max_score, max_index] = max(scores);
max_bbox = bboxes(max_index,:);
max_label = labels(max_index);
% project to 3d
centroid = round([max_bbox(1) + max_bbox(3) * 0.5, max_bbox(2) + max_bbox(4) * 0.5]);
bbox_3d = p3D_from_p2D(centroid,di,"world");

% get nearest item from fixed_places shape change objects
shape_change_objects = [object_1 object_2 object_3 object_4 object_5];

errs = zeros(1,5);
for i=1:size(shape_change_objects,2)
    obj = shape_change_objects(1:3,i);
    errs(i) = norm(bbox_3d-obj);
end

[min_error,min_index] = min(errs);


best_object = shape_change_objects(:,min_index);

best_object_approach = best_object + z_approach;

% if can ...
if max_label=="can"
    if min_index == 1 || min_index == 2 || min_index == 5
        best_object = best_object - [0;0;0.07;0;0;0];
    end
    goto(best_object_approach);
    goto(best_object);
    SLActivateGripper(1);
    goto(best_object_approach);
    goto(init_pose);
    goto(can_bin)
    SLActivateGripper(0);
% if bottle ...
else
    goto(best_object_approach);
    goto(best_object);
    SLActivateGripper(1);
    goto(best_object_approach);
    goto(init_pose);
    goto(bottle_bin)
    SLActivateGripper(0);
end