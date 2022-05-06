function pc = get_direction(ptcloud)
%GOTO_PLACE Summary of this function goes here
pcs = pca(ptcloud.Location);
pc = pcs(:,1);
end

