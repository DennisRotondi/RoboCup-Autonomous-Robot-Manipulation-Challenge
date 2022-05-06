function tf = get_camera_transf
    tr = getTransform(rostf,frame_id,'camera_link');
    attemptn=0;
    while(size(tr) == [0,1])
        pause(1)
        tf = getTransform(rostf,'world','camera_link');
        attemptn=attemptn+1;
        if attemptn > 5
            tf = "fail";
            return
        end
    end
end

