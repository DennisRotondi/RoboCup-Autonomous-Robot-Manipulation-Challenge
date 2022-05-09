function image = get_image(depth_or_rgb)
%GET_IMAGE takes as input a paramete depth_or_rgb={'d','r'}
%   to specify if you want the depth or rgb image.
    assert(depth_or_rgb=='d' || depth_or_rgb=='r') 
    if depth_or_rgb=='r'
        persistent rgbImgSub
        rgbImgSub = rossubscriber('/camera/color/image_raw');
        curImage = receive(rgbImgSub);
        image = readImage(rgbImgSub.LatestMessage);
    else
        persistent rgbDptSub
        rgbDptSub = rossubscriber('/camera/depth/image_raw');
        curDepth = receive(rgbDptSub);
        image = readImage(rgbDptSub.LatestMessage); 
    end
end

