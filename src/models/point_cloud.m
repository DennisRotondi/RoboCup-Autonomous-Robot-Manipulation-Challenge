%references:    https://ch.mathworks.com/help/vision/point-cloud-processing.html
%               https://ch.mathworks.com/help/vision/ref/pctransform.html
%               https://ch.mathworks.com/help/vision/ref/pcshow.html
%               https://ch.mathworks.com/help/vision/ref/pcregistericp.html
ptCloud = pcread('models/bottle.ply');
ptCloud2 = pcread('models/can.ply');

%TODO downsample, remember that in gazebo models are scaled, https://ch.mathworks.com/help/vision/ref/pcdownsample.html
% percentage = 0.041;
% orgPtCloudOut = pcdownsample(orgPtCloud,'random',percentage,PreserveStructure=true);

% pcshow(ptCloud)
% pcshow(ptCloud2)

%example trasform
theta = pi/4;

rot = [ 1        0          0; 
        0 cos(theta) sin(theta) ; 
        0 -sin(theta) cos(theta) ]
       
trans = [2, 5, 6];
tform = rigid3d(rot,trans);

ptCloudOut = pctransform(ptCloud,tform);
pcshowpair(ptCloud,ptCloudOut)
pause
%adding a possible InitialTransform=tform improve drastically the computation if
%rotate better add this rotation
[tsf,transformed,ms1]=pcregistericp(ptCloudOut,ptCloud,Verbose=false); %args order: moving, fixed
[tsf2,transformed2,ms2]=pcregistericp(ptCloudOut,ptCloud2,Verbose=false,InitialTransform=tsf); %args order: moving, fixed
pcshowpair(ptCloud,transformed)
disp(ms1)
disp(ms2)
%now I want to classify if ptCloudOut is a bottle or a can
if ms1>ms2
    disp("It is a can")
    disp("reliability "+(1-ms2/ms1))
else
    disp("It is a bottle")
    disp("reliability "+(1-ms1/ms2))
end
