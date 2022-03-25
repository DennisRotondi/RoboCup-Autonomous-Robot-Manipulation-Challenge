function started = StartSimulation(IP)
    started = false;
    try
        rosIP = IP; % IP address of ROS enabled machine  
        rosshutdown; % shut down existing connection to ROS
        rosinit(rosIP,11311);
        
        load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
        
        RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
        physicsClient = rossvcclient('gazebo/unpause_physics');
        physicsResp = call(physicsClient,'Timeout',3);
        started = true;
        pause(1);
    catch
        fprintf('Error in starting the simulation');
    end
end

