if not(exist('started','var') && started)
    started = StartSimulation("192.168.1.104");
end

[gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
gripperCommand = rosmessage('control_msgs/GripperCommand');
gripperCommand.Position = 0.0;  % 0 is open, 0.04 is closed. No intermidiate values
gripGoal.Command = gripperCommand;
sendGoal(gripAct,gripGoal);


