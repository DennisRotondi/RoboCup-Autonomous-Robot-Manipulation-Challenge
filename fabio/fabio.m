places

goto = @(pose) (moveGripperTo(robot,trajAct,trajGoal,jointSub,pose(1:3)',pose(4:6)'));

goto(blue_bottle_approach);
goto(blue_bottle);
SLActivateGripper(1);
goto(blue_bottle_approach);
goto(bottle_bin);
SLActivateGripper(0);

goto(yellow_bottle_approach);
goto(yellow_bottle);
SLActivateGripper(1);
goto(yellow_bottle_approach);
goto(can_bin); % avoid collision with can
goto(bottle_bin);
SLActivateGripper(0);

goto(can_in_boxes_approach);
goto(can_in_boxes);
SLActivateGripper(1);
goto(can_in_boxes_approach);
goto(can_bin);
SLActivateGripper(0);

goto(red_can_0);
goto(red_can_1);
goto(red_can_2);
SLActivateGripper(1);
goto(red_can_0);
goto(can_bin);
SLActivateGripper(0);

goto(can_on_boxes_approach);
goto(can_on_boxes);
SLActivateGripper(1);
goto(can_on_boxes_approach);
goto(can_bin);
SLActivateGripper(0);
