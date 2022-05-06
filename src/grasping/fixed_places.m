% locations of interest in the base_link frame

init_pose_q = [ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847];
init_tf = getTransform(robot, init_pose_q, 'gripper');
init_pose_t = init_tf(1:3,4);
init_pose_a = rotm2eul(init_tf(1:3,1:3),"xyz");

init_pose = [init_pose_t' init_pose_a]';

goto = @(pose) (moveGripperTo(robot,trajAct,trajGoal,jointSub,pose(1:3)',pose(4:6)'));
z_approach = [0;0;0.3;0;0;0];

blue_bottle = [-0.0260, -0.5780, 0.14, 0, pi, pi/2]';
blue_bottle_approach = blue_bottle + z_approach;


yellow_bottle = [-0.05,0.63,-0.05,0,pi,pi/2]';
yellow_bottle_approach = yellow_bottle + z_approach;

bottle_bin = [-0.35,-0.5,0.3,0,pi,pi/2]';
can_bin = [   -0.35, 0.5,0.3,0,pi,pi/2]';

can_in_boxes = [0.34,0.54,0.035,0, pi, -pi/4]';
can_in_boxes_approach = can_in_boxes + z_approach;

red_can_0 = [0.75,0.55,0.3,-2.7540,0.3614,-0.8571]';
red_can_1 = [0.75,0.55,0.15,-2.8455,0.2839,-0.8280]';
red_can_2 = [0.79,0.62,-0.05,-2.8455,0.2839,-0.8280]';

can_on_boxes = [0.02, 0.465,0.12,0,pi,pi/2]';
can_on_boxes_approach = can_on_boxes + z_approach;

red_bottle_edge_0 = [0.20,0.72,0.05,0,2*pi/3,pi/2]';
red_bottle_edge_1 = [0.28,0.715,-0.05,0,5*pi/6,pi/2]';
red_bottle_edge_2 = [0.2,0.73,0.3,0,5*pi/6,pi/2]';

%% shape change region

object_1 = [0.20,0.36,0.1,0,pi,pi/2]'; %% might not pick up if it's a can
object_1_approach = object_1+z_approach;

object_2 = [0.43,0.195,0.1,0,pi,pi/2]'; %% might not pick up if it's a can
object_2_approach = object_2+z_approach;

object_3 = [0.205+0.14,-0.102+0.11,-0.03,0, pi, 0]';
object_3_approach = object_3+z_approach;

object_4 = [0.205+0.15,-0.175,-0.04,0,pi,pi/2]';
object_4_approach = object_4+z_approach;

object_5 = [0.26,-0.44,0.1,0,pi,pi/2]'; %% might not pick up if it's a can
object_5_approach = object_5+z_approach;

