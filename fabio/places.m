% locations of interest in the base_link frame

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
