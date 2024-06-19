
clc;
clear all;
close all;

% Initialize the robot
robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 4);

% Define and add the first body and its joint
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('joint1', 'revolute'); % Rotation around the base
setFixedTransform(jnt1, trvec2tform([0 0 0]));
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Define and add the second body and its joint
body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('joint2', 'revolute'); % 1st link around base
setFixedTransform(jnt2, trvec2tform([0 0 0.25])); % Assume 0.25 meter length for link1
jnt2.JointAxis = [0 1 0];
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% Define and add the third body and its joint
body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('joint3', 'revolute');
setFixedTransform(jnt3, trvec2tform([0 0 1])); % Assume 1 meter length for link2
jnt3.JointAxis = [0 1 0]; % 2nd link around 1st
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

% Define and add the fourth body
body4 = rigidBody('link4');
jnt4 = rigidBodyJoint('fixed'); % Fixed joint for end effector
setFixedTransform(jnt4, trvec2tform([1 0 0])); % 1 meter length for link3
body4.Joint = jnt4;
addBody(robot, body4, 'link3');

showdetails(robot);

% Add collision objects to the robot links
collisionObjBody2 = collisionCylinder(0.05, 0.20);
collisionObjBody2.Pose = trvec2tform([0 0 0.125]); % Adjust position along the link
addCollision(robot.Bodies{1}, collisionObjBody2);

collisionObjBody3 = collisionCylinder(0.05, 0.8);
collisionObjBody3.Pose = trvec2tform([0 0 0.5])
addCollision(robot.Bodies{2}, collisionObjBody3);

collisionObjBody4 = collisionCylinder(0.05, 0.8);
collisionObjBody4.Pose = trvec2tform([0.5 0 0]) * axang2tform([0 1 0 pi/2]);
addCollision(robot.Bodies{3}, collisionObjBody4);


% Define a static obstacle
obstacle = collisionBox(1, 1, 1); % Create collision box
obstacle.Pose = trvec2tform([0 -0.75 1]); % Set box position away from the robot's start position

% Define start and end joint configurations
jointAnglesStart = [0, 0, 0];
jointAnglesEnd = [-1/2*pi, pi/2, -pi/2];


% Plan path using RRT* algorithm
targets = flipud(RRTstar(jointAnglesEnd,robot,obstacle)); % RRT* path planner

% Initialize joint angles array
jointAngles = [];
i = size(targets);
d = 0;


% Generate intermediate configurations
for j = 1:i(1)-1
    x = linspace(targets(j, 1), targets(j+1, 1), 20);
    y = linspace(targets(j, 2), targets(j+1, 2), 20);
    z = linspace(targets(j, 3), targets(j+1, 3), 20);
    
    temp = [x; y; z]';
    jointAngles = [jointAngles; temp];
    d = d + 20;
end

% Visualize the robot in the new configurations
figure;
title('3-DOF Robot in Specified Configuration');

i = 1;
collisionDetected = false;

while d > 1
    i = i + 1;
    d = d - 1;
    % Update the robot configuration
    config = jointAngles(i, :);
    
% Check for collisions
if checkCollisionState(robot, config, obstacle)
    collisionDetected = true;
    show(robot, config, 'Collisions', 'on', 'Visuals', 'off');
    show(obstacle);
    title("Collision Detected");
    disp('Collision detected!');
    pause(0.5);
    break; % Stop the loop if collision is detected
else
    % Visualize the robot in the current configuration
    show(robot, config, 'Collisions', 'off', 'Visuals', 'on');
    hold on;
    show(obstacle);
    hold off;
    % Pause to create animation effect
    pause(0.05);
end
end

% Output collision result
if collisionDetected
    disp('Collision was detected during the robot motion.');
else
    disp('No collision detected during the robot motion.');
end


