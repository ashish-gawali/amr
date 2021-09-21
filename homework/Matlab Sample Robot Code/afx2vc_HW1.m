
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Ashish Gawali
% AMR 2021 
% Date: 09/07/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%clear workspace
clear
close all
clc


% Workspace Size
%xlim([0 200])
%ylim([0 200])

%%Simulating for various problems
%[x2_initial, y2_initial, theta2_initial] = problem1();
%[x3_initial, y3_initial, theta3_initial] = problem2(150, 190, 0);
problem3();
%problem4();
%problem5();

%%problem1
function [x2_initial, y2_initial, theta2_initial] = problem1()

%Initial Pose
x_initial = 100;
y_initial = 100;

% Initial Orientation 
angle = pi*rand;
theta_initial = angle;
theta_initial*180/pi;

%Robot final pose
x_final = 20;
y_final = 30;
theta_final = angle + pi/6;
theta_final*180/pi;

robot = SquareRobot(x_initial, y_initial, theta_initial);
robot1 = SquareRobot(x_final, y_final, theta_final);

robot2 = SquareRobot(0, 0, 0);
plot(robot(:,1),robot(:,2),'-*',robot1(:,1),robot1(:,2),'-o');

legend('initial', 'final')
xlim([-10 200])
ylim([-10 200])

x2_initial = x_final;
y2_initial = y_final;
theta2_initial = theta_final;

end

%%problem4
function problem4
%defining the line
x1 = 0; %rand *200;
x2 = 200; %rand *200;
y1= 0; %rand *200;
y2 = 200 %rand *200;

slope = (y2 - y1)/(x2 - x1);
c = y2 - (slope * x2);
a = slope;
b = -1;
plotX1 = -c/a; %y =0
plotX2 = -(c + 300*b)/a; % y= 300
denom = sqrt(a*a + b*b);
theta_d = atan2(-a,b);



%setting the initial positions and speed
speed = 3;
x_initial = rand *50 +50;
y_initial = rand *50 +300;


dt = 0.2;
nSteps = 500;

robotPos_x = [];
robotPos_y = [];
robotPos_theta = [];

robotPos_x(1) = x_initial;
robotPos_y(1) = y_initial;
robotPos_theta(1) = pi*rand;

%defining variables
Kt = 0.5;
Kh = 0.6;

%plotting graphs
robot = SquareRobot(robotPos_x(1),robotPos_y(1),robotPos_theta(1));
plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-');

desired_normalDistance = 10;
for i=1:nSteps
    
    normal_distance = (a*robotPos_x(i) + b*robotPos_y(i) + c)/denom;
    %distance_err = desired_normalDistance - normal_distance
    heading_angle = Kh * (theta_d - robotPos_theta(i));
    
    robotPos_x(i+1) = robotPos_x(i) + speed*cos(robotPos_theta(i))*dt ;%+ 10* (-sin(robotPos_theta(i)));
    robotPos_y(i+1) = robotPos_y(i) + speed*sin(robotPos_theta(i))*dt ;%+ 10 * cos(robotPos_theta(i));
    robotPos_theta(i+1) = -Kt * normal_distance + Kh * heading_angle;
    %robotPos_theta(i+1) = Kh * heading_angle;
    
    robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
    plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-');
    xlim([-100 400])
    ylim([-100 400])
    pause(0.01)
end
hold on
line([plotX1 plotX2], [0  300])
hold off

end

function problem5()
%considering constant linear and angular speed
linear_speed = 5;
angular_speed = pi/10;

x_start = 10;
y_start = 10;
theta_start = 0;

x_goal = 130;
y_goal = 130;
theta_goal = pi/4;

%time
dt = 0.1;

%number of steps
nSteps = 500;

%initialsing the vector of robot poses
robotPos_x = [];
robotPos_y = [];
robotPos_theta = [];

%setting the starting pose
robotPos_x(1) = x_start;
robotPos_y(1) = y_start;
robotPos_theta(1) = theta_start;

%setting gains
Kp = 0.3; % Kp > 0
Kb = -0.1; % Kb < 0
Kalpha = 0.4; % Kalpha - Kp > 0

%defining vecors for delta, alpha and  beta
delta_vec = [];
alpha_vec = [];
beta_vec = [];

prev_beta = 5;
prev_alpha = 5;
prev_delta = 5;
for i = 1:nSteps
    flag = 0;
    delta_x = x_goal - robotPos_x(i);
    delta_y = y_goal - robotPos_y(i);
    
    delta = sqrt(delta_x * delta_x + delta_y * delta_y);
    alpha = atan2(delta_y, delta_x) - robotPos_theta(i);
    beta = -robotPos_theta(i)- alpha;
    
    speed = Kp * delta;
    ang_speed = Kalpha * alpha + Kb * beta;
    
    robotPos_x(i+1) = robotPos_x(i) + speed * cos(robotPos_theta(i)) * dt;
    robotPos_y(i+1) = robotPos_y(i) + speed * sin(robotPos_theta(i)) * dt;
    robotPos_theta(i+1) = robotPos_theta(i) + ang_speed * dt;
    
    %getting the robot state and plotting it
    robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
    plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-');
    xlim([-10 210])
    ylim([-10 210])
    pause(0.01)
    
    distance_err = sqrt((x_goal -robotPos_x(i)).^2 +(y_goal - robotPos_y(i)).^2)
    
    if distance_err<0.5
        robotPos_x(i+1) = robotPos_x(i);
        robotPos_y(i+1) = robotPos_y(i);
        flag = 1;
    end
    
    if theta_goal - robotPos_theta(i)<0.01
        flag = flag +1;
    end
    
    if flag == 2
        break;
    end
end
assignin('base','theta',robotPos_theta);
assignin('base','robotPos_x',robotPos_x);
assignin('base','robotPos_y',robotPos_y);
end

