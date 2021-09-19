
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
%[x3_initial, y3_initial, theta3_initial] = problem2(x2_initial, y2_initial, theta2_initial);
%problem3();
%problem4();

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
Kp = 0.1; % Kp > 0
Kb = -0.1; % Kb < 0
Kalpha = 0.2; % Kalpha - Kp > 0

%defining vecors for delta, alpha and  beta
delta_vec = [];
alpha_vec = [];
beta_vec = [];

prev_beta = 5;
prev_alpha = 5;
prev_delta = 5;
for i = 1:nSteps
    delta_x = x_goal - robotPos_x(i);
    delta_y = y_goal - robotPos_y(i);
    
    delta = sqrt(delta_x * delta_x + delta_y * delta_y);
    alpha = atan2(delta_y, delta_x) - robotPos_theta(i);
    beta = -robotPos_theta(i)- alpha;
    
     
    if beta<0.01 && beta>-0.01 || prev_beta == 0
        beta = 0;
    end

    if alpha<0.01 && alpha>-0.01
        alpha = 0;
    end

    if delta<3 && delta>-3 || prev_delta == 0
        delta = 0;
    end

    
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
    
    delta_vec(i) = delta;
    alpha_vec(i) = alpha;
    beta_vec(i) = beta;
    
    prev_delta = delta;
    prev_alpha = alpha;
    prev_beta = beta;
    if delta == 0 && alpha == 0 && beta == 0
        break;
    end

end

%%problem1
function [x2_initial, y2_initial, theta2_initial] = problem1()

%Initial Pose
x_initial = 100;
y_initial = 100;

% Initial Orientation 
angle = pi*rand;
theta_initial = angle;
theta_initial*180/pi

%Robot final pose
x_final = 20;
y_final = 30;
theta_final = angle + pi/6;
theta_final*180/pi

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

%%problem 2
function [x3_initial, y3_initial, theta3_initial] = problem2(x2_initial, y2_initial, theta2_initial)

%%Starting work on problem 2
%Getting the final pose
x2_final = rand*200;
y2_final = rand*200;
%theta2_final = rand*pi;

%Setting the velocity
Kv = 0.1; %velocity constant

%steering angle
theta2_final = atan2((y2_final - y2_initial),(x2_final - x2_initial));
Kp = 0.1;
steering = Kp*(theta2_final - theta2_initial);

%defining x and y values
x2 = [];
y2 = [];
theta2 = [];
vel2 = [];

x2(1) = x2_initial;
y2(1) = y2_initial;
theta2(1) = theta2_initial;
vel2(1) = 0; %in metres per sec

%steps to in the plot
nSteps = 1000;

%time step
dt = 0.1;

for i = 1:nSteps
    
    %udpating x and y coordinates
    x2(i+1) = x2(i) + vel2(i)*cos(theta2(i))*dt;
    y2(i+1) = y2(i) + vel2(i)*sin(theta2(i))*dt;
    
    %updating velocity with each set of coordinates
    vel2(i+1) = Kv*sqrt((x2(i+1) - x2_final)*(x2(i+1) - x2_final) + (y2(i+1)-y2_final)*(y2(i+1)-y2_final));
    if(vel2(i+1)>5)
        vel2(i+1) = 5;
    end
    
    %getting theta
    theta2(i+1) = atan2((y2_final - y2(i+1)),(x2_final - x2(i+1)));
    
    %getting the robot state and plotting it
    robot = SquareRobot(x2(i),y2(i),theta2(i));
    plot(robot(:,1),robot(:,2),'-',x2,y2,'-');
    xlim([-10 210])
    ylim([-10 210])
    pause(0.01)
end

%Setting xAxis based on the number of steps
xAxis = 1:0.1:(0.1*nSteps)+1;

%Getting a new figure to plot the velocity w.r.t time
figure
plot(xAxis, vel2)
title('Velocity')
xlim([0 0.1*nSteps])
ylim([0 6])

x3_initial = x2_final;
y3_initial = y2_final;
theta3_initial = theta2_final;
end

%%problem 3
function problem3()

x3_initial = rand*200;
y3_initial = rand*200;
theta3_initial =  pi*rand;


%Initlializing the 3 points
points_x = [rand*200 rand*200 rand*200];
points_y = [rand*200 rand*200 rand*200];

%Decalring the velocity and pose as vectors
v3 = [];
x3 = [];
y3 = [];
theta3 = [];

%initlializing for first point
vel3(1) = 0;
x3(1) = x3_initial;
y3(1) = y3_initial;
theta3(1) = theta3_initial;


%number of steps
nSteps = 500;

%time period
dt = 0.1;

%Setting up things for pid
vel_desired = 3;
integral = 0;
kp = 0.5;
ki = 0.001;
kd = 0.01;
previous_error = 0;


robot = SquareRobot(x3(1), y3(1), theta3(1));
plot(robot(:,1),robot(:,2),'-');
xlim([-10 210])
ylim([-10 210])

x3_loc = [];
y3_loc = [];
theta3_all = [];
vel3_all = [];

isComplete = 0;
indx = 1;




for j = 1:length(points_x)
    x3_final = points_x(j);
    y3_final = points_y(j);
    theta3_final = 0;
    vel3_final = 0;
    for i=1:nSteps
        
        %setting positions
        x3(i+1) = x3(i) + vel3(i)*cos(theta3(i))*dt;
        y3(i+1) = y3(i) + vel3(i)*sin(theta3(i))*dt;
        
        %getting velocity for next run
        err = vel_desired - vel3(i);
        integral = integral + err*dt;
        derivative = (err - previous_error)/dt;
        pid_output =  kp*err + ki*integral + kd * derivative; 
        vel3(i+1) = vel3(i) + pid_output - 0.01*vel3(i);
        
        %shit = mod(atan2(y,x),2*pi);
        %getting theta
        theta3(i+1) = atan2((y3_final - y3(i+1)),(x3_final - x3(i+1)));
        
        
        if theta3(i+1)*theta3(i)<0 && i>3
            theta3(i+1) = theta3(i);
            theta3_final = theta3(i);
            vel3(i+1) = vel3(i);
            vel3_final = vel3(i);
            break;
        end
        
        
        %plotting the bot
        %getting the robot state and plotting it
        %{
        robot = SquareRobot(x3(i),y3(i),theta3(i));
        plot(robot(:,1),robot(:,2),'-',x3,y3,'-');
        xlim([-10 210])
        ylim([-10 210])
        pause(0.01)
        %}
    end
    
    x3_loc = [x3_loc x3];
    y3_loc = [y3_loc y3];
    theta3_all = [theta3_all theta3];
    vel3_all = [vel3_all vel3];
    
    %reinitalizing    
    vel3(1) = vel3_final;
    x3(1) = points_x(j);
    y3(1) = points_y(j);
    theta3(1) = theta3_final;
    
end

for i= 1:length(x3_loc)
    robot = SquareRobot(x3_loc(i),y3_loc(i),theta3_all(i));
    plot(robot(:,1),robot(:,2),'-',x3_loc,y3_loc,'-');
    xlim([-10 210])
    ylim([-10 210])
    pause(0.01)
end

%Getting a new figure to plot the velocity w.r.t time
figure
plot(vel3_all)
title('Velocity')
ylim([0 4])

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
x_initial = rand *50 +100;
y_initial = rand *50 +50;


dt = 0.1;
nSteps = 500;

robotPos_x = [];
robotPos_y = [];
robotPos_theta = [];

robotPos_x(1) = x_initial;
robotPos_y(1) = y_initial;
robotPos_theta(1) = pi*rand;

%defining variables
Kt = 0.06;
Kh = 0.04;

%plotting graphs
robot = SquareRobot(robotPos_x(1),robotPos_y(1),robotPos_theta(1));
plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-');

for i=1:nSteps
    normal_distance = (a*robotPos_x(i) + b*robotPos_y(i) + c)/denom;
    heading_angle = Kh * (theta_d - robotPos_theta(i));
    
    robotPos_x(i+1) = robotPos_x(i) + speed*cos(robotPos_theta(i))*dt ;%+ 10* (-sin(robotPos_theta(i)));
    robotPos_y(i+1) = robotPos_y(i) + speed*sin(robotPos_theta(i))*dt ;%+ 10 * cos(robotPos_theta(i));
    robotPos_theta(i+1) = -Kt * normal_distance + Kh * heading_angle;
    %robotPos_theta(i+1) = Kh * heading_angle;
    
    robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
    plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-');
    xlim([-100 210])
    ylim([-100 210])
    pause(0.01)
end
hold on
line([plotX1 plotX2], [0  300])
hold off

end
