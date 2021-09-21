%%problem 2
function [x3_initial, y3_initial, theta3_initial] = problem2(x2_initial, y2_initial, theta2_initial)

%%Starting work on problem 2
%Getting the final pose
x_goal = rand*200;
y_goal = rand*200;

%defining x and y values
robotPos_x = [];
robotPos_y = [];
robotPos_theta = [];
velocity = [];
theta_err = []; 

robotPos_x(1) = x2_initial;
robotPos_y(1) = y2_initial;
robotPos_theta(1) = theta2_initial;
velocity(1) = 0; %in metres per sec
theta_err(1) = theta2_initial;

%steps to in the plot
nSteps = 500;

%time step
dt = 0.25;

%Setting the Gains
Kp = 0.5; %for turning
Kv = 0.1; %velocity constant

i = 1; 
distance(i) = (((robotPos_x(i)-x_goal).^2 + (robotPos_y(i)-y_goal).^2).^0.5);
robotPos_theta(i) = theta2_initial;
velocity(i) = 0;
theta_goal(i) = atan2((y_goal - robotPos_y(i)),(x_goal - robotPos_x(i)));
%i = i+1;

while distance(i)>0.001
    theta_goal(i) = atan2((y_goal - robotPos_y(i)),(x_goal - robotPos_x(i)));
    
    robotPos_x(i+1) = robotPos_x(i) + dt*velocity(i)*cos(robotPos_theta(i));
    robotPos_y(i+1) = robotPos_y(i) + dt*velocity(i)*sin(robotPos_theta(i));
    distance(i+1) = (((robotPos_x(i+1)-x_goal).^2 + (robotPos_y(i+1)-y_goal).^2).^0.5);
    velocity(i+1) = Kv*distance(i+1);
    
    if velocity(i+1)>5
        velocity(i+1) = 5;
    end
    
    %getting steering
    theta_err(i) = theta_goal(i) - robotPos_theta(i);
    steering = Kp* theta_err(i);
    steering = atan2(sin(steering),cos(steering));
    robotPos_theta(i+1) = steering*dt + robotPos_theta(i);
    
    
    %getting the robot state and plotting it
    robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
    plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-', x_goal, y_goal, 'o');
    xlim([-10 210])
    ylim([-10 210])
    pause(0.01)
    
    i = i+1;
    if(i>1000)
        break;
    end
end


%{
for i = 1:nSteps
    
    %udpating x and y coordinates
    robotPos_x(i+1) = robotPos_x(i) + velocity(i)*cos(robotPos_theta(i))*dt;
    robotPos_y(i+1) = robotPos_y(i) + velocity(i)*sin(robotPos_theta(i))*dt;
    
    %setting velocity of next iteration
    distance  = sqrt((robotPos_x(i+1) - x_goal).^2 +  (robotPos_y(i+1) - y_goal).^2);
    velocity(i+1) = Kv*distance;
    %updating velocity with each set of coordinates
    if(velocity(i+1)>5)
        velocity(i+1) = 5;
    end
    
    %getting theta
    theta_err(i) = theta_goal - robotPos_theta(i);
    steering = Kp* (theta_goal - robotPos_theta(i));
    steering = atan2(sin(steering),cos(steering));
    robotPos_theta(i+1) = steering*dt + robotPos_theta(i);
    
    if (distance<=0.5)
        break;
    end
    %getting the robot state and plotting it
    robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
    plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-', x_goal, y_goal, 'o');
    xlim([-10 210])
    ylim([-10 210])
    pause(0.01)
end
%}

assignin('base','velocity',velocity);
assignin('base','robotPos_x',robotPos_x);
assignin('base','robotPos_y',robotPos_y);
assignin('base','theta_err',theta_err);


%Getting a new figure to plot the velocity w.r.t time
figure
plot(velocity)
title('Velocity')
xlim([0 nSteps])
ylim([0 6])

x3_initial = x_goal;
y3_initial = y_goal;
theta3_initial = theta_goal;
end