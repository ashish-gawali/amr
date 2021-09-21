%%problem 3
function problem3()

x_initial = rand*200;
y_initial = rand*200;
theta_initial =  pi*rand;


%Initlializing the 3 points
goal_x = [rand*50 50+rand*100 150+rand*50];
goal_y = [rand*50 50+rand*100 150+rand*50];

%Decalring the velocity and pose as vectors
velocity = [];
robotPos_x = [];
robotPos_y = [];
robotPos_theta = [];

%number of steps
nSteps = 500;

%time period
dt = 0.1;

%Setting up things for pid
vel_desired = 3;
integral = 0;
kp = 0.3;
ki = 0.001;
kd = 0.01;
kSteering = 0.5;
previous_error = 0;

xlim([-10 210])
ylim([-10 210])

% robotPos_x_all = [];
% robotPos_y_all = [];
% robotPos_theta_all = [];
% velocity_all = [];

i = 1;
robotPos_x(i) = x_initial;
robotPos_y(i) = y_initial;
robotPos_theta(i) = theta_initial;
velocity(i) = 0;

%{
for j = 1:length(goal_x)
    x_goal = goal_x(j);
    y_goal = goal_y(j);
    theta3_final = 0;
    vel3_final = 0;
    
    
    while 1
        
        %setting positions
        robotPos_x(i+1) = robotPos_x(i) + velocity(i)*cos(robotPos_theta(i))*dt;
        robotPos_y(i+1) = robotPos_y(i) + velocity(i)*sin(robotPos_theta(i))*dt;
        
        %getting velocity for next run
        err = vel_desired - velocity(i);
        integral = integral + err*dt;
        derivative = (err - previous_error)/dt;
        pid_output =  kp*err + ki*integral + kd * derivative; 
        velocity(i+1) = velocity(i) + pid_output - 0.01*velocity(i);
        
        %theta_0to2pi = mod(atan2(y,x),2*pi);
        %getting theta
        robotPos_theta(i+1) = atan2((y_goal - robotPos_y(i+1)),(x_goal - robotPos_x(i+1)));
        
        distance_err = sqrt((robotPos_x(i+1) - x_goal).^2 + (robotPos_y(i+1) - y_goal).^2);
        
        if distance_err < 0.4
            robotPos_x(i+1) = robotPos_x(i);
            robotPos_y(i+1) = robotPos_y(i);
            robotPos_theta(i+1) = robotPos_theta(i);
            
            %theta3_final = robotPos_theta(i);
            velocity(i+1) = velocity(i);
            %vel3_final = velocity(i);
            break;
        end
%         if theta3(i+1)*theta3(i)<0 && i>3
%             theta3(i+1) = theta3(i);
%             theta3_final = theta3(i);
%             vel3(i+1) = vel3(i);
%             vel3_final = vel3(i);
%             break;
%         end
        
        
        %plotting the bot
        %getting the robot state and plotting it
        
        robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
        plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-',goal_x,goal_y,'o');
        xlim([-10 210])
        ylim([-10 210])
        pause(0.01)
        
        i=i+1;
    end
    
    
    
    
end
%}
for j = 1:length(goal_x)
    x_goal = goal_x(j);
    y_goal = goal_y(j);
    theta3_final = 0;
    vel3_final = 0;
    
    
    while 1
        distance = sqrt((x_goal - robotPos_x(i)).^2 + (y_goal - robotPos_y(i)).^2);
        
        robotPos_x(i+1) = robotPos_x(i) + dt*velocity(i)*cos(robotPos_theta(i));
        robotPos_y(i+1) = robotPos_y(i) + dt*velocity(i)*sin(robotPos_theta(i));
        
        %getting velocity for next run
        err = vel_desired - velocity(i);
        integral = integral + err*dt;
        derivative = (err - previous_error)/dt;
        pid_output =  kp*err + ki*integral + kd * derivative; 
        velocity(i+1) = velocity(i) + pid_output - 0.01*velocity(i);
        
        %getting theta
        theta_goal(i) = atan2((y_goal-robotPos_y(i)),(x_goal - robotPos_x(i)));
        theta_err(i) = theta_goal(i) - robotPos_theta(i);
        steering = kSteering* theta_err(i);
        steering = atan2(sin(steering),cos(steering));
        robotPos_theta(i+1) = steering*dt + robotPos_theta(i);

        i = i+1;
        if distance<0.5
            break
        end
        
        %getting the robot state and plotting it
        robot = SquareRobot(robotPos_x(i),robotPos_y(i),robotPos_theta(i));
        plot(robot(:,1),robot(:,2),'-',robotPos_x,robotPos_y,'-', x_goal, y_goal, 'o');
        xlim([-10 210])
        ylim([-10 210])
        pause(0.01)

%         if(i>1000)
%             break;
%         end
    end
end

% assignin('base','velocity',velocity);
% assignin('base','robotPos_x',robotPos_x);
% assignin('base','robotPos_y',robotPos_y);
%Getting a new figure to plot the velocity w.r.t time
figure
plot(velocity)
title('Velocity')
ylim([0 4])

end
