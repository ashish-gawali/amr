function [pose] = problem1_extra()
%% Parameters
%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

x_initial = 100;
y_initial = 100;

% Initial Orientation 
angle = pi*rand;
theta_initial = angle;
theta_initial*180/pi

%%Robot final pose
x_final = 20;
y_final = 30;
theta_final = angle + pi/6;
theta_final*180/pi


%%get robot path
%number of steps of the simualtion
nstep = 50;

%defining the step size
x_nSteps = abs(x_final - x_initial)/nstep;
y_nSteps = abs(y_final - y_initial)/nstep;
theta_steps = abs(theta_final - theta_initial)/nstep;

if x_final < x_initial
    x_nSteps = -x_nSteps;
end
if y_final < y_initial
    y_nSteps = -y_nSteps;
end
if theta_final < theta_initial
    theta_steps = -theta_steps;
end
%defining the poses
x = x_initial:x_nSteps:x_final;
y = y_initial:y_nSteps:y_final;
theta = theta_initial:theta_steps:theta_final;

%% Build Robot Model
robot = SquareRobot(x(1),y(1),theta(1));

str1 = 'initial theta = ' + string(theta_initial);
str2 = 'final theta = ' + string(theta_final);
str = {{str1,str2}};

plot(robot(:,1),robot(:,2),'-');
%text(180,180,str);
xlim([0 200])
ylim([0 200])
pause(5)
%% Move Robot



%time step
dt = 0.1;

%tanslating
for i = 1:nstep
    
    %robot non-holonomic dynamics (as seen in class)
    %x(i+1) = x_err + vel * cos(theta(i)) * dt;
    %y(i+1) = y_err + vel * sin(theta(i)) * dt;
    %theta(i+1) = theta(i) + steering * dt;
    
    robot = SquareRobot(x(i),y(i),theta(1));
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end

%rotating
for i = 1:nstep
    
    %robot non-holonomic dynamics (as seen in class)
    %x(i+1) = x_err + vel * cos(theta(i)) * dt;
    %y(i+1) = y_err + vel * sin(theta(i)) * dt;
    %theta(i+1) = theta(i) + steering * dt;
    
    robot = SquareRobot(x(nstep+1),y(nstep+1),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end
pose = [x_final; y_final; theta_final];
end