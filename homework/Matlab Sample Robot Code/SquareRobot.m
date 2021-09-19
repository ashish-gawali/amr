%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Ashish Gawali(UVA)
% AMR 2021 
% Date: 09/11/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [robot] = SquareRobot(x,y,theta)

center = [x y];

% Robot Square Shape
const = 10;
a = [0, const/2];
b = [const/2, const/2];
c = [2*const, 0];
d = [const/2, -const/2];
e = [0, -const/2];

%Rotation Matrix
rotmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rota = (rotmat * (a'));
rotb = (rotmat * (b'));
rotc = (rotmat * (c'));
rotd = (rotmat * (d'));
rote = (rotmat * (e'));

%final robot config after tranformation
robot1 = [rota(1) + center(1), rota(2) + center(2)];
robot2 = [rotb(1) + center(1), rotb(2) + center(2)];
robot3 = [rotc(1) + center(1), rotc(2) + center(2)];
robot4 = [rotd(1) + center(1), rotd(2) + center(2)];
robot5 = [rote(1) + center(1), rote(2) + center(2)];


robot = [robot1;robot2;robot3;robot4;robot5;robot1];
end