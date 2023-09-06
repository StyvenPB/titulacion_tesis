function [T] = cdir(q)

% Medidas de los eslabones en cm
l1= 17.75; l2=10; l3= 7.75;  l4= 15; l5= 12.25; l6= 7.75; l7= 10;  l8= 12.25;  l9= 12.25; 

T01 = transfDH(l1, (-90*pi/180)+q(1),   -l2,  90*pi/180);
T12 = transfDH(0, (180*pi/180)+q(2),     l3,  90*pi/180);
T23 = transfDH(l4, (90*pi/180)+q(3),      0,  90*pi/180);
T34 = transfDH(l5, (270*pi/180)+q(4),    l6,  0);
T45 = transfDH(l7, (90*pi/180)+q(5),      0,  90*pi/180);
T56 = transfDH(l8, (180*pi/180)+q(6),     0,  90*pi/180);
T67 = transfDH(l9, q(7),                  0,  0);

T= T01*T12*T23*T34*T45*T56*T67;  % Matriz de transformación homogenea 
end

