%% OPTIMIZACIÓN
clear; close; clc

%% Método Monte-Carlo
close;
%load('randnum.mat')
k= 2000;        %   # muestras 
thmin=[0,  0,  0, 0,   0, 0, 0]*pi/180; 
thmax=[360, 360, 360, 360, 360, 360, 360]*pi/180; 

p= wspace(thmin, thmax, k); % Posiciones alcanzables usando Monte-Carlo

for n=1:k
     plot3(p(1,n), p(2,n), p(3,n), '+r'), grid on, hold on; xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')
     disp(n)
     pause(0.01)   
end 

gfc=figure; plot3(p(1,:), p(2,:), p(3,:), '+r'), grid on, hold on; xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')

%% Espacio de trabajo sutura

d= 35; % Longitud del cubo 20 - 35 cm 
div=10;
x= -18; y=-10; z=-5;
Pdes=wspaceciru(d,div,x,y,z);

for i=1:length(Pdes)
    plot3(Pdes(1, i),Pdes(2,i), Pdes(3,i), '* b'); hold on    
end
xlim([-60 60]); ylim([-45 60]); zlim([-45 50]), grid on, title('Espacio de trabajo para cirugía (cm)');  xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')
set(gcf,'renderer','Painters')


%% Optimización con Algoritmos genéticos 

clc; 
costFun = @costodh;   % Función de costo con norma euclidiana

th= [thmin, thmax];   % Ángulos máximos y mínimos
th_init= [ ];         

% Restricciones: limites inferiores y superiores para cada eslabón
lb = [-90, -90, -90, -90, -90, -90, -90, 45, 45, 45, 45, 45, 45, 45]*pi/180;
ub = [-45, -45, -45, -45, -45, -45, -45, 90, 90, 90, 90, 90, 90, 90]*pi/180;  

A = []; b = []; Aeq = []; beq = [];

nonlcon= @restridh;

options = optimoptions('ga');
options = optimoptions(options,'ConstraintTolerance', 1e-50);  % constraint tolerance
options.InitialPopulationMatrix = th_init;   %%% [] 
% numero de población >> PopulationSize = 100 defecto

nvars = 14;
thOpt = ga(costFun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options); % pdf de ga

% thOpt= [-0.785399563565381,-0.785412468322467,-1.57079632679490,-1.32524593924836,-1.56713421741990,-0.785403961688517,-0.785398163397448,1.57079632679490,1.57079632679490,0.785424133955353,1.56994183460740,0.888376501357120,1.57079632679490,1.57079632679490]
% thOpt= [-0.785437760476003,-0.785400373485786,-1.57078858370259,-1.08277146369951,-1.51949750178202,-0.785401047523441,-1.52429873542199,1.57079337102092,1.57077859012148,0.785440617214707,1.08964502314295,0.883096611339036,1.57078814293747,1.26477588686492]

% Nueva fc
% thOpt= [-0.785408576946824,-0.785436635178063,-1.57071631831507,-0.936844326896099,-1.03406008872000,-0.785434122617666,-1.29117961508478,1.57078708361238,1.57077277261800,0.785416324596725,0.797498044906936,0.835513061235850,1.57079583120448,1.23011039447850]
% thOpt= [-0.785400232296555,-0.785401371409067,-1.57073915143270,-0.941062914892378,-1.57052859457938,-0.785413453078862,-1.33636661154474,1.57076753328889,1.57077463800628,0.785398163397448,1.03006805960775,0.888537353228037,1.57079416227443,1.22202571214606]
%% Costo inicial y final:
CFi    = costodh(th);
    disp(strcat('Initial Cost =',num2str(CFi)));

CF_Opt = costodh(thOpt);
    disp(strcat('Final Cost =',num2str(CF_Opt))); 

%% Plot del nuevo espacio de trabajo que tendrá el dispositivo maestro
popt= wspace(thOpt(1:7), thOpt(8:14), k);

% for n=1:k
%     plot3(popt(1,n), popt(2,n), popt(3,n), '+r'), grid on, hold on; xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')
%     disp(n)
%     pause(0.01)   
% end 
gfc= figure,  plot3(popt(1,:), popt(2,:), popt(3,:), '+r'), grid on, hold on; xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')


%% Optimización Fmincon

clc; 
costFun = @costodh;

th_init= [thmin, thmax];

% Restricciones: limites inferiores y superiores para cada link
lb = [-90, -90, -90, -90, -90, -90, -90, 45, 45, 45, 45, 45, 45, 45]*pi/180;
ub = [-45, -45, -45, -45, -45, -45, -45, 90, 90, 90, 90, 90, 90, 90]*pi/180;  

A = []; b = [];
Aeq = []; beq = [];

nonlcon= @restridh;

options = optimoptions('fmincon');
options = optimoptions(options,'ConstraintTolerance', 1e-50); % constraint tolerance

thOptFM = fmincon(costFun, th_init, A, b, Aeq, beq, lb, ub, nonlcon, options);
% thOptFM = [-0.785398186087641,-0.785398201240939,-1.57079631867601,-1.13746469655521,-1.23753078618080,-0.785398195884121,-1.17828191018832,1.57079631707135,1.57079619846500,0.785398335490266,1.19802941299170,0.854777060822391,1.57079631378616,1.17791258000403]

%% Costo inicial-final
CFi    = costodh(th_init);
    disp(strcat('Initial Cost =',num2str(CFi)));
CF_Opt = costodh(thOptFM);
    disp(strcat('Final Cost =',num2str(CF_Opt)));  
    
%% Plot del nuevo espacio de trabajo que tendrá el dispositivo maestro
popt= wspace(thOptFM(1:7), thOptFM(8:14), k);


for n=1:k
    plot3(popt(1,n), popt(2,n), popt(3,n), '+r'), grid on, hold on; xlabel('x (cm)'), ylabel('y (cm)'), zlabel('z (cm)')
    disp(n)
    pause(0.01)   
end 


