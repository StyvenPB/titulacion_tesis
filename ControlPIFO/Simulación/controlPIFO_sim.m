%% PI de Orden Fraccionario Simulación 
clear all; close all; clc;

% Espacio de estados de la función de transferencia 
num= [0.0005396  0.004573]; den = [0.0344  1];
[A,B,C,D] = tf2ss(num,den);

% Parámetros del controlador y el tiempo de muestreo
Kc=35; Ki=500; cr=1.55;  
T = 0.001; ulow = 0; uhigh = 12;  % Saturando de 0 a 12V

a=0.8;  % exponente fraccionario
c=(2/T)^a; p=[1 -a a^2/3 -a/3]; q=[1  a a^2/3  a/3]; % Discretización de Tustin 
pe=Kc*conv(p,q) + Ki*c^(-1)*conv(q,q) ; 
pu=conv(p,q); 

% Considerando pe*e = pu*u => pe*u/pu(1)=pu*u/pu(1)
pu=pu/pu(1); pe=pe/pu(1); 
b2=pu(2); b3=pu(3); b4=pu(4); b5=pu(5); b6=pu(6); b7=pu(7);
a1=pe(1); a2=pe(2); a3=pe(3); a4=pe(4); a5=pe(5); a6=pe(6); a7=pe(7);

% Condiones iniciales para el sistema
x1 = 0; x2 = 0; br=1;
u_1=0.5; u_2=0; u_3=0; u_4=0; u_5=0; u_6=0;
e_1=0; e_2=0; e_3=0; e_4=0; e_5=0; e_6=0; u=0;

Mm = 1000;              % Iteraciones
R=[]; U=[]; Y=[];       %  Variables para almacernar datos 

for k = 1:Mm
    r = 0.15; R(k) = r; % Referencia
    e = cr*r - x1;      % Error
    % Ley de control de PIFO
    v=-b2*u_1-b3*u_2-b4*u_3-b5*u_4-b6*u_5-b7*u_6 + ...
    a1*e+a2*e_1+a3*e_2+a4*e_3+a5*e_4+a6*e_5+a7*e_6; U(k)=u;
    
    if(v < ulow), u = ulow; elseif( v > uhigh), u = uhigh; else u = v; 
    end
    U(k) = u;

    % Discretización del modelo
    x1 = x1 + T*(A*x1+B*u);  
    Y(k)=x1;
    
    % Actualizaciones
    u_6=u_5; u_5=u_4; u_4=u_3; u_3=u_2; u_2=u_1; u_1=u; 
    e_6=e_5; e_5=e_4; e_4=e_3; e_3=e_2; e_2=e_1; e_1=e;
end

% Gráficos
ejex = linspace(0,Mm*T,Mm); 
figure, subplot(2,1,1),plot(ejex,R,'r', 'LineWidth', 2); hold on; plot(ejex,Y,'b','LineWidth', 2); grid; ylabel('Torque de Motor (Nm)'), ylim([ 0 1.2*max(R)]), legend('Refencia','Señal controlada')
subplot(2,1,2); plot(ejex,U, 'r',  'LineWidth', 2); grid; ylabel('Esfuerzo de control (V)')
xlabel('Tiempo   [s]'); 