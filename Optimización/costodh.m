function CF = costodh(th)
thmin= th(1, 1:7);
thmax= th(1, 8:14);

d= 35;   % Longitud del cubo 20cm // 35 cm
%p0= [-18, -12, -20]; % Coordenada inicial del espacio de trabajo
div=10;

x= -18; y=-5; z=-20;    % Coordenada inicial

Pdes=wspaceciru(d,div,x,y,z);

k=2000;

p= wspace(thmin, thmax, k);

CF=0;
for i=1:k
    for m=1:length(Pdes)
        xx(1,m) = Pdes(1,m)-p(1,k);    
        yy(1,m) = Pdes(2,m)-p(2,k);
        zz(1,m) = Pdes(3,m)-p(3,k);        
    end
    % concatena error
    e_eucli(k,:)= [min(xx), min(yy), min(zz)];
end

% norma euclidiana para error
CF=norm(e_eucli,2);
end

