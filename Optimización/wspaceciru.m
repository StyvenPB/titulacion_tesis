function [Pdes] = wspaceciru(d,div,xx,yy,zz) % tamaño de cubo, numero de puntos por lado, coordenadas iniciales 

Pd = [0, d, d, 0, 0,  0, d, d, 0,  0,  d,  d;     %X= X+p0(1);
      0, 0, d, d, 0,  0, 0, d, d,  d,  d,  0;     %Y= Y+p0(2);
      0, 0, 0, 0, 0,  d, d, d, d,  d,  d,  d];    %Z= Z+p0(3);
Pd= Pd + [xx;yy;zz];

% Se genera el cubo 
Pdes=[];
for i=1:length(Pd)
    if i==1
        x=linspace(0+xx,d+xx,div);
        for k=1:div
            Pdes(:,k)= [x(k); Pd(2,i); Pd(3,i)];
        end      
        
    elseif i==2
        y= linspace(0+yy,d+yy,div);
        for k=1:div
            Pdes(:,k+div)= [Pd(1,i); y(k); Pd(3,i)];
        end       
        
    elseif i==3
        x=linspace(d+xx,0+xx,div);
        for k=1:div
            Pdes(:,k+2*div)= [x(k); Pd(2,i); Pd(3,i)];
        end
        
    elseif i==4
        y= linspace(d+yy,0+yy,div);
        for k=1:div
            Pdes(:,k+3*div)= [Pd(1,i); y(k); Pd(3,i)];
        end
        
    elseif i==5
        z= linspace(0+zz,d+zz, div);
        for k=1:div
            Pdes(:,k+4*div)= [Pd(1,i); Pd(2,i); z(k)];
        
        end
        
    elseif i==6
        x= linspace(0+xx,d+xx, div);
        for k=1:div
            Pdes(:, k+5*div)= [x(k); Pd(2,i); Pd(3,i)];
        end
    
    elseif i==7
        y= linspace(0+yy,d+yy,div);
        for k=1:div
            Pdes(:,k+6*div) = [Pd(1,i); y(k); Pd(3,i)];
        end
            
    elseif i==8
        x= linspace(0+xx,d+xx,div);
        for k=1:div
            Pdes(:,k+7*div) = [x(k); Pd(2,i); Pd(3,i)];
        end
        
    elseif i==9
        y=linspace(d+yy,0+yy,div);
        for k=1:div
            Pdes(:,k+8*div) = [Pd(1,i); y(k); Pd(3,i)];        
        end
        
    elseif i==10
        z= linspace(d+zz,0+zz,div);
        for k=1:div
            Pdes(:,k+9*div) = [Pd(1,i); Pd(2,i); z(k)];            
        end
    elseif i==11
        z= linspace(d+zz,0+zz,div);
        for k=1:div
            Pdes(:,k+10*div) = [Pd(1,i); Pd(2,i); z(k)];
        end        
    elseif i==12
         z= linspace(d+zz,0+zz,div);    
        for k=1:div
            Pdes(:,k+11*div)= [Pd(1,i); Pd(2,i); z(k)];
        end
    end

end


end

