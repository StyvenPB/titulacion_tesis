function [c, ceq] = restridh(x)
    % Links y ángulos  
    thmin1= x(1); thmin2= x(2); thmin3=x(3);  thmin4= x(4); thmin5=x(5); thmin6=x(6); thmin7=x(7); thmax1=x(8);
    thmax2=x(9); thmax3=x(10); thmax4=x(11); thmax5=x(12); thmax6=x(13); thmax7=x(14); 
    
    % Valor fijo (No optimizable)
    % Re
    c=[ ];
    ceq = [];
    %ceq=[ thmin7-(60*pi/180);  thmax7-(120*pi/180)];
end