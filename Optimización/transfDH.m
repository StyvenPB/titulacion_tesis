function T = transfDH(d, th, a, alp)
%  Matriz de tranformacion homogenea
T= [cos(th), -cos(alp)*sin(th), sin(alp)*sin(th), a*cos(th);
    sin(th), cos(alp)*cos(th), -sin(alp)*cos(th), a*sin(th);
    0, sin(alp), cos(alp), d;
    0, 0, 0, 1];

end

