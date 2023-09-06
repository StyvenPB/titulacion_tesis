function [p] = wspace(thmin, thmax, iter)
load('randnum.mat', 'randnum')

for n=1:iter

    % th = thmin + (thmax-thmin)*rand;      
    th(1, n)= thmin(1)+ (thmax(1)-thmin(1))*randnum(1,n);
    th(2, n)= thmin(2)+ (thmax(2)-thmin(2))*randnum(2,n);
    th(3, n)= thmin(3)+ (thmax(3)-thmin(3))*randnum(3,n);
    th(4, n)= thmin(4)+ (thmax(4)-thmin(4))*randnum(4,n);
    th(5, n)= thmin(5)+ (thmax(5)-thmin(5))*randnum(5,n);
    th(6, n)= thmin(6)+ (thmax(6)-thmin(6))*randnum(6,n);
    th(7, n)= thmin(7)+ (thmax(7)-thmin(7))*randnum(7,n);
    
    % cinemática directa
    TT= cdir(th(:,n));

    % Guardando las posiciones
    p(:,n)= TT(1:3, 4);

end

