function [ang,ver]=angle_fit(l1,l2)
%find tow line ang and ver
p1=polyfit(l1.x,l1.y,1);
p2=polyfit(l2.x,l2.y,1);
k1=p1(1);
b1=p1(2);
k2=p2(1);
b2=p2(2);
xx=(b2-b1)/(k1-k2);
yy=k1*xx+b1;
ver(1,1)=xx;
ver(1,2)=yy;
ang1=atan(k1)*180/pi;
ang2=atan(k2)*180/pi;
ang=abs(ang1-ang2);
if ang<90
    ang=180-ang;
end

end