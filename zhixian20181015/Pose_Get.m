function [dist,ang]=Pose_Get(line)
linex=line.x;
liney=line.y;
linex_new=[];
liney_new=[];
j=1;
for i=1:length(linex)-1
    p1=[linex(1,i) liney(1,i)];
    p2=[linex(1,i+1) liney(1,i+1)];
    dist=norm(p1-p2);
    if dist<2
       linex_new(1,j)=linex(1,i+1);
       liney_new(1,j)=liney(1,i+1);
       j=j+1;
    end
end
cx=linex(1,round(length(linex)/2));%mid point
p=polyfit(linex_new,liney_new,1);
% py=polyval(p,px);
cy=polyval(p,cx);
% scatter(cx,cy)
k=p(1);
ang=atan(k)*180/pi;
dist=sqrt(cx^2+cy^2)*100;
end