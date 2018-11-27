function [x,y,polar,gate]=Polar2Cart(polar,mul)
laser_size=length(polar);
x=zeros(1,laser_size);
y=x;
gate=x(1,1:end-1);
for i=1:laser_size
    x(1,i)=polar(1,i)*cos(i*4/laser_size-2+pi/2);
    y(1,i)=polar(1,i)*sin(i*4/laser_size-2+pi/2);
    d=norm([x(1,i),y(1,i)]-[0,0]);
    gate(1,i)=mul*d*sin(pi/180);
end
k=(polar==inf);
x(k)=[];
y(k)=[];
polar(k)=[];
gate(k)=[];