clc
clear
close all

draw=1;%plot
file='test';
dir=strcat('./data/',file,'.mat');
load(dir)

laser_=laser.ls;
mdist=[];
mang=[];
line_size=10;%line length
gate_mul=2;
esp=0.05;
l1_d=0.25;
l_det=0.05;

merrx=[];
merry=[];
mang=[];

global linesnew jj
linesnew={};
jj=0;
for j=1:size(laser_,1)
    ls=laser_(j,:);
    ls1=ls;
    [lx,ly,ls,gate]=Polar2Cart(ls,gate_mul);%
%     lx=lx(1,200:340);
%     ly=ly(1,200:340);
%     ls=ls(1,200:340);
%     gate=gate(1,200:340);
    if draw
        scatter(lx(1,:),ly(1,:),1,'k')
        axis([-8 8 -8 8])
        hold on
        scatter(0,0,100,'k','^')
    end
    lines=Region_Segment(ls,lx,ly,line_size,gate);
    for k=1:length(lines)
        find_corner(lines{k},esp);%break line
        if length(linesnew)==1
            for i=1:length(linesnew)
                l1=linesnew{i};
                if l1.x(1,1)
                    if abs(l1.d-l1_d)<l_det
                        l1x=l1.x(5:end-5);%删除两边各两个点
                        l1y=l1.y(5:end-5);
                        p=polyfit(l1x,l1y,1);%拟合直线
                        ang=abs(atan(p(1))*180/pi);%角度
                        xx=mean(l1x);%x中点
                        yy=(p(1)*mean(l1x)+p(2));%y中点
                        if draw
                            xx
                            yy
                            ang
                            hold on
%                             scatter(l1.x,l1.y,'r')
                            xx=-8:0.01:8;
                            yy=xx*p(1,1)+p(1,2);
                            plot(xx,yy,'r')
                            pause(0.1)
                        end
                    end
                end
            end
        end
        linesnew={};
        jj=0;
    end
    if draw
        hold off
    end
end