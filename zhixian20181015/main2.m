clc
clear
close all

draw=0;%plot
file='1_1.8';
s=regexp(file,'_', 'split');
w=str2double(s{1})*60;
h=str2double(s{2})*100;
dir=strcat('./data/',file,'.mat');
load(dir)

laser_=laser.ls;
mdist=[];
mang=[];
line_size=10;%line length
gate_mul=4;
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
                if abs(l1.d-l1_d)<l_det
                        l1x=l1.x(3:end-2);
                        l1y=l1.y(3:end-2);
                        p=polyfit(l1x,l1y,1);
                        ang=abs(atan(p(1))*180/pi);
                        xx=mean(l1x)*100;
                        yy=(p(1)*mean(l1x)+p(2))*100;
                        err_x=abs(xx-w);
                        err_y=abs(yy-h);
                        if err_x<5
                            if draw
                                scatter(l1.x,l1.y,1,'r')
                            end
                            merrx=[merrx err_x];
                            merry=[merry err_y];
                            mang=[mang ang];
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
mean(merrx)
mean(merry)
mean(mang)
