clc
clear
close all
draw=1;

file='2_1.8';
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
l_det=0.1;

merrx=[];
merry=[];
mang=[];

global linesnew jj
linesnew={};
jj=0;
j=12;
    ls=laser_(j,:);
    ls1=ls;
    [lx,ly,ls,gate]=Polar2Cart(ls,gate_mul);%
    scatter(lx(1,:),ly(1,:),1,'k')
    axis([-8 8 -8 8])
    hold on
    scatter(0,0,100,'k','^')
    lines=Region_Segment(ls,lx,ly,line_size,gate);
    
%     for i=1:length(lines)
%         scatter(lines{i}.x,lines{i}.y)
%         hold on
%         pause(0.1)
%     end
    for k=1:length(lines)
        k=6
        find_corner(lines{k},esp);%break line
        for i=1:length(linesnew)
            scatter(linesnew{i}.x,linesnew{i}.y)
        end
        pause(0.1)
        if length(linesnew)==1
            for i=1:length(linesnew)
                l1=linesnew{i};
                if abs(l1.d-l1_d)<l_det
                        xx=100*(l1.x(1,1)+l1.x(1,end))/2;
                        yy=100*(l1.y(1,1)+l1.y(1,end))/2;
                        delty=l1.y(1,1)-l1.y(1,end);
                        deltx=l1.x(1,1)-l1.x(1,end);
                        ang=abs(atan2(delty,deltx)*180/pi);
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
mean(merrx)
mean(merry)
mean(mang)