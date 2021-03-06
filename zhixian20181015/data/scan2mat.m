clc
clear
close all

%f=fopen('/home/ares/zhixian20181015/data/scan.txt');
f=fopen('./laser.txt');
draw_on=1;
j=0;
ls=[];
while ~feof(f)
    line=deblank(fgetl(f));
    s=str2double(regexp(line,'\s+','split'));
    point_num=length(s);
    ls=[ls;s];
    if draw_on==1
        for i=1:point_num
            x(1,i)=s(1,i)*cos((i*4/point_num-2));
            y(1,i)=s(1,i)*sin((i*4/point_num-2));
        end
        scatter(x,y,1,'k')
        hold on
        scatter(0,0,100,'^r')
        axis([-10 10 -10 10])
        hold off
        pause(0.1)
    end
    j=j+1;
end
laser.ls=ls;
save('test.mat','laser')
