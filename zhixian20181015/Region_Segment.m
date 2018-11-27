function lines=Region_Segment(laser,lx,ly,line_size,gate)
laser_size=length(laser);
x=lx;
y=ly;
lines={};
id=1;
j=1;
%动态阈值分割
for i=1:laser_size-1
    point1=[x(1,i) y(1,i)];
    point2=[x(1,i+1) y(1,i+1)];
    dis=norm(point2-point1);
    %
    if (dis>gate(1,i))
        %
        if (i-id>line_size)
            lines{j}.x=x(1,id:i);
            lines{j}.y=y(1,id:i);
            p1=[x(1,id) y(1,id)];
            p2=[x(1,i-1) y(1,i)];
            lines{j}.d=norm(p1-p2);
            j=j+1;
        end
        id=i+1;
%         scatter(x(1,i),y(1,i))
%         pause(0.1)
    end
end
% for i=1:length(lines)
%     scatter(lines{i}.x,lines{i}.y)
% end
end