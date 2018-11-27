function find_corner(line,eps)
global linesnew jj
x=line.x;
y=line.y;
line1={};
line2={};
l0=[x(1) y(1)];
l1=[x(end) y(end)];
dis = norm(l1-l0);
lcos=(x(end)-x(1))/dis;
lsin=-(y(end)-y(1))/dis;
maxDis=0;
n=length(x);
for i=1:n
    xdis=(x(i)-x(1))*lsin;
    ydis=(y(i)-y(1))*lcos;
    dDis=abs(xdis+ydis);
    if dDis>maxDis
        maxDis=dDis;
        id=i;
    end
end
if ( maxDis<eps || n<4)
    line.d=dis;
    jj=jj+1;
    linesnew{jj}=line;
else
    line1.x=x(1,1:id);
    line1.y=y(1,1:id);
    find_corner(line1,eps);
    line2.x=x(1,id+1:end);
    line2.y=y(1,id+1:end);
    find_corner(line2,eps);
end

end