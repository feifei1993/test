function pm=Fit_Err(line)
line_x=line.x;
line_y=line.y;
p=polyfit(line_x,line_y,1);
px=line_x;
py=polyval(p,px);
pm=py-line_y;
end