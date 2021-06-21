clear all
close all


% https://qintech.wordpress.com/matlab/mechanics-simulation/four-bar-linkage/

a = 23
b = 30;
c = 23;
d = 28;

Ac = 90:1:110;


for (i = 1:length(Ac) ) 
  
 
 A = sind(Ac);
 B = d / c + cosd(Ac);
 C=d/a*cosd(Ac)+(a*a+c*c+d*d-b*b)/(2*a*c);
  
 x2 = a * cosd(Aa);
 y2 = a * sind(Aa);
 x3 = d + cosd(Ac);
 y3 = sind(Ac);
  
 

  Ox(i) = 0;
  Oy(i) = 0;
  
  Ax(i) = Ox(i) + L1*cosd(theta2(i));
  Ay(i) = Oy(i) + L1*sind(theta2(i));
    
  Bx(i) = Ox(i) + Ax(i) + L2*cosd(theta3(i));
  By(i) = Oy(i) + Ay(i) + L2*sind(theta3(i));
  
  Cx(i) = 23;
  Cy(i) = 0;  
   
  plot( [Ox(i) Ax(i)], [Oy(i) Ay(i)], [Ax(i) Bx(i)], [Ay(i) By(i)], ...
        [Bx(i) Cx(i)], [By(i) Cy(i)])
     
     hold on;  
     grid on;
     %axis equal;
     %axis ([-15 15 -15 10]);
     drawnow
     
  
end








