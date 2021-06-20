clear all
close all

%https://www.youtube.com/watch?v=4O-XPJ7flLU

L0 = 7;
L1 = 3;
L2 = 7.5;
L3 = 4.5;

L_PA = 4;
alpha = 35;

w2 = 5;
theta2 = 0:2:360;


for (i = 1:length(theta2) ) 
  
  AC(i) = sqrt(L0^2 + L1^2 - 2*L0*L1*cosd(theta2(i)));
  beta(i) = acosd((L0^2 + AC(i)^2 - L1^2) / (2*L2*AC(i)));
  psi(i) = acosd((L2^2 + AC(i)^2 - L3^2) / (2*L2*AC(i)));
  lamda(i) = acosd((L3^2 + AC(i)^2 - L2^2) / (2*L3*AC(i)));
  
  theta3(i) = psi(i) - beta(i);
  theta4(i) = 180 - lamda(i) - beta(i);
  
  if theta2(i) > 180
    theta3(i) = psi(i) + beta(i);
    theta4(i) = 180 - lamda(i) + beta(i);
  endif
  
  %Define joint positions
  Ox(i) = 0;
  Oy(i) = 0;
  
  Ax(i) = Ox(i) + L1*cosd(theta2(i));
  Ay(i) = Oy(i) + L1*sind(theta2(i));
    
  Bx(i) = Ox(i) + Ax(i) + L2*cosd(theta3(i));
  By(i) = Oy(i) + Ay(i) + L2*sind(theta3(i));
  
  Cx(i) = L0;
  Cy(i) = 0;
  
  Px(i) = Ax(i) + L_PA*cos(alpha + theta3(i));
  Py(i) = Ay(i) + L_PA*sin(alpha + theta3(i));
  
  plot( [Ox(i) Ax(i)], [Oy(i) Ay(i)], [Ax(i) Bx(i)], [Ay(i) By(i)], ...
        [Bx(i) Cx(i)], [By(i) Cy(i)], 'LineWidth', 3)
     
     hold on;  
     grid on;
     axis equal;
     axis ([-5 15 -5 10]);
     drawnow
     
  
end








