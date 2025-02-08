function Rn = CinematicaDiretta(theta1, theta2, theta3)
% Cinematica diretta

L1 = 0;
L2 = 0.9;
L3 = 0.9;


Rb0 =  [1 0 0 0.5; 
        0 1 0 0.5; 
        0 0 1   1; 
        0 0 0   1];

R01 = [ cos(theta1)   0    sin(theta1)  L1*cos(theta1);
        sin(theta1)   0   -cos(theta1)  L1*cos(theta1);
             0        1        0        0;
             0        0        0        1 ];
         
R12 = [ cos(theta2)    -sin(theta2)   0     L2*cos(theta2);
        sin(theta2)    cos(theta2)    0     L2*sin(theta2);
             0             0          1           0;
             0             0          0           1 ];
         
R23 = [ cos(theta3)    -sin(theta3)    0    L3*cos(theta3);
         sin(theta3)     cos(theta3)   0    L3*sin(theta3);
             0             0           1           0;
             0             0           0           1 ];



Rn = Rb0*R01*R12*R23;
end

