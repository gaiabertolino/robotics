
% Cinematica inversa

L1 = 0;
L2 = 0.9;
L3 = 0.9;

% Primo punto: [0.8 0.8 0.5]
% Calcolo di theta1

P1 = [0.8 0.8 0.5];

theta1 = atan2(P1(1), P1(2))
theta1 = atan2(-P1(1), -P1(2))

% theta2 = atan2( (L2 + L3*cos(theta3) ) * pz – L3* sqrt(px2 + py2 ),
%                   +- (L2 + L3*cos(theta3) ) * sqrt(px2 + py2 ) + L3*sen(theta3)*pz )

