function Xdot = control_NL(t,X,q,qd,qdd,k1,k2,k3)

% Situazione reale
x_sit = X(1);
y_sit = X(2);
theta_sit = X(3);

% Coordinate
x_star = q(1);
y_star = q(2);

% Velocità
xd_star = qd(1);
yd_star = qd(2);
theta_star = atan2(yd_star, xd_star);

% Accelerazione
xdd_star = qdd(1);
ydd_star = qdd(2);

% Legge di controllo
v_star = sqrt(xd_star^2 + yd_star^2);
if(v_star<0.001)
    v_star=0.001; 
end

w_star = (ydd_star * xd_star - yd_star * xdd_star)/(v_star^2);

% Errore
ex = cos(theta_sit)*(x_star-x_sit) + sin(theta_sit)*(y_star-y_sit);
ey = -sin(theta_sit)*(x_star-x_sit) + cos(theta_sit)*(y_star-y_sit);
etheta = angdiff(theta_star,theta_sit); 

u1 = -k1(v_star, w_star)*ex;
u2 = -k2*v_star*(sin(etheta)/etheta)*ey - k3(v_star,w_star)*etheta;

v = v_star * cos(etheta) - u1;
w = w_star - u2;

Xdot = [v*cos(theta_sit);
        v*sin(theta_sit);
        w
        ];
end

