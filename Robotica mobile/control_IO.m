function Xdot = control_IO(t,X,q,qd, k1,k2,b)

% Situazione reale
x_sit = X(1);
y_sit = X(2);
theta_sit = X(3);

% Punto B

xB = x_sit+b*cos(theta_sit);
yB = y_sit+b*sin(theta_sit);

% Coordinate
x_star = q(1);
y_star = q(2);

% Velocità
xBd_star = qd(1);
yBd_star = qd(2);
Inv = [ cos(theta_sit) sin(theta_sit)
        -sin(theta_sit)/b cos(theta_sit)/b ];

% Legge di controllo
u1 = xBd_star + k1*(x_star-xB);
u2 = yBd_star + k2*(y_star-yB);
vw = Inv*[u1;u2];
v = vw(1);
w = vw(2);

Xdot = [v*cos(theta_sit);
        v*sin(theta_sit);
        w
        ];
end

