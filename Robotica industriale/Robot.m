clear;
close all;
clc;
global L1 L2 L3 P;

% Dichiarazione della tabella di D-H 
% Lunghezza dei bracci
L1 = 0;
L2 = 0.9;
L3 = 0.9;

% Offeset del braccio
d1 = 0;
d2 = 0;
d3 = 0;

% Tipologia di giunto:
% 0 se rotoidale
% 1 se prismatico
sigma1 = 0;
sigma2 = 0;
sigma3 = 0;

% Angolo di twist
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;

% Definizione dei link 
link1 = Link([0,d1,L1,alpha1,sigma1], 'standard');
link2 = Link([0,d2,L2,alpha2,sigma2], 'standard');
link3 = Link([0,d3,L3,alpha3,sigma3], 'standard');

% Fusione in un'unica struttura dei link
rob = SerialLink([link1 link2 link3], 'name', 'puma 560');

% % Condizione iniziale
Q0 = [0.01; 0.01; 0.01];

% Punti sul cammino da attraversare
P1 = [0.8; 0.8; 0.5];
P2 = [1.2; 0.8; 0.5];
P3 = [1.0; 1.2; 0.5];

% Tempo totale di traiettoria
total = 30;

% TRAIETTORIA TRIANGOLARE

% Primo tratto da P1 a P2

T1 = 0;
T2 = 15;
ts = [T1:T2];
sigma = (ts-T1)/(T2-T1);
lambda1 = poly3(sigma);
lambda1d = poly3d(sigma);
N1 = length(lambda1);

% Ciclo di approssimazione
for i = 1:N1
    P = P1 + lambda1(i)*(P2-P1);
    Q = fminsearch('errore_3link',Q0);
    QQ(i,:) = Q; 
    PP(i,:) = P; 
    
    J = jacobiano(Q);
    Pd = (P2-P1) * lambda1d(i)/(T2-T1);
    Qd = inv(J)*Pd;
    QQd(i,:) = Qd;
    Q0 = Q;
end

% Secondo tratto da P2 a P3
T3 = 25;
ts = [T2:T3];
sigma = (ts-T2)/(T3-T2);
lambda2 = poly3(sigma);
lambda2d = poly3d(sigma);
N2 = length(lambda2);

% Ciclo di approssimazione
for i = 1:N2
    P = P2 + lambda2(i)*(P3-P2);
    Q = fminsearch('errore_3link',Q0); 
    QQ(i+N1,:) = Q; 
    PP(i+N1,:) = P; 
    
    J = jacobiano(Q);
    Pd = (P3-P2) * lambda2d(i)/(T3-T2);
    Qd = inv(J)*Pd;
    QQd(i+N1,:) = Qd;
    Q0 = Q;
end

% Terzo tratto da P3 a P1
T4 = total;
ts = [T3:T4];
sigma = (ts-T3)/(T4-T3);
lambda3 = poly3(sigma);
lambda3d = poly3d(sigma);
N3 = length(lambda3);

% Ciclo di approssimazione
for i = 1:N3
    P = P3 + lambda3(i)*(P1-P3);
    Q = fminsearch('errore_3link',Q0); 
    QQ(i+N1+N2,:) = Q; 
    PP(i+N1+N2,:) = P; 
    
    J = jacobiano(Q);
    Pd = (P1-P3) * lambda3d(i)/(T1-T3);
    Qd = inv(J)*Pd;
    QQd(i+N1+N2,:) = Qd;
    Q0 = Q;
end


% Rappresentazione del robot in 3D
figure(1)
rob.plot([0 0 0]);
title('Rappresentazione 3d del robot antropomorfo')

% Rappresentazione animata del movimento
figure(2)
plot3(P1(1), P1(2), P1(3), 'o');
hold on
plot3(P2(1), P2(2), P2(3), 'o');
hold on
plot3(P3(1), P3(2), P3(3), 'o');
hold on
for i = 1:N1
plot3(PP(i,1),PP(i,2),PP(i,3),'*')
title('Animazione dell''andamento dell''end effector');
grid on
pause(total/length(PP));
hold on
drawnow
end
hold on
for i = N1+1:N1+N2
plot3(PP(i,1),PP(i,2),PP(i,3),'*')
grid on
pause(total/length(PP));
hold on
drawnow
end
hold on
for i = N1+N2+1:N1+N2+N3
plot3(PP(i,1),PP(i,2),PP(i,3),'*')
grid on
pause(total/length(PP));
hold on
drawnow
end

passo = total/(length(QQ)+1);
t = [1:passo:total];

% Grafico della variazione dei valori di Q in base a lambda
figure(3)
subplot(3,1,1);
plot(t,QQ(:,1)*180/pi, 'marker','o', 'color','red')
title('Variazione di theta1 (angolo di giunto) nel tempo - TRIANGOLO')
grid

subplot(3,1,2);
plot(t,QQ(:,2)*180/pi,'marker','o', 'color','blue')
title('Variazione di theta2 (angolo di giunto) nel tempo - TRIANGOLO')
grid

subplot(3,1,3);
plot(t,QQ(:,3)*180/pi,'marker','o', 'color','yellow')
title('Variazione di theta3 (angolo di giunto) nel tempo - TRIANGOLO')
grid

figure(4)
plot(t,QQ(:,1)*180/pi, 'marker','o', 'color','red')
hold on
plot(t,QQ(:,2)*180/pi,'marker','o', 'color','blue')
hold on
plot(t,QQ(:,3)*180/pi,'marker','o', 'color','yellow')
grid
title('Sovrapposizione dei grafici dei giunti - TRIANGOLO')
hold off

% Grafici velocità
figure(5)
subplot(3,1,1);
plot(t,QQd(:,1), 'marker','o', 'color','red')
title('Varizione della velocità del primo giunto - TRIANGOLO')
grid

subplot(3,1,2);
plot(t,QQd(:,2),'marker','o', 'color','blue')
title('Varizione della velocità del secondo giunto - TRIANGOLO')
grid

subplot(3,1,3);
plot(t,QQd(:,3),'marker','o', 'color','yellow')
title('Varizione della velocità del terzo giunto - TRIANGOLO')
grid

% Sovrapposizione velocità dei giunti
figure(6)
plot(t,QQd(:,1)*180/pi, 'marker','o', 'color','red')
hold on
plot(t,QQd(:,2)*180/pi,'marker','o', 'color','blue')
hold on
plot(t,QQd(:,3)*180/pi,'marker','o', 'color','yellow')
grid
title('Sovrapposizione dei grafici delle velocità dei giunti - TRIANGOLO')
hold off


% TRAIETTORIA CIRCOLARE

C = [1, 0.95];
R = 0.25;
ts = [0:T4];
sigma = ts/T4;
lambda = poly3(sigma);
lambdad = poly3d(sigma);
N1 = length(lambda);
diff = (90 + asind(0.2/0.25))*pi/180;

for i = 1:N1
    theta = lambda(i)*2*pi;
    P = [R*cos(theta-diff) + C(1); R*sin(theta-diff) + C(2); 0.5];
    Q = fminsearch('errore_3link', Q0);
    Pc(i,:) = P;
    Qc(i,:) = Q;

    J = jacobiano(Q);
    Pd = [-R*sin(theta-diff); R*cos(theta-diff); 0];
    Qd = inv(J)*Pd;
    Qcd(i,:) = Qd;
end

% Rappresentazione della traiettoria circolare

figure(7)
plot3(P1(1), P1(2), P1(3), 'o');
hold on
plot3(P2(1), P2(2), P2(3), 'o');
hold on
plot3(P3(1), P3(2), P3(3), 'o');
hold on
for i = 1:length(Pc)
plot3(Pc(i,1),Pc(i,2),0.5,'*')
grid on
pause(total/(length(Pc)));
hold on
drawnow
title('Animazione della posizione dell''end effector')
end

passo = total/(length(Qc)+1);
t = [1:passo:total];

% Grafico della variazione dei valori di Q in base a lambda
figure(8)
subplot(3,1,1);
plot(t,Qc(:,1)*180/pi, 'marker','o', 'color','red')
title('Variazione di theta1 (angolo di giunto) nel tempo - CIRCONFERENZA')
grid

subplot(3,1,2);
plot(t,Qc(:,2)*180/pi,'marker','o', 'color','blue')
title('Variazione di theta2 (angolo di giunto) nel tempo - CIRCONFERENZA')
grid

subplot(3,1,3);
plot(t,Qc(:,3)*180/pi,'marker','o', 'color','yellow')
title('Variazione di theta3 (angolo di giunto) nel tempo - CIRCONFERENZA')
grid

figure(9)
plot(t,Qc(:,1)*180/pi, 'marker','o', 'color','red')
hold on
plot(t,Qc(:,2)*180/pi,'marker','o', 'color','blue')
hold on
plot(t,Qc(:,3)*180/pi,'marker','o', 'color','yellow')
grid
title('Sovrapposizione dei grafici dei giunti - CIRCONFERENZA')
hold off


% Grafici velocità
figure(10)
subplot(3,1,1);
plot(t,Qcd(:,1), 'marker','o', 'color','red')
title('Varizione della velocità del primo giunto - CIRCONFERENZA')
grid

subplot(3,1,2);
plot(t,Qcd(:,2),'marker','o', 'color','blue')
title('Varizione della velocità del secondo giunto - CIRCONFERENZA')
grid

subplot(3,1,3);
plot(t,Qcd(:,3),'marker','o', 'color','yellow')
title('Varizione della velocità del terzo giunto - CIRCONFERENZA')
grid

% Sovrapposizione grafici velocità
figure(11)
plot(t,Qcd(:,1), 'marker','o', 'color','red')
title('Variazione della velocità dei giunti - CIRCONFERENZA')
grid
hold on
plot(t,Qcd(:,2), 'marker','o', 'color','blue')
hold on
plot(t,Qcd(:,3), 'marker','o', 'color','yellow')
hold off









