clear;
close all;
clc;

global muro START GOAL alpha

START = [0.5;1.5];
GOAL = [16.5;7.5];

muro = [9,5.5,9,5.5];

alpha = 1/3;

o1 = [12,6,1,2]; 
o2 = [8.5,4.5,0.5,0.5]; 
o3 = [5,4,1.5,1]; 
o4 = [12,1.5,2,1];
o5 = [6.5,0.5,2.5,0.5]; 
o6 = [4.5,8.5,2.5,1.5];


% Posizione degli ostacoli
O = [rect(o1(1),o1(2),o1(3),o1(4)); 
    rect(o2(1),o2(2),o2(3),o2(4)); 
    rect(o3(1),o3(2),o3(3),o3(4)); 
    rect(o4(1),o4(2),o4(3),o4(4));
    rect(o5(1),o5(2),o5(3),o5(4)); 
    rect(o6(1),o6(2),o6(3),o6(4))];

planner = input("Scrivi \n - 1 per potenziali artificiali \n - 2 per Voronoi \n - 3 per potenziali discreti \n - 4 per grafi di visibilità \n >> ");
controller = input("Scrivi \n - 1 per errore linearizzato \n - 2 per errore non linearizzato \n - 3 per I/O linearization \n >> ");

% Dati del grafo
deltaXY = 0.2;

xm = min(START(1),GOAL(1));
xm = min(xm, min(O(:,1)));

xM = max(START(1),GOAL(1));
xM = max(xM, max(O(:,1)));

ym = min(START(2),GOAL(2));
ym = min(ym, min(O(:,2)));

yM = max(START(2),GOAL(2));
yM = max(yM, max(O(:,2)));

% Rappresentazione ostacoli
figure(1);
title('Ostacoli')
hold on
grid on
plot(START(1), START(2), 'xm', 'LineWidth',1);
plot(GOAL(1), GOAL(2), 'xr','LineWidth',1);
plot(O(:,1),O(:,2),'.'); 
axis([xm xM ym yM]);
axis('equal');

%-----------------------------------------------
% PARTE DI PATH PLANNING

% Potenziali artificiali
if planner == 1
    
    % Definizione dei potenziali attrattivo e rotanti
    % Il potenziale attrattivo sarà un paraboloide
    % Quelli repulsivi saranno degli arcotangente (dunque rotanti)
    Ja = @(x,y,Gx,Gy)((1/2)*((x-Gx).^2+(y-Gy).^2));
    Jr = @(x,y,Ox,Oy)(atan2(Oy-y,Ox-x)+pi);

    % Calcolo dei gradienti
    nablaJaX = @(x,y,Gx,Gy)(x-Gx);
    nablaJaY = @(x,y,Gx,Gy)(y-Gy);
    nablaJrX = @(x,y,Ox,Oy)((-Oy+y)./(((x-Ox).^2+(y-Oy).^2)));
    nablaJrY = @(x,y,Ox,Oy)((Ox-x)./(((x-Ox).^2+(y-Oy).^2)));

    % Regione di validità del potenziale repulsivo
    d = 1;
    r = @(x,y,Ox,Oy)((x-Ox).^2 + (y-Oy).^2 <= d^2);

    % Pesi dei potenziali
    wa = 20;
    wo = 5;

    % Rappresentazione del potenziale attrattivo

    xx = xm-2:deltaXY:xM+2;
    yy = ym-2:deltaXY:yM+2;
    [XX,YY] = meshgrid(xx,yy);

    Za = Ja(XX,YY,GOAL(1),GOAL(2));
    nablaJaXX = nablaJaX(XX,YY,GOAL(1),GOAL(2));
    nablaJaYY = nablaJaY(XX,YY,GOAL(1),GOAL(2));

    figure(2);
    subplot(121);
    hold on;
    grid on;
    surf(XX,YY,Za); % plot della funzione
    title('Potenziale attrattivo');
    subplot(122); 
    hold on;
    grid on;
    quiver(XX,YY,-nablaJaXX,-nablaJaYY); % plot dell'antigradiente

    % Rappresentazione del potenziale repulsivo

    Zr = zeros(size(Za));
    nablaJrXX = zeros(size(nablaJaXX));
    nablaJrYY = zeros(size(nablaJaYY));

    for i=1:size(O,1)
        oi = O(i,:);
        Zr = Zr + Jr(XX,YY,oi(1),oi(2)).*r(XX,YY,oi(1),oi(2));
        nablaJrXX = nablaJrXX + nablaJrX(XX,YY,oi(1),oi(2)).*r(xx,YY,oi(1),oi(2));
        nablaJrYY = nablaJrYY + nablaJrY(XX,YY,oi(1),oi(2)).*r(xx,YY,oi(1),oi(2));
    end
    
    % Normalizzazione (permette una migliore rappresentazione)
    nablaJrXXn = nablaJrXX./sqrt(nablaJrXX.^2 + nablaJrXX.^2);
    nablaJrYYn = nablaJrYY./sqrt(nablaJrYY.^2 + nablaJrYY.^2);

    figure(3);
    subplot(121);
    hold on;
    grid on;
    surf(XX,YY,Zr); % plot della funzione
    title('Potenziale repulsivo');
    subplot(122); 
    hold on;
    grid on;
    quiver(XX,YY,-nablaJrXXn,-nablaJrYYn);


    % Gradiente totale
    J = wa*Za + wo*Zr;
    nablaJx = wa*nablaJaXX + wo*nablaJrXX;
    nablaJy = wa*nablaJaYY + wo*nablaJrYY;

    % Normalizzazione (permette una migliore rappresentazione)
    nablaJxn = nablaJx./sqrt(nablaJx.^2 + nablaJy.^2);
    nablaJyn = nablaJy./sqrt(nablaJx.^2 + nablaJy.^2);

    figure(4);
    subplot(2,2,[1 3]);
    hold on;
    grid on;
    surf(XX,YY,J);
    title('Potenziale totale');
    subplot(222);
    hold on;
    grid on;
    quiver(XX, YY, -nablaJx, -nablaJy);
    subplot(224);
    hold on;
    grid on;
    quiver(XX,YY, -nablaJxn, -nablaJyn);

    % Algoritmo di ottimizzazione
    passo = 0.001; % passo non adattativo ma costante
    intorno = 0.2;
    iter = 1000; % limite iterazioni
    X = zeros(2,iter);
    X(:,1) = START;

    for k=2:iter
        Xcorr = X(:,k-1);
        nablaA = [ nablaJaX(Xcorr(1), Xcorr(2), GOAL(1), GOAL(2));
                   nablaJaY(Xcorr(1), Xcorr(2), GOAL(1), GOAL(2))]; % gradiente attrattivo
        nablaR = [0;0];
        % ciclo che calcola i potenziali repulsivi degli ostacoli
        for i=1:size(O,1)
            oi = O(i,:);
            nablaR = nablaR + [ nablaJrX(Xcorr(1),Xcorr(2),oi(1),oi(2))*r(Xcorr(1),Xcorr(2),oi(1),oi(2));
                                nablaJrY(Xcorr(1),Xcorr(2),oi(1),oi(2))*r(Xcorr(1),Xcorr(2),oi(1),oi(2))];
        end
        nablaJ = wa*nablaA + wo*nablaR;
        Xsucc = Xcorr - passo*nablaJ;
        X(:,k) = Xsucc;

        % Verifica dell'intorno
        if norm(Xsucc-GOAL) <= intorno
            break;
        end
    end

    % Riallocazione delle risorse per non sprecarle
    if k<iter
        X(:,k+1:end) = [];
    end

    % Raffigurazione totale
    figure(5)
    quiver(XX,YY, -nablaJxn, -nablaJyn);
    hold on
    grid on
    plot(START(1), START(2), 'xm', 'LineWidth',2);
    plot(GOAL(1), GOAL(2), 'xr','LineWidth',2);
    plot(O(:,1),O(:,2),'.','LineWidth',0.1);
    axis([xm xM ym yM]);
    axis('equal');
    hold on
    for i = 1:size(X,2)
        plot(X(1,i),X(2,i),'xk','Linewidth',2);
        hold on
    end
   title('Traiettoria');
    path = X';
end

% Voronoi
if planner == 2
    
  % Aggiunta del perimetro esterno
  O = [O; rect(muro(1),muro(2),muro(3),muro(4))];
    
  % Creazione grafo di Voronoi
  [vx, vy] = voronoi(O(:,1),O(:,2));
  i = 1;
  x = START(1);
  y = START(2);
  xg = GOAL(1);
  yg = GOAL(2);
  
  % Processo di eliminazione
  for k=1:length(vx)
     if cont(vx(k),vy(k))
         vx(k) = 1E100;
         vy(k) = 1E100;
     end
  end
  
% Ciclo di individuazione del cammino  
path = [x;y];
while (abs(x - xg) > 2)||(abs(y - yg) > 2)
    distVoro = sqrt((vx - x).^2+(vy - y).^2);
    distGoal = sqrt((xg - x).^2+(xg - y).^2);
    
    % Trova il vertice di Voronoi più vicino
    [mn ind] = min(distVoro(:));
    xt = vx(ind);
    yt = vy(ind);
    goalj = sqrt((xg - xt).^2+(xg - yt).^2);
    if (goalj < distGoal)
        if distVoro(ind) > 0.1
            xn = linspace(x,xt,20);
            yn = y + (yt-y)*(xn-x)/(xt-x);
            path = [path, [xn; yn]];
        else
            path = [path, [x;y]];
        end
        x = xt;
        y = yt;
    end
     vx(ind) = 1E100;
     vy(ind) = 1E100;
end

 % Aggiunta dei punti finali per giungere al goal
 xn = linspace(x,xg,50);
 yn = y + (yg-y)*(xn-x)/(xg-x);
 path = [path, [xn; yn]];

figure(2)
voronoi(O(:,1),O(:,2));
title('Grafo di Voronoi e path del robot');
hold on
plot(path(1,:),path(2,:),'-r');
hold on
plot(START(1), START(2), 'ok');
hold on
plot(GOAL(1), GOAL(2), 'ok');
hold off

path = unique([path(1,:)' path(2,:)'],'rows');
end


% Potenziali discreti
if planner == 3
    
     global discr
    
    % Fattore che determina l'ampiezza della griglia
    % sottoforma di frazione3
    
    
    % Punti della griglia
    dx = muro(1)*2/alpha;
    dy = muro(2)*2/alpha;
    
    discr = zeros(dy,dx);

    % Rappresentazione dei punti della griglia
   for i=0:dx
       for j=0:dy
           plot(i*alpha,j*alpha,'ok')
           grid on
           hold on
       end
   end
   
    
   % Assegnazione dei mumeri alla matrice

   xg = GOAL(1);
   yg = GOAL(2);
   ixg = round(xg/alpha);
   iyg = muro(2)*2/alpha-floor(yg/alpha);
   x = ixg;
   y = iyg;
   punti = [x y];
   i = 1;
   index = 0;
   visited = [x y];
    while i<dx*dy
        for j=x-index:x+index % indice lungo l'ascissa3
            if (j>0 & j<=(muro(1)*2/alpha))
                for k=y-index:y+index % indice lungo l'ordinata
                    if (k>0 & k<=(muro(2)*2/alpha))
                        %fprintf('j = %d, k = %d\n', j,k);
                        if ((j==x-index | j==x+index | k==y-index | k==y+index) & x>0 & x<=(muro(1)*2/alpha) & y>0 & y<=(muro(2)*2/alpha))

                            x1 = j*alpha;
                            y1 = ((muro(2)*2/alpha-k)*alpha);
                            x2 = j*alpha;
                            y2 = ((muro(2)*2/alpha-k+1)*alpha);
                            x3 = (j-1)*alpha; 
                            y3 = ((muro(2)*2/alpha-k)*alpha);
                            x4 = (j-1)*alpha;
                            y4 = ((muro(2)*2/alpha-k+1)*alpha);

                            if (j==ixg & k==iyg)
                                discr(k,j) = 0;
                            elseif (discr(k,j) == 0 & (cont(x1,y1) | cont(x2,y2) | cont(x3,y3) | cont(x4,y4)))
                                discr(k,j) = 500;
                                i = i+1;
                            elseif discr(k,j) == 0
                                    discr(k,j) = index;
                                i = i+1;
                            else
                            end
                        end
                    end
                end
            end
        end
    index = index+1;
    end
    
    % Algoritmo di movimento
    xs = round(START(1)/alpha);
    ys = muro(2)*2/alpha-floor(START(2)/alpha);
    val = discr(ys,xs)-1;
    ret = [xs,ys];
    salire = true;
    path = zeros(length(discr(:,1)),length(discr(1,:)));
    path(ys,xs) = 1;
    x = xs;
    y = ys;
    xg = round(GOAL(1)/alpha);
    yg = muro(2)*2/alpha-floor(GOAL(2)/alpha);
    while x ~= xg | y  ~= yg
        if (x+1<=xg & discr(y,x+1) == val-1)
            x = x+1;
            val = val-1;
            salire = true;
            path(y,x) = 1;
        elseif x+1<=xg & discr(y,x+1) == val
            x = x+1;
            salire = true;
            path(y,x) = 1;
        else
            if (salire == true & y-1<=ys & discr(y-1,x) <= val)
                if discr(y-1,x) == val
                    y = y-1;
                else
                    y = y-1;
                    val = val-1;
                end
                path(y,x) = 1;
            elseif y+1 > 0 & discr(y+1,x) <= val
                if discr(y+1,x) == val
                    y = y+1;
                else
                    y = y+1;
                    val = val-1;
                end
                salire = false;
                path(y,x) = 1;
            end
            
        end
    end
    
% Rappresentazione del path
included = [];
figure(3);
title('Traiettoria')
hold on
grid on
plot(START(1), START(2), 'xm', 'LineWidth',1);
plot(GOAL(1), GOAL(2), 'xr','LineWidth',1);
plot(O(:,1),O(:,2),'.'); 
axis([xm xM ym yM]);
axis('equal');
 for i=0:dx
       for j=0:dy
           plot(i*alpha,j*alpha,'ok')
           grid on
           hold on
       end
 end
 hold on
    for i=1:length(path(:,1)) % altezza della matrice (numero righe)
        for j=1:length(path(1,:)) % lunghezza della matrice (numero colonne)
            if path(i,j) == 1
                included = [included; j*alpha-alpha/2 (muro(2)*2/alpha-i)*alpha+alpha/2];
                plot(j*alpha-alpha/2,(muro(2)*2/alpha-i)*alpha+alpha/2,'*k')
                hold on
            end
        end
    end
    path = sort(included);
end


% Grafo di visibilità
if planner == 4 
   
  alpha = 0.2;
  v = [GOAL(1) GOAL(2);
      START(1) START(2);
      o1(1)+o1(3) o1(2)+o1(4);
      o1(1)+o1(3) o1(2)-o1(4);
      o1(1)-o1(3) o1(2)+o1(4);
      o1(1)-o1(3) o1(2)-o1(4);
      
      o2(1)+o2(3) o2(2)+o2(4);
      o2(1)+o2(3) o2(2)-o2(4);
      o2(1)-o2(3) o2(2)+o2(4);
      o2(1)-o2(3) o2(2)-o2(4);
      
      o3(1)+o3(3) o3(2)+o3(4);
      o3(1)+o3(3) o3(2)-o3(4);
      o3(1)-o3(3) o3(2)+o3(4);
      o3(1)-o3(3) o3(2)-o3(4);
      
      o4(1)+o4(3) o4(2)+o4(4);
      o4(1)+o4(3) o4(2)-o4(4);
      o4(1)-o4(3) o4(2)+o4(4);
      o4(1)-o4(3) o4(2)-o4(4);
      
      o5(1)+o5(3) o5(2)+o5(4);
      o5(1)+o5(3) o5(2)-o5(4);
      o5(1)-o5(3) o5(2)+o5(4);
      o5(1)-o5(3) o5(2)-o5(4);
      
      o6(1)+o6(3) o6(2)+o6(4);
      o6(1)+o6(3) o6(2)-o6(4);
      o6(1)-o6(3) o6(2)+o6(4);
      o6(1)-o6(3) o6(2)-o6(4)];
  
  points = [];
  full = [];
  for i=1:length(v(:,1))
       x1 = v(i,1);
       y1 = v(i,2);
      for j=1:length(v(:,1))
          x2 = v(j,1);
          y2 = v(j,2);
          noObs = true;
          if (x1 ~= x2 | y1 ~= y2)
              full = [full; x1 y1 x2 y2 sqrt((x2-x1)^2+(y2-y1)^2)];
               x = linspace(x1,x2,5000);
               y = y1 + (y2-y1)*(x-x1)/(x2-x1);
               retta = [x' y'];
               n = 1;
               while noObs == true & n <= length(x)
                   if cont(retta(n,1), retta(n,2))
                       noObs = false;
                   else
                       n = n+1;
                   end
               end
             if noObs == true
                  points = [points; x1 y1 x2 y2 sqrt((x2-x1)^2+(y2-y1)^2)];
             end
          end
       end
  end
  
  
  figure(1)
  for i=1:length(points(:,1))
      plot([points(i,1),points(i,3)],[points(i,2),points(i,4)]);
      hold on
  end
  
  % Algoritmo di scelta del cammino
    xg = GOAL(1);
    yg = GOAL(2);
    xs = START(1);
    ys = START(2);
    x = xs;
    y = ys;
    arrived = false;
    path = [];
    
    while arrived ~= true
        dist = 0;
        xn = 0;
        yn = 0;
        i = 1;
        while i<=length(points(:,1))
            if (points(i,1) == x & points(i,2) == y & points(i,5)>dist)
                dist = points(i,5);
                xn = points(i,3);
                yn = points(i,4);
                points = setdiff(points,points(i,:),'rows');
            else
                i = i+1;
            end
        end
        j = 1;
        while j<=length(points(:,1))
            if ((points(j,3) == x & points(j,4) == y) |(points(j,3) == xn...
                    & points(j,4) == yn) | (points(j,1) == x & points(j,2) == y) ...
                    | points(j,3) <= x | points(j,3) <= xn)
                points = setdiff(points,points(j,:),'rows');
            else
                j = j+1;
            end
        end
        path = [path; x y xn yn];
        if (xn == xg & yn == yg)
            arrived = true;
        end
        x = xn;
        y = yn;
        
    end
  
    figure(3)    
    plot(START(1), START(2), 'xm', 'LineWidth',1);
    plot(GOAL(1), GOAL(2), 'xr','LineWidth',1);
    plot(O(:,1),O(:,2),'.'); hold on
    grid on
    axis([xm xM ym yM]);
    axis('equal');
    hold on
  for i=1:length(path(:,1))
  plot([path(i,1),path(i,3)],[path(i,2),path(i,4)]);
  hold on
  pause(0.7)
  end
  
  included = [];
  for i=1:length(path(:,1))
      x1 = path(i,1);
      y1 = path(i,2);
      x2 = path(i,3);
      y2 = path(i,4);
      x = linspace(path(i,1),path(i,3)-0.1,10/alpha);
      y = y1 + (y2-y1)*(x-x1)/(x2-x1);
      included = [included [x;y] ];
  end
  path = included';
  
  
  
end

%-----------------------------------------------
% PARTE DI CONTROL PLANNING

misura = size(path);
theta = 0.001;
if misura(1,1) < 500
    misura(1,1) = misura(1,1)*4;
end
[q, qd, qdd, tvec, pp] = trapveltraj(path', misura(1,1));
passo = [START(1), START(2), theta];

% CONTROLLO LINEARIZZATO
if controller == 1
    
    a = 0.56;
    delta = 0.99;
    
    ex = zeros(length(q)+1,1);
    ex(1) = q(1,1) - passo(1);
    ey = zeros(length(q)+1,1);
    ey(1) = q(2,1) - passo(2);
    etheta = zeros(length(q)+1,1);
    etheta(1) = angdiff(atan2(qd(2,1),qd(1,1)),passo(3));
    
    % Ciclo di integrazione
    for i=1:length(q)
        f_robot = @(t,X) control_lin(t,X,q(:,i), qd(:,i), qdd(:,i),a,delta) ; 
        [t,stato] = ode45(f_robot,[i-1,i],[passo(i,1); passo(i,2); passo(i,3)]);
        passo(i+1,:) = stato(end,:);
        ex(i+1) = q(1,i) - stato(end,1);
        ey(i+1) = q(2,i) - stato(end,2);
        etheta(i+1) = angdiff(atan2(qd(2,i),qd(1,i)),stato(end,3));
    end
end

% CONTROLLO NON LINEARIZZATO
if controller == 2
    
    k1 = @(vs,ws)(sin(vs)+2);
    k2 = 1;
    k3 = @(vs,ws)(cos((vs)+2));
    
    ex = zeros(length(q)+1,1);
    ex(1) = q(1,1) - passo(1);
    ey = zeros(length(q)+1,1);
    ey(1) = q(2,1) - passo(2);
    etheta = zeros(length(q)+1,1);
    etheta(1) = angdiff(atan2(qd(2,1),qd(1,1)),passo(3));
        
    % Ciclo di integrazione
    for i=1:length(q)
        f_robot = @(t,X)(control_NL(t,X,q(:,i), qd(:,i), qdd(:,i),k1,k2,k3));
        [t,stato] = ode45(f_robot,[i-1,i],[passo(i,1); passo(i,2); passo(i,3)]);
        passo(i+1,:) = stato(end,:);
        ex(i+1) = q(1,i) - stato(end,1);
        ey(i+1) = q(2,i) - stato(end,2);
        etheta(i+1) = angdiff(atan2(qd(2,i),qd(1,i)),stato(end,3));
    end
end

% CONTROLLO INPUT OUTPUT LINEARIZATION
if controller == 3
    
    k1 = 1;
    k2 = 1;
    b = 0.5;
    
    ex = zeros(length(q)+1,1);
    ex(1) = q(1,1) - passo(1);
    ey = zeros(length(q)+1,1);
    ey(1) = q(2,1) - passo(2);
    etheta = zeros(length(q)+1,1);
    etheta(1) = angdiff(atan2(qd(2,1),qd(1,1)),passo(3));
        
    % Ciclo di integrazione
    for i=1:length(q)
        f_robot = @(t,X)(control_IO(t,X,q(:,i), qd(:,i),k1,k2,b));
        [t,stato] = ode45(f_robot,[i-1,i],[passo(i,1); passo(i,2); passo(i,3)]);
        passo(i+1,:) = stato(end,:);
        ex(i+1) = q(1,i) - stato(end,1);
        ey(i+1) = q(2,i) - stato(end,2);
        etheta(i+1) = angdiff(atan2(qd(2,i),qd(1,i)),stato(end,3));
    end
    
end


% Rappresentazione path
    figure(7)
    subplot(2,1,1);
    plot(START(1), START(2), 'xm'); hold on
    plot(GOAL(1), GOAL(2), 'xr'); hold on
    plot(O(:,1),O(:,2),'.'); hold on
    hold on
    grid on
    axis([xm xM ym yM]);    
    axis('equal');
    hold on
    subplot(2,1,2);
    plot(START(1), START(2), 'xm'); hold on
    plot(GOAL(1), GOAL(2), 'xr'); hold on
    plot(O(:,1),O(:,2),'.'); hold on
    grid on
    grid on
    axis([xm xM ym yM]);   
    axis('equal');
    subplot(2,1,1)
    plot(q(1,:),q(2,:),'.')  
    title('Traiettoria del path planning');
    subplot(2,1,2);
    plot(passo(:,1),passo(:,2),'.')
    title('Traiettoria del trajectory tracking');
    
    % Rappresentazione errore 
    figure(8)
    subplot(2,1,1);
    hold on
    grid on
    axis([xm xM ym yM]);    
    axis('equal');
    hold on
    subplot(2,1,2);
    grid on
    axis([xm xM ym yM]);    
    axis('equal');
    subplot(2,1,1)
    plot(ex) 
    title('Evoluzione dell''errore ex');
    subplot(2,1,2)
    plot(ey)
    title('Evoluzione dell''errore ey');