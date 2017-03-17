clear all
close all
clc

% img=imread('C:\Users\Zacharie\Pictures\Saved Pictures\Samurai_Champloo_Wallpaper.jpg');
% imshow(img);
% length(img(1,:,:))
% length(img(:,1,:))

%imshow(img);
% Modélisation du robot sur l'eau
E=2; % Erreur pour considérer le point atteint

P2=[300;10]; %Point 2
P3=[280;150]; % Point 3
% P4=[240;50]; %Point 4
% P5=[240;160]; %Point 5



v2x=[0]; %Accélération initiale en x (ms-²)
v2y=[0]; %Accélération initiale en y (ms-²)
w2=[0]; %Accélération angulaire initiale(rads-²)
vy=[0]; %(Vitesse initiale en y (m/s)
vx=[0]; %(Vitesse initiale en x (m/s)
w=[0]; %(Vitesse angulaire initiale (rad/s)
theta=[180]; % Cap initial
x=[250]; %Position initiale en x
y=[50]; % Position initiale en y
P1=[x(1);y(1)]; %Point 1
Tx=[x(1),50]; %Coordonées en x de la droite cible (en m)
Ty=[y(1),2];% Coordonées en y de la droite cible (en m)
e=0; %Erreur initiale (mise à 0)
m=[0;0]; %Va12riable de déclaration de m  
X=[];
Y=[];
V=[];
W=[];
Theta=[];
i=0;
[x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P2,E,x(1),y(1),theta(1),vx(1),vy(1),w(1),v2x(1),v2y(1),w2(1),e);
X=[X x];
Y=[Y y];
V=[V (sqrt(vx.^2+vy.^2))];
Theta=[Theta theta];
W=[W w];
i=i+If;
[x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P3,E,x(If),y(If),theta(If),vx(If),vy(If),w(If),v2x(If),v2y(If),w2(If),e);
X=[X x];
Y=[Y y];
V=[V (sqrt(vx.^2+vy.^2))];
Theta=[Theta theta];
W=[W w];
i=i+If;
[x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P1,E,x(If),y(If),theta(If),vx(If),vy(If),w(If),v2x(If),v2y(If),w2(If),e);
X=[X x];
Y=[Y y];
V=[V (sqrt(vx.^2+vy.^2))];
Theta=[Theta theta];
W=[W w];
i=i+If;
% [x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P4,E,x(If),y(If),theta(If),vx(If),vy(If),w(If),v2x(If),v2y(If),w2(If),e);
% X=[X x];
% Y=[Y y];
% V=[V (sqrt(vx.^2+vy.^2))];
% Theta=[Theta theta];
% W=[W w];
% i=i+If;
% [x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P5,E,x(If),y(If),theta(If),vx(If),vy(If),w(If),v2x(If),v2y(If),w2(If),e);
% X=[X x];
% Y=[Y y];
% V=[V (sqrt(vx.^2+vy.^2))];
% Theta=[Theta theta];
% W=[W w];
% i=i+If;
Theta=Theta*180/pi;

t=1:i;
figure
plot(t,Theta);
title('theta(t)')
figure
plot(t,V);
title('V(t)')

figure
img2=imread('C:\Users\Zacharie\Documents\MATLAB\Modélisation Guerlédan\Guerledan.png');
img=imresize(img2,2);
imshow(img);
title('Trajectoire')
hold on
plot(X,length(img(:,1,:))-Y);




plot(x(1),length(img(:,1,:))-y(1),'+r')


% for j=2:i
%     if W(j)~=W(j-1)
%     
%     plot(X(j),Y(j),'+b') % Affichage des points de chgt de vitesse angulaire (Croix bleue)
%     
%     end
%     if V(j)~=V(j-1)
%         plot(X(j),Y(j),'or') % Affichage des points de chgt de vitesse ( rond rouge)
%     end
% end
plot(P1(1),length(img(:,1,:))-P1(2),'+black');
plot(P2(1),length(img(:,1,:))-P2(2),'+black');
plot(P3(1),length(img(:,1,:))-P3(2),'+black');
% plot(P4(1),length(img(:,1,:))-P4(2),'+black');
% plot(P5(1),length(img(:,1,:))-P5(2),'+black');
 T1x=[P1(1);P2(1)];
T2x=[P2(1);P3(1)];
T3x=[P3(1);P1(1)];
T1y=[length(img(:,1,:))-P1(2);length(img(:,1,:))-P2(2)];
T2y=[length(img(:,1,:))-P2(2);length(img(:,1,:))-P3(2)];
T3y=[length(img(:,1,:))-P3(2);length(img(:,1,:))-P1(2)];
% T4x=[P4(1);P1(1)];
% T4y=[length(img(:,1,:))-P4(2);length(img(:,1,:))-P1(2)];
% T5x=[P4(1);P5(1)];
% T5y=[length(img(:,1,:))-P4(2);length(img(:,1,:))-P5(2)];

plot(T1x,T1y);
plot(T2x,T2y);
plot(T3x,T3y);
% plot(T4x,T4y);
% plot(T5x,T5y);


