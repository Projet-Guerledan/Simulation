function [x,y,theta,vx,vy,w,v2x,v2y,w2,e,If]=CheckPoint(P1,E,x1,y1,theta1,vx1,vy1,w1,v21x,v21y,w21,e)

i=1;
h=0.6; % Hauteur du drone (en m)
l=1; % Longueur du drone (m)
L=0.4; % Largeur du drone (m)
M=1.200; % Masse du drone  (kg)
d=10; % Diam�tre de l'h�lice (pouces)
Ph=6;  % Pas de l'h�lice (pouces)
kv=840; % Kv des moteurs brushless (Normalement identiques)
c=0.2; %Distance des flotteurs au centre de gravit� selon y. 
P=0.02835*Ph*(d^3)*(kv^2)*(10^(-10)); % Force de Pouss�e selon l'�quation d'Abbott (Sans la tension qui donne la vitesse de rotation avec Vr=UKv)
Jz=(M*(L^2+l^2))/3; % Moment d'inertie du drone semon l'axe z (celui qui nous int�resse pour la rotation du drone
dt=0.5; % Pas de temps (en s)


v2x=[v21x]; %Acc�l�ration initiale en x (ms-�)
v2y=[v21y]; %Acc�l�ration initiale en y (ms-�)
w2=[w21]; %Acc�l�ration angulaire initiale(rads-�)
vy=[vy1]; %(Vitesse initiale en y (m/s)
vx=[vx1]; %(Vitesse initiale en x (m/s)
w=[w1]; %(Vitesse angulaire initiale (rad/s)
theta=[theta1]; % Cap initial
x=[x1]; %Position initiale en x
y=[y1]; %Position initiale en y
m=[0;0];
Tx=[x(1),P1(1)]; %Coordon�es en x de la droite cible (en m)
Ty=[y(1),P1(2)];% Coordon�es en y de la droite cible (en m)

hull=[x(1);y(1);1]; % Position du bateau (Point noir)
img2=imread('C:\Users\Zacharie\Documents\MATLAB\Mod�lisation Guerl�dan\Guerledan.png');
img=imresize(img2,2);
 while norm(m-P1)>E
         i=i+1;
         m=[x(i-1);y(i-1)]; % Position du drone
        a=[x(1);y(1)]; %Point initial de la droite cible (point de d�part du drone)
        b=P1; % Point final de la droite cible
        e1=e; %Mise en m�moir de l'erreur pr�c�dente
        e=det([(b-a)/norm(b-a),m-a]); % Ecart du drone � la cible
        de=e-e1; %Diff�rence de l'erreur et de l'erreur pr�c�dente (Pour d�terminer si le drone se rapproche de la cible)
        
     

    [u1,u2]=control(e,de);
    w2(i)=c*P*((u1^2)-(u2^2))/Jz;  % Calcul de l'accel angulaire au temps i avec le th des moments
    w(i)=(w2(i)-w2(i-1))*dt+w(i-1); % Calcul de la vitesse angulaire avec Euler
    theta(i)=(theta(i-1)+w(i)*dt); % Calcul de l'angle avec Euler
    v2x(i)=cos(theta(i))*P*((u1^2)+(u2^2))/M; % Calcul de l'accel en x au tps i avec le PFD
    v2y(i)=sin(theta(i))*P*((u1^2)+(u2^2))/M; % Calcul de l'accel en x au tps i avec le PFD
    vx(i)=(v2x(i)-v2x(i-1))*dt+vx(i-1);% Calcul de la vitesse en x avec Euler
    vy(i)=(v2y(i)-v2y(i-1))*dt+vy(i-1);% Calcul de la vitesse en y avec Euler
    x(i)=x(i-1)+vx(i)*dt; % Calcul de la position en x avec Euler
    y(i)=y(i-1)+vy(i)*dt; % Calcul de la position en y avec Euler
    clf; axis([0,length(img(1,:,:)),0,length(img(:,1,:))]);
    
    hold on % Trac� Du rep�re sur lequel on va afficher le mvt du drone
    imshow(img);
    hold on
   
    hull=[x(i);length(img(:,1,:))-y(i);length(img(:,1,:))-theta(i)]; %Rafraichissement de la Position

     plot(Tx,length(img(:,1,:))-Ty); %Trac� de la droite cible


    plot(hull(1),hull(2),'oblack'); % Affichage de la position
    draw_tank(hull,'blue')
    plot(x,length(img(:,1,:))-y);
    drawnow()
 
    %pause(0.01); % Temps de pause de 1 centi�me de s pour la visibilit�
    
     end   


If=i;
 plot(Tx,Ty); %Trac� de la droite cible




end