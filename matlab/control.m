function [ U1,U2 ] = control( x,y )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if x>0.5&&y>=-0.005*abs(x)
         U1=5; % Tension donnée au moteur 1 Pour se rapprocher de la cible si elle est en dessous 
         U2=7; % Même principe pour le moteur 2
     elseif x<-0.5&&y<=0.005*abs(x)
         U1=7; % Même principe pour tourner dans l'autre sens
         U2=5;
     else
        U1=7; % Si on est suffisament proche on applique la même tension aux deux moteurs
        U2=7;
      end

end

