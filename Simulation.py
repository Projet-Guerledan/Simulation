import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as img
import math
import random
import keyboard as kb
import sys




h=0.6
l=1
L=0.4
M=1.2
d=10
Ph=6
kv=840
c=0.2
p1=0.02835*Ph*(d**3)*(kv**2)*(10**(-10))
Jz=M*((L**2)+(l**2))/3
dt=0.5





#Calcul de l'accel angulaire
def w2(u1,u2):
	A=c*p1*((u1**2)-(u2**2))/Jz
	return A

#Calcul de la vitesse angulaire
def w(a,b,c,dt):
	A=(a-b)*dt+c
	return A

#Calcul de l'angle
def theta(a,b,dt):
	A=a*dt+b
	return A

#Calcul de l'accel en x
def v2x(O,u1,u2):
	A=math.cos(O)*p1*((u1**2)+(u2**2))/M
	return A

#Calcul de l'accel en y
def v2y(O,u1,u2):
	A=math.sin(O)*p1*((u1**2)+(u2**2))/M
	return A

#Calcul de la vitesse (valable pour les composantes en x ou en y)
def v(x1,x2,c,dt):
	A=(x1-x2)*dt+c
	return A

#Calcul de la position (valable pour les composantes en x ou en y)
def xy(a,b,dt):
	A=a*dt+b
	return A
#Calcul de l'erreur entre le drone et la droite cible
def e(a,b,m):
	n=Norm(sub(b,a))
	A=np.linalg.det(np.matrix([div(sub(b,a),n),sub(m,a)]))
	return A

#Methode pour soustraire 2 vecteurs
def sub(a,b):
	A=[b[0]-a[0],b[1]-a[1]]
	return A

#Norme d'un vecteur
def Norm(a):
	A=math.sqrt(a[0]**2+a[1]**2)
	return A

#division d'un vecteur par un scalaire
def div(a,b):
	A=[a[0]/b,a[1]/b]
	return A

#Algorithme de controle "GangBang"( Vla le nom quoi XD) (Sujet a amelioration : ie ajout d'un PID)
#Renvoie U=[u1,u2]

def control(e,de):
	U=[]
	if e>0.5 and de>=-0.005*math.sqrt(e**2):
		u1=4
		u2=6
		U.append(u1)
		U.append(u2)
	elif e<-0.5 and de<=0.005*math.sqrt(e**2):
		u1=6
		u2=4
		U.append(u1)
		U.append(u2)
	else:
		u1=6
		u2=6
		U.append(u1)
		U.append(u2)
	return U


#Algorithme pour definir un point cible
#En fonction du point initial (ie position du drone),de l'erreur initiale, du cap initial du robot, des vitesses scalaires/angulaires, et des accel scalaires/angulaires
#Renvoie A=[point final, erreur fin, cap fin, Vx fin, Vy fin, w fin, accel x fin, accel y fin, accel ang fin, Listes des x (pour tracer), Listes des y (pour tracer)]


def checkPoint(a,c,e2,theta1,vx1,vy1,w1,v2x1,v2y1,w21):
	i=0
	e3=[e2]
	a1=a
	V2x=[v2x1]
	V2y=[v2y1]
	W2=[w21]
	Vy=[vy1]
	Vx=[vx1]
	W=[w1]
	Theta=[theta1]
	X=[a[0]]
	Y=[a[1]]
	while Norm(sub(a,c))>2:	
		e1=e3[i]
		i=i+1
		
		e3.append(e(a1,c,a))
		
		de=e3[i]-e1
		control(e3[i],de)
		u1=control(e3[i],de)[0]
		u2=control(e3[i],de)[1]
		W2.append(w2(u1,u2))
		
		W.append(w(W2[i],W2[i-1],W[i-1],dt))
	
		Theta.append(theta(W[i],Theta[i-1],dt))
			
		V2x.append(v2x(Theta[i],u1,u2))
		V2y.append(v2y(Theta[i],u1,u2))
		Vx.append(v(V2x[i],V2x[i-1],Vx[i-1],dt))
		Vy.append(v(V2y[i],V2y[i-1],Vy[i-1],dt))
		X.append(xy(Vx[i],X[i-1],dt))
		Y.append(xy(Vy[i],Y[i-1],dt))
		xr=(random.random()-0.5)/5
		yr=(random.random()-0.5)/5
		a=[X[i]+xr,Y[i]+yr]
		if i==1500:
			print(i)
			break	
	A=[a,e3[i],Theta[i],Vx[i],Vy[i],W[i],V2x[i],V2y[i],W2[i],X,Y]

	
	
	return A

"""
V2x=[0]
V2y=[0]
W2=[0]
Vy=[0]
Vx=[0]
W=[0]
Theta=[0]
X=[0]
Y=[20]
P1=[0,0]
P2=[70,10]
P3=[24,50]
P4=[0,50]
P5=[50,60]
P6=[X[0],Y[0]]
E=[0]
de=0;
XY=[X[0],Y[0]]
U=[0,0]
e1=0
XY1=[XY,E[0],Theta[0],Vx[0],Vy[0],W[0],V2x[0],V2y[0],W2[0]]
user_input= raw_input("enter target point")

XY1=checkPoint(XY1[0],user_input,XY1[1],XY1[2],XY1[3],XY1[4],XY1[5],XY1[6],XY1[7],XY1[8])
X=XY1[9]
Y=XY1[10]
plt.plot(X,Y)
plt.draw()
plt.pause(1)
raw_input("<hit Enter to close>")
plt.close()	"""

if __name__=='__main__':
	
	V2x=[0]
	V2y=[0]
	W2=[0]
	Vy=[0]
	Vx=[0]
	W=[0]
	Theta=[0]
	X=[0]
	Y=[20]
	P1=[0,0]
	P2=[70,10]
	P3=[24,50]
	P4=[0,50]
	P5=[50,60]
	P6=[X[0],Y[0]]
	E=[0]
	de=0;
	XY=[X[0],Y[0]]
	U=[0,0]

	e1=0


	XY1=checkPoint(XY,P2,E[0],Theta[0],Vx[0],Vy[0],W[0],V2x[0],V2y[0],W2[0])
	X=XY1[9]
	Y=XY1[10]
	XY1=checkPoint(XY1[0],P3,XY1[1],XY1[2],XY1[3],XY1[4],XY1[5],XY1[6],XY1[7],XY1[8])
	X=X+XY1[9]
	Y=Y+XY1[10]
	XY1=checkPoint(XY1[0],P1,XY1[1],XY1[2],XY1[3],XY1[4],XY1[5],XY1[6],XY1[7],XY1[8])
	X=X+XY1[9]
	Y=Y+XY1[10]
	XY1=checkPoint(XY1[0],P4,XY1[1],XY1[2],XY1[3],XY1[4],XY1[5],XY1[6],XY1[7],XY1[8])
	X=X+XY1[9]
	Y=Y+XY1[10]
	XY1=checkPoint(XY1[0],P5,XY1[1],XY1[2],XY1[3],XY1[4],XY1[5],XY1[6],XY1[7],XY1[8])
	X=X+XY1[9]
	Y=Y+XY1[10]

	plt.plot(X,Y)
	
	print(XY1[0])
	print(Norm(sub(XY1[0],P5)))
	plt.draw()
	plt.pause(1)
	raw_input("<hit Enter to close>")
	plt.close()
