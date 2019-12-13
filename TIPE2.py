

##################  F0NCTION DE CORRECTION DU MOUVEMENT DU KITE EN 2D  #######################





# READ ME

''' Objectif: Automatiser la manoeuvre du kiteship '''

''' Méthode : Créer une fonction Correction :
    En entrée : position, forces subies, trajectoire souhaitée (en 8)
    En sortie : Vecteur correcteur permettant de rejoindre le mouvement souhaité'''





# IMPORTATION DES MODULES

import math
import matplotlib.pyplot as plt
import numpy as np

'''ax = plt.axes()
fig, ax = plt.subplots()
ax.quiver(X, Y, U, V,units='xy' ,scale=1)'''




# CREATION D'UNE CLASSE VECTEUR 2D

'''On se contentera pour l'intant d'une modélisation en 2D'''

''' Permet d'améliorer la gestion du code
    et de grandement faciliter sa lisibilité / compréhension,
    surtout grâce à l'utilisation de surcharges de méthodes '''

class Vecteur :

    def __init__ (self,x,y) :
        self.x = x
        self.y = y

    def __repr__ (self) :
        return '(' + str(self.x) + ',' + str(self.y) + ')'

    def Dessine (self, origine, couleur) :
        ''' Représente le vecteur self d'origine 'origine' '''
        if type(self) == Vecteur and type(origine) == Vecteur :
            X = [origine.x , origine.x + self.x]
            Y = [origine.y ,origine.y + self.y]
            plt.plot(X,Y,couleur)

    def Addition (self,v) :
        if type(self) == Vecteur and type(v) == Vecteur :
            return Vecteur(self.x + v.x , self.y + v.y)

    def __add__ (self,v) :
        return self.Addition(v)
        
    def Mult_Scalaire (self,a) :
        if type(self) == Vecteur and type(a) == (float or int) :
            return Vecteur (a*self.x , a*self.y)

    def __mul__ (self,a) :
        return self.Mult_Scalaire(a)
        
    def Soustraction (self,v) :
        if type(self) == Vecteur and type(v) == Vecteur :
            return Vecteur(self.x - v.x , self.y - v.y)

    def __sub__ (self,v) :
        return self.Soustraction(v)

    def Norme (self) :
        if type(self) == Vecteur :
            return np.sqrt(self.x**2 + self.y**2)






# DONNEES

''' Courbe théorique (équation paramétrique) '''

def x(t):
    return np.sin(t)

def y(t):
    return 1/3*np.sin(2*t)



''' Equations physiques '''
    
def Vitesse (V,t) :
    if type(V) == Vecteur :
        x_vitesse = 0 * V.x
        y_vitesse = 0.5 * V.y
        return Vecteur(x_vitesse , y_vitesse)

def Poids (m,g) :
    return Vecteur(0,-m*g)

def Trainée (ρ,S,Cx,V_app) :
    return 0.5 * ρ * S * Cx * V_app*V_app

def Portance (ρ,S,Cy,V_app) :
    return 0.5 * ρ * S * Cy * V_app*V_app

def Frottements (V, t) :
    V_app = Vent_apparent(t, Vitesse(V,t)).Norme()
    return Vecteur(Trainée(ρ,S,Cx,V_app), Portance(ρ,S,Cy,V_app))

def Vent (t) :
    x_vent = 0.1*t
    y_vent = 0
    return Vecteur(x_vent, y_vent)

def Vent_apparent (t,V):
    return Vent(t) + V

def Résultante_Forces (V, t) :
    return Poids(m,g) + Frottements (V, t) + Vent(t)



''' Constantes '''

global m # Masse (en kg)
global g # Constante gravitationnelle terrestre (en m/s**2)
global S # Surface de la voile (en m**2)
global ρ # Masse volumique de l'air au niveau de la mer a 20°C (en kg/m**3)
global Cx # Coefficient de traînée (sans dimension)
global Cy # Coefficient de portance (sans dimension)

m = 0.04 # Normalement 0.4
g = 9.8
S = 1.2
ρ = 1.225
Cx = 0.01
Cy = 0.01







# FONCTIONS 

def Vecteur_théorique (x,y,t,dt) :
    ''' Retourne vecteur_théorique = Vecteur(f(t)f(t+dt)) '''
    return Vecteur(x(t+dt) ,y(t+dt)) - Vecteur (x(t) ,y(t))


def Vecteur_non_corrigé (V,t) :
    ''' Equations physiques '''
    return Vitesse(V,t) + Résultante_Forces (V, t)

def Vecteur_correction (vecteur_théorique, vecteur_non_corrigé) :
    return vecteur_théorique - vecteur_non_corrigé


def Modélisation (x, y, a, b, n) :
    dt = (b-a)/n
    T = np.arange(a,b,dt)
    for t in T :
        V = Vecteur(x(t),y(t))
        vecteur_théorique = Vecteur_théorique (x,y,t,dt)
        vecteur_non_corrigé = Vecteur_non_corrigé (V,t)
        vecteur_correction = Vecteur_correction (vecteur_théorique, vecteur_non_corrigé)
        vecteur_théorique.Dessine(V,'k')
        vecteur_non_corrigé.Dessine(V,'orange')
        vecteur_correction.Dessine(V,'g')
        plt.pause(0.01) # PERMET D'ANIMER
        
        
def Modélisation_Retard (x, y, a, b, n) :
    dt = (b-a)/n
    T = np.arange(a,b,dt)
    V = Vecteur(x(a),y(a))
    Vréel1 = Vecteur_non_corrigé(V,a)
    for t in T :
        V = Vecteur(x(t),y(t))
        vecteur_théorique = Vecteur_théorique (x,y,t,dt)
        vecteur_non_corrigé = Vecteur_non_corrigé (Vréel1,t-1*dt)
        vecteur_correction = Vecteur_correction (vecteur_théorique, vecteur_non_corrigé)
        Vréel2 = vecteur_non_corrigé + vecteur_correction
        vecteur_théorique.Dessine(V,'k')
        Vréel2.Dessine(Vréel1,'g')
        Vréel1 += Vréel2
        

def Modélisation_Non_Corrigée (x, y, a, b, n) :
    dt = (b-a)/n
    T = np.arange(a,b,dt)
    V_init = Vecteur(x(a),y(a))
    xnc = Vecteur_non_corrigé (V_init,a).x
    ync = Vecteur_non_corrigé (V_init,a).y
    Vnc = Vecteur(xnc,ync)
    Vnc.Dessine(V_init,'orange')
    for t in T :
        Vt = Vecteur(x(t),y(t))
        vecteur_théorique = Vecteur_théorique (x,y,t,dt)
        vecteur_théorique.Dessine(Vt,'k')
        Vecteur_non_corrigé (Vnc,t).Dessine(Vnc,'orange')
        Vnc += Vecteur_non_corrigé (Vnc,t)
        plt.pause(0.01)




# MODELISATION

a = 0
b = 6*np.pi
n = 300


Modélisation (x, y, a, b, n)

'''Modélisation_Non_Corrigée (x, y, a, b, n)'''

'''Modélisation_Retard (x, y, a, b, n)'''

plt.show()

V = Vecteur(2,2)


