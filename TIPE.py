

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
'''On se contentera pour l'intant d'une modélisation en 2D'''






# CREATION D'UNE CLASSE VECTEUR 2D

''' Permet d'améliorer la gestion du code
    et de grandement faciliter sa lisibilité / compréhension,
    surtout grâce à l'utilisation de surcharges de méthodes '''

class Vecteur :

    def __init__ (self,x,y) :
        self.x = x
        self.y = y

    def __repr__ (self) :
        return '(' + str(self.x) + ',' + str(self.y) + ')'

    def Dessine (self,x,y,couleur) :
        ''' Représente le vecteur self d'origine (x,y) '''
        if type(self) == Vecteur :
            X = [x , x + self.x]
            Y = [y , y + self.y]
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

    def Produit_Scalaire (self,v) :
        if type(self) == Vecteur and type(v) == Vecteur :        
            return self.x*v.x + self.y*v.y






# DONNEES

''' Courbe théorique (équation paramétrique) '''

def x(t):
    return np.sin(t)

def y(t):
    return 1/3*np.sin(2*t)



''' Equations physiques '''
    
def Vitesse (x,y,t) :
    x_vitesse = 0.2 * x(t)
    y_vitesse = 0.8 * x(t)
    return Vecteur(x_vitesse , y_vitesse)

def Poids (masse,g) :
    return Vecteur(0,-masse*g)

def Frottements (coeff_frottement, x, y, t) :
    vecteur_vitesse = Vitesse (x,y,t)
    x_frottements = -vecteur_vitesse.x * coeff_frottement
    y_frottements = -vecteur_vitesse.y * coeff_frottement
    return Vecteur (x_frottements, y_frottements)

def Vent (t) :
    x_vent = 0.5
    y_vent = 0
    return Vecteur(x_vent, y_vent)

def Résultante_Forces (x, y, t) :
    return Poids(masse,g) + Frottements (coeff_frottement, x, y, t) + Vent(t)


''' Constantes '''

global coeff_frottement
global masse
global g #constante gravitationnelle terrestre
global Surface_voile

coeff_frottement = 0.1
masse = 0.04 #kg normalement 0.4
g = 9.8
Surface_voile = 1.2 #m^2



# FONCTIONS 

def Calcul_vecteur_théorique (x,y,t,dt) :
    ''' Retourne vecteur_théorique = Vecteur(f(t)f(t+dt)) '''
    x_théorique = x(t+dt) - x(t)
    y_théorique = y(t+dt) - y(t)
    return Vecteur(x_théorique,y_théorique)


def Calcul_vecteur_non_corrigé (x,y,t) :
    ''' Equations physiques '''
    return Vitesse(x,y,t) + Résultante_Forces (x, y, t)

def Calcul_vecteur_correction (vecteur_théorique, vecteur_non_corrigé) :
    return vecteur_non_corrigé - vecteur_théorique


def Modélisation (x, y, a, b, n) :
    dt = (b-a)/n
    T = np.arange(a,b,dt)
    for t in T :
        vecteur_théorique = Calcul_vecteur_théorique (x, y, t, dt)
        vecteur_non_corrigé = Calcul_vecteur_non_corrigé(x,y,t)
        vecteur_correction = Calcul_vecteur_correction (vecteur_théorique, vecteur_non_corrigé)
        vecteur_théorique.Dessine(x(t), y(t),'k')
        vecteur_non_corrigé.Dessine(x(t), y(t),'orange')
        vecteur_correction.Dessine(x(t), y(t),'g')
        

        



# MODELISATION :

a = 0
b = 4*np.pi
n = 300


Modélisation (x, y, a, b, n)
plt.show()



    
