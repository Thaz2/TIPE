

##################  F0NCTION DE CORRECTION DU MOUVEMENT DU KITE EN 3D  #######################





#####        READ ME

''' Objectif: Automatiser la manoeuvre '''

''' Stratégie : Créer une fonction Correction :
    En entrée : position, forces subies, trajectoire souhaitée (en 8)
    En sortie : Vecteur correcteur permettant de rejoindre le mouvement souhaité '''





#####        IMPORTATION DES MODULES

import math
import matplotlib.pyplot as plt
from numpy import *

# Pour une modélisation 3D :

from mpl_toolkits.mplot3d import Axes3D
ax = Axes3D(plt.figure()) 




#####         CREATION D'UNE CLASSE VECTEUR 3D

''' Permet d'améliorer la gestion du code
    et de grandement faciliter sa lisibilité / compréhension,
    surtout grâce à l'utilisation de surcharges de méthodes '''

class Vecteur :


    def __init__ (self,x,y,z) :

        ''' On décrit ces vecteurs par leurs trois coordonnées cartésiennes '''
        
        self.x = x
        self.y = y
        self.z = z


    def __repr__ (self) :
        return 'vecteur = ' + '(' + str(self.x) + ',' + str(self.y) + ',' + str(self.z) + ')'


    def Dessine (self, origine, couleur) :
        
        ''' Représente le vecteur self à l'origine "origine" '''
        
        if type(self) == Vecteur and type(origine) == Vecteur :
            X = [origine.x , origine.x + self.x]
            Y = [origine.y , origine.y + self.y]
            Z = [origine.z , origine.z + self.z]
            plt.plot(X,Y,Z,couleur)


    def Dessine2 (self, origine, couleur) :
        
        ''' Représente le vecteur self à l'origine "origine" '''
        ''' On utilise l'outil quiver pour représenter une forme de flèche '''
        
        if type(self) == Vecteur and type(origine) == Vecteur :
            plt.quiver(origine.x, origine.y, origine.z, self.x, self.y, self.z, length = 0.05, normalize = True)


    def Addition (self,other) :
        
        ''' Additionne le vecteur "self et le vecteur "other" '''
        
        if type(self) == Vecteur and type(other) == Vecteur :
            x = self.x + other.x
            y = self.y + other.y
            z = self.z + other.z
            return Vecteur(x,y,z)

        
    def __add__ (self,other) :
        
        ''' Permet de remplacer la commade self.Addition(other)
            par la commande plus courte, plus claire et plus lisible
            self + other '''

        return self.Addition(other)

    
    def Mult_Scalaire (self,a) :

        ''' Multiplie le vecteur "self" par le scalaire "a" '''
        
        if type(self) == Vecteur :
            x = a*self.x
            y = a*self.y
            z = a*self.z
            return Vecteur (x,y,z)

        
    def __mul__ (self,a) :
        
        ''' Permet d'écrire a*self '''
        
        return self.Mult_Scalaire(a)

    
    def Soustraction (self,other) :
        if type(self) == Vecteur and type(other) == Vecteur :
            x = self.x - other.x
            y = self.y - other.y
            z = self.z - other.z
            return Vecteur(x,y,z)


    def __sub__ (self,other) :
        
        ''' Permet d'écrire self - other '''
        
        return self.Soustraction(other)


    def Norme (self) :
        if type(self) == Vecteur :
            return sqrt(self.x**2 + self.y**2 + self.z**2)

        
    def Produit_Scalaire (self,other) :
        if type(self) == Vecteur and type(other) == Vecteur :        
            return self.x*other.x + self.y*other.y + self.z*other.z


    def FromVecteurToArray (self) :
        
        ''' Permet de passer de la classe Vecteur ici créée
            à la classe Array afin de pouvoir
            multiplier par une matrice / M.dot(V) '''
        
        if type(self) == Vecteur :
            return array([ [self.x], [self.y], [self.z] ])


    def ProduitMatrice (self,M) :
        
        ''' Renvoie le Vecteur V transformé par la matrice M.
            Sera utile pour appliquer une rotation '''
        
        if type(self) == Vecteur and len(M) == 3 :
            V = self.FromVecteurToArray()
            return FromArrayToVecteur( M.dot(V) )


def FromArrayToVecteur (V) :
    
    ''' Permet de retourner de la classe Array à la classe Vecteur '''
    
    x, y, z = V[0][0], V[1][0], V[2][0]
    return Vecteur(x,y,z)






#####        ROTATION 

''' Il est nécessaire d'appliquer une matrice de rotation
    aux courbes ci dessus afin de prendre en compte
    l'inclinaison du kite, qui a une grande influence
    sur les forces en jeu '''

def Rotation (angle,axe) :
    
    ''' Retourne une matrice de rotation d'angle "angle" et d'axe "axe" '''
    
    θ =  angle
    if axe == 'x' :
        return array ([ [ 1, 0, 0 ],
                        [ 0, cos(θ), -sin(θ) ],
                        [ 0, sin(θ), cos(θ) ] ])
    
    if axe == 'y' :           
        return array ([ [ cos(θ), 0, sin(θ) ],
                        [ 0, 1, 0 ],
                        [ -sin(θ) ,0, cos(θ) ] ])

    if axe == 'z' :
        return array ([ [ cos(θ), -sin(θ), 0 ],
                        [ sin(θ), cos(θ), 0 ],
                        [ 0, 0, 1 ] ])
                        
                        
global Rot  # Matrice de rotation à appliquer à tous les vecteurs
            

Rot = Rotation(pi/6,'x')







### EQUATION PARAMETRIQUE EN 3D (Courbe de Viviani)

''' La courbe de Viviani modélise parfaitement le mouvement réel du Kite
    Il s'agit de l'intersection d'une sphère et d'un cylindre '''

''' On utilise des variables globales car elle sont fixées
    et cela permet d'alléger le nombre d'arguments des fonctions ci dessous '''

global R    # Rayon de la sphère
global a    # Rayon du cylindre
global l    # Longueur de la corde
global h    # Hauteur du Kite au centre du '8'

R = 1
a = R/2
l = 2
h = 2

''' Pour pouvoir l'appliquer, il faut choisir l'orientation de la base de l'espace :
    On choisit donc y croissant selon la direction du bateau,
    z orienté vers le haut,
    x orienté vers la droite. '''

global x
global y
global z
global M

def x (t) :
    return R*sin(t)

def y (t) :
    return R*cos(t)**2 + l

def z (t) :
    return R*sin(-t)*cos(t) + h


def M (t) :

    ''' Retourne la position du Kite à l'instant t '''
    
    Pos = Vecteur( x(t) ,y(t), z(t) )       # Coordonnées de la courbe de Viviani
    Pos = Pos.ProduitMatrice(Rot)           # Auxquelles on applique la rotation
    return Pos







### EQUATIONS PHYSIQUES

''' Equations physiques : On repère le kite par
        - le temps t
        - sa position M(x(t), y(t), z(t))
        - sa vitesse V(M,dt)
    Ainsi, la donnée de t et dt est suffisante au calcul des forces '''

''' Constantes physiques : '''

global m        # Masse (en kg)
global g        # Constante gravitationnelle terrestre (en m/s**2)
global S        # Surface de la voile (en m**2)
global ρ        # Masse volumique de l'air au niveau de la mer a 20°C (en kg/m**3)
global Cx       # Coefficient de traînée (sans dimension)
global Cy       # Coefficient de portance (sans dimension)

m = 0.03
g = 9.8
S = 1.2
ρ = 1.225
Cx = 1
Cy = 1


def NormeV (t) :
    return 10 * cos(t - pi/6)**2 + 2


def Vitesse (t,dt) :
    V = M(t+dt) - M(t)
    V = V * NormeV(t)
    return V    # en m/s

    
def Poids () :
    ''' Selon z décroissants '''
    return Vecteur(0,0,-m*g)


def Vent (t) :
    ''' A choisir arbitrairement pour observer les conséquences '''
    x_vent = 0
    y_vent = 0
    z_vent = 0
    return Vecteur(x_vent, y_vent, z_vent)


def Vent_apparent (t,dt) :
    return Vent(t) + Vitesse(t,dt)


def Trainée (t,dt) :
    V_app = Vent_apparent(t, dt).Norme()
    return 0.5 * ρ * S * Cx * V_app*V_app


def Portance (t,dt) :
    V_app = Vent_apparent(t, dt).Norme()
    return 0.5 * ρ * S * Cy * V_app*V_app


def Frottements (t,dt) :
    ''' La Trainée est selon x et la portance selon z '''
    return Vecteur(Trainée(t,dt), 0, Portance(t,dt))


def Résultante_Forces (t,dt) :
    ''' On somme toutes les forces en jeu '''
    return Poids() + Frottements (t,dt) + Vent(t)







### FONCTIONS


def Vecteur_théorique (t,dt) :
    ''' Retourne vecteur_théorique = Vecteur(f(t)f(t+dt)) '''
    Vth = M(t+dt) - M(t)
    return Vth


def Vecteur_non_corrigé (t,dt):
    ''' Issu des équations physiques '''
    Vnc = Vitesse(t,dt)*0 + Résultante_Forces(t,dt)
    return Vnc

            
def Vecteur_correction (t,dt) :
    ''' Le vecteur correction recherché est alors la difference des deux précédents '''
    Vcr = Vecteur_théorique(t,dt) - Vecteur_non_corrigé(t,dt)
    return Vcr



def Modélisation (a, b, n) :
    
    ''' a,b : temps initial et final de l'intervalle sur lequel on modélise
        n : nombre de pas de la modélisation '''
    
    dt = (b-a)/n                            # Taille du pas
    T = arange(a,b,dt)                      # Discrétisation de l'intervalle

    ''' On annote la figure pour s'orienter '''

    plt.xlabel('x (en mètres)')
    plt.ylabel('y (en mètres)')
    plt.title('Modélisation de la trajectoire du Kite')
    M(a).Dessine(Vecteur(0,0,0),'b')        # Fil
    
    for t in T :                            # A chaque instant                      
        
        ''' On calcule les 3 vecteurs nécessaires '''
            
        vecteur_théorique = Vecteur_théorique (t,dt)
        vecteur_non_corrigé = Vecteur_non_corrigé (t,dt)
        vecteur_correction = Vecteur_correction (t,dt)
        
        ''' Puis on les représente sur la figure '''

        Pos = M(t)                          # Position de la voile à l'instant t
        vecteur_théorique.Dessine(Pos,'k')
        vecteur_non_corrigé.Dessine(Pos,'orange')
        #vecteur_correction.Dessine(Pos,'g')
        plt.pause(0.0000001)                # Permet d'animer
        



        
def Puissance_Moyenne (x, y, z, a, b, n) :

    ''' Retourne la puissance moyenne d'une certaine courbe x,y,z
        sur l'intervalle de temps [a,b] '''

    dt = (b-a)/n
    T = arange(a,b,dt)
    P = 0
    v0 = Vecteur(1,0,0)                 # Vecteur unitaire dans la direction du bateau
    
    for t in T :
        Pos = M(t)
        F = Résultante_Forces (t,dt)        
        P += F.Produit_Scalaire (v0)    # La puissance est la résultante des forces projetée sur ce vecteur
        
    return P

        
def Comparaison (M1,M2) :
    return 1
    

        

### MODELISATION :



a = 0
b = 2*pi
n = 150

Modélisation (a, b, n)
plt.show()

#P = Puissance_Moyenne (x, y, z, a, b, n)

    
