

##################  F0NCTION DE CORRECTION DU MOUVEMENT DU KITE EN 3D  #######################






##########        READ ME

''' Objectif: Automatiser la manoeuvre '''

''' Stratégie : Créer une fonction Correction :
    En entrée : position, forces subies, trajectoire souhaitée (en 8)
    En sortie : Vecteur correcteur permettant de rejoindre le mouvement souhaité '''






##########        IMPORTATION DES MODULES

import math
import matplotlib.pyplot as plt
from numpy import *

from mpl_toolkits.mplot3d import Axes3D     # Pour une modélisation 3D
ax = Axes3D(plt.figure())






##########         CREATION D'UNE CLASSE VECTEUR 3D

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


    def Dessine2 (self, origine) :
        
        ''' Représente le vecteur self à l'origine "origine" '''
        ''' On utilise l'outil quiver pour représenter une forme de flèche '''
        
        if type(self) == Vecteur and type(origine) == Vecteur :
            plt.quiver(origine.x, origine.y, origine.z, self.x, self.y, self.z, length = 0.00002, normalize = False)


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
        
        ''' Retourne le produit scalaire usuel dans R^3 de self et other '''
        
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






##########        ROTATION 

''' Il est nécessaire d'appliquer une matrice de rotation
    à chaque vecteur position afin de prendre en compte
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
global θ    # Angle entre l'axe horizontal Oy et la corde OM       

θ = pi/4
Rot = Rotation(θ,'x')






##########         EQUATION PARAMETRIQUE EN 3D (Courbe de Viviani)

''' La courbe de Viviani modélise parfaitement le mouvement réel du Kite
    Il s'agit de l'intersection d'une sphère et d'un cylindre '''

''' On utilise des variables globales car elle sont fixées
    et cela permet d'alléger le nombre d'arguments des fonctions ci dessous '''

global R    # Rayon de la sphère (en mètres) 
global r    # Rayon du cylindre (ou est il passé ?) (en mètres)
global l    # Longueur des fils (en mètres)
global h    # Hauteur du Kite à l'instant t0 (début du mouvement) (en mètres)

R = 200
r = R/10
l = R           # Le rayon de la sphère dans laquelle peut évoluer le Kite est la longeur des fils
h = l*sin(θ)    # La hauteur du Kite est la projection de sa position sur z0

''' Pour pouvoir paramétrer cette courbe,
    il faut choisir l'orientation de la base de l'espace.
    On choisit donc les notations usuelles :
    
    x0 orienté vers la droite,
    y0 rentrant (croissant selon le sens de déplacement du bateau),
    z0 orienté vers le haut.
    
    Alors les équations paramétriques s'écrivent : '''

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






##########         EQUATIONS PHYSIQUES

''' Equations physiques : On repère le kite par
        - le temps t
        - sa position M(x(t), y(t), z(t))
        - sa vitesse V(M,dt)
    Ainsi, la donnée de t et dt est suffisante pour le calcul des forces '''

''' Constantes physiques : '''

global m        # Masse (en kg)
global g        # Constante gravitationnelle terrestre (en m/s**2)
global S        # Surface de la voile (en m**2)
global ρ        # Masse volumique de l'air au niveau de la mer a 20°C (en kg/m**3)
global Cx       # Coefficient de traînée (sans dimension)
global Cy       # Coefficient de portance (sans dimension)

m = 20
g = 9.8
S = 200
ρ = 1.225
Cx = 0.6
Cz = 0.7


def NormeV (t) :
    ''' Seule la norme de la vitesse peut être obtenue théoriquement '''
    return 10 * cos(t-pi/4)**2 + 1


def Vitesse (t,dt) :
    V = M(t+dt) - M(t)      # Vecteur unitaire vitesse (tangent au mouvement)
    V = V * NormeV(t)       # Multiplié par sa norme
    return V                # en m/s

    
def Poids () :
    ''' Selon z décroissants '''
    return Vecteur(0,0,-m*g)    # En Newtons


def Vent (t) :
    ''' A choisir arbitrairement pour observer les conséquences '''
    x_vent = 0
    y_vent = 0
    z_vent = 0
    return Vecteur(x_vent, y_vent, z_vent)  # en m/s


def Vent_apparent (t,dt) :
    ''' Vent effectivement subi par le Kite, dépendant de sa vitesse '''
    return Vent(t) + Vitesse(t,dt)          # en m/s


def Trainée (t,dt) :
    V_app = Vent_apparent(t, dt)
    N_app = V_app.Norme()
    Norme_Trainée = 0.5 * ρ * S * Cx * N_app**2         # Expression de la norme de la trainée
    Sens_Trainée = V_app * (-1/N_app)                   # La trainée est de sens opposé au vent apparent
    return Sens_Trainée * Norme_Trainée                 # En Newtons


def Portance (t,dt) :
    V_app = Vent_apparent(t, dt)
    N_app = V_app.Norme()
    Norme_Portance = 0.5 * ρ * S * Cz * N_app**2
    Sens_Portance = (z0 + y0)                           # La portance est orthogonal au vent apparent
    return Sens_Portance * Norme_Portance


def Résultante_Forces (t,dt) :
    ''' On somme toutes les forces en jeu '''
    return Poids() + Trainée(t,dt) + Portance(t,dt)     # En Newtons






##########          MODELISATION DU PORTE CONTENEUR

''' On réprésente le porte-conteneur pour donner un sens de direction et de perspective '''

global X_porte_conteneur
global Y_porte_conteneur
global Z_porte_conteneur

X_porte_conteneur = [0, 0, 5, 5, 0, -2.5, -2.5, 0, -2.5, 7.5, 5, 7.5, 7.5, 5, 7.5, 2.5, -2.5, 0, 2.5, 5, 7.5, 7.5, 7.5, -2.5, -2.5, 7.5, 7.5, 7.5, 7.5, 7.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, 7.5, 7.5, 7.5, -2.5, -2.5, 7.5, 7.5, 7.5, 7.5, 7.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, 7.5]
Y_porte_conteneur = [0, 50, 50, 0, 0, 0, 50, 50, 50, 50, 50, 50, 0, 0, 0, -20, 0, 0, -20, 0, 0, 15, 35, 35, 15, 15, 15, 35, 35, 35, 35, 35, 35, 15, 15, 15, 15, 20, 30, 30, 20, 20, 20, 30, 30, 30, 30, 30, 30, 20, 20, 20, 20]
Z_porte_conteneur = [0, 0, 0, 0, 0, 5, 5, 0, 5, 5, 0, 5, 5, 0, 5, 5, 5, 0, 5, 0, 5, 5, 5, 5, 5, 5, 10, 10, 5, 10, 10, 5, 10, 10, 5, 10, 10, 10, 10, 10, 10, 10, 15, 15, 10, 15, 15, 10, 15, 15, 10, 15, 15]

''' Réorientations '''

def Orientation (x) :
    return -x

Y_porte_conteneur = [ Orientation(y) for y in Y_porte_conteneur ]

''' Mise à l'échelle '''

def Echelle (a,x) :
    return a*x

X_porte_conteneur = [ Echelle(5,x) for x in X_porte_conteneur ]
Y_porte_conteneur = [ Echelle(6,y) for y in Y_porte_conteneur ]
Z_porte_conteneur = [ Echelle(3,z) for z in Z_porte_conteneur ]

''' Translations '''

def Translation (a,x) :
    return a + x

X_porte_conteneur = [ Translation(-12.5,x) for x in X_porte_conteneur ]
Y_porte_conteneur = [ Translation(-120,y) for y in Y_porte_conteneur ]
Z_porte_conteneur = [ Translation(-15,z) for z in Z_porte_conteneur ]






#####           MODELISATION DE LA TRAJECTOIRE, FORCES ET VITESSES

''' Base orthonormée de l'espace, pour pouvoir projeter les vecteurs : '''

global x0
global y0
global z0

x0 = Vecteur(1,0,0)
y0 = Vecteur(0,1,0)
z0 = Vecteur(0,0,1)

global a    # Temps initial
global b    # Temps final
global n    # Nombre de pas

a = 0
b = 2* pi
n = 100


def Vecteur_théorique (t,dt) :
    ''' Retourne vecteur_théorique = vecteur tangent au mouvement '''
    Vth = M(t+dt) - M(t)
    return Vth


def Modélisation (a, b, n) :
    
    ''' a,b : temps initial et final de l'intervalle sur lequel on modélise
        n : nombre de pas de la modélisation '''
    
    dt = (b-a)/n                                # Taille du pas
    T = arange(a,b,dt)                          # Discrétisation de l'intervalle

    ''' On annote la figure pour s'orienter '''

    plt.xlabel('x0 (en mètres)')
    plt.ylabel('y0 (en mètres)')
    plt.title('Modélisation de la trajectoire du Kite')
    plt.plot(X_porte_conteneur, Y_porte_conteneur, Z_porte_conteneur)      # Réprésentation du porte-conteneur    
    M(a).Dessine(Vecteur(0,0,0),'red')          # Réprésentation des deux fils
    
    for t in T :                                # A chaque instant                      
        
        ''' On calcule les 3 vecteurs nécessaires '''
            
        vecteur_théorique = Vecteur_théorique (t,dt)
        Forces = Résultante_Forces(t,dt)
        vitesse = Vitesse(t,dt)
        
        ''' Puis on les représente sur la figure. '''

        Pos = M(t)                              # Position de la voile à l'instant t
        vecteur_théorique.Dessine(Pos,'k')
        Forces.Dessine2(Pos)
        vitesse.Dessine(Pos,'orange')
        plt.pause(0.0000001)                    # Permet d'animer



Modélisation (a, b, n)
plt.show()




##########          COMPARAISON DE DIFFERENTES TRAJECTOIRES

# PROBLEME : la vitesse n'a été calculée que dans le cas de la courbe en 8 

def Puissance_Moyenne (x, y, z, a, b, n) :

    ''' Retourne la puissance moyenne en Watts d'une certaine courbe x,y,z
        sur l'intervalle de temps [a,b]. '''

    dt = (b-a)/n
    T = arange(a,b,dt)
    P = 0
    
    for t in T :
        Pos = Vecteur( x(t) ,y(t), z(t) )
        F = Résultante_Forces (t,dt)        
        P += F.Produit_Scalaire (y0)        # Car on a choisi y0 croissant selon le sens de déplacement du bateau
        
    return P    # En Watts.

        
def Comparaison (x1, y1, z1, x2, y2, z2, a, b, n) :

    ''' Comparaison de la puissance moyenne fournie par deux courbes différentes.
        Si notre hypothèse est juste, la trajectoire en '8' devrait toujours en sortir victorieuse. '''
        
    P1 = Puissance_Moyenne(x1, y1, z1, a, b, n)
    P2 = Puissance_Moyenne(x2, y2, z2, a, b, n)
    if P1 == P2 :
        return 'Les deux courbes fournissent en moyenne la même puissance : ' + str(P1) + ' Watts.'

    elif P1 > P2 :
        return 'La courbe 1 fournit en moyenne une puissance plus importante que la courbe 2. En effet : ' 'P1 = '+ str(P1) + ' W > P2 = ' + str(P2) + ' W.'

    elif P2 > P1 :
        return 'La courbe 2 fournit en moyenne une puissance plus importante que la courbe 1. En effet : ' 'P2 = '+ str(P2) + ' W > P1 = ' + str(P1) + ' W.'



''' Comparaison du mouvement "en 8" et "immobile" '''

def x2(t) :
    return 0

def y2(t) :
    return l

def z2(t) :
    return h

print( Comparaison(x, y, z, x2, y2, z2, a, b, n) )
print( "A titre de comparaison, la puissance de propulsion du paquebot Queen Mary 2, l'un des plus gros paquebots du monde, est de 117 MW (Source : Carnival Group, constructeur)." )






##########          CORRECTION ET APPLICATION AVEC ARDUINO


def Vecteur_correction (t,dt) :
    
    ''' Le vecteur correction recherché est alors la difference entre
        le vecteur résultante des forces et le vecteur théorique. '''
    
    V_corr = Résultante_Forces(t,dt) - Vecteur_théorique(t,dt)
    return V_corr


def Scalaire_correction (t,dt) :
    
    ''' On ne peut cependant controler le Kite que selon x0
        d'où la nécessité d'une projection : '''
    
    V_corr = Vecteur_correction(t,dt)
    S_corr = V_corr.Produit_Scalaire(x0)
    return S_corr


def Consigne_Moteurs (t,dt) :
    
    ''' L'objectif est de controler la voile par deux moteurs à double sens de rotation,
        chacun ayant le contrôle d'un côté du fil.
        
        Il faut donc à chaque instant fournir aux moteurs un réel
        appartenant à [-vitesse_de_rotation_max, vitesse_de_rotation_max]
        dont le signe indique le sens de rotation,
        et la valeur absolue la vitesse de rotation.
        On choisit la convention ' + ' pour tirer et ' - ' pour laisser filer.
        
        On néglige pour l'instant le temps de réaction du moteur
        (il suffit de renvoyer Arduino(t-tréac,dt) pour en prendre compte).

        Pour les premiers tests, on remplacera les moteurs par des diodes :
        5 à gauche et 5 à droite,
        de couleurs "proportiennelles" à la vitesse de rotation. '''
    
    S_corr = Scalaire_correction(t,dt)

    ''' On répartit l'effort sur les deux moteurs pour protéger le système '''
    
    moteur_gauche = -S_corr / 2      # Car x0 est orienté vers la droite
    moteur_droit = S_corr / 2
    return moteur_gauche , moteur_droit 


def Arduino (a,b,n) :
    
    ''' a,b : temps initial et final de l'intervalle sur lequel on modélise
        n : nombre de pas de la modélisation '''
    
    dt = (b-a)/n                        # Taille du pas
    T = arange(a,b,dt)                  # Discrétisation de l'intervalle
    for t in T :                        # A chaque instant :
        print(Consigne_Moteurs(t,dt))   # On applique la fonction ci-dessus


Arduino(a,b,n)

    
