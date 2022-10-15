
from sympy import *

# Nombres de corps du robot

corps = 4

# Initialisation des paramètres DH avec indexation afin d'avoir d[i], alpha[i], etc... pour tout les corps du robot

d = IndexedBase('d',shape=(corps))
theta = IndexedBase('theta',shape=(corps))
r = IndexedBase('r',shape=(corps))
alpha = IndexedBase('alpha',shape=(corps))


# Initialisation des coordonnées articulaires q1, q2, etc...

q = IndexedBase('q',shape=(corps))

# Symboles des longueurs des corps du robot

(L0, L1, L3, L4, L2, x, mu, nu) = symbols('L0 L1 L3 L4 L2 x mu nu')


# Tableau des paramètres DH 

replacementDH= [
    (d[0], L0), (d[1], L1), (d[2], 0), (d[3], 0), (d[4], 0),
    (theta[0], 0), (theta[1], q[0]), (theta[2], -q[1] + mu), (theta[3], -q[2]-mu), (theta[4], -q[3]),
    (r[0], 0), (r[1], 0), (r[2], L2), (r[3], L3), (r[4], L4),
    (alpha[0], 0), (alpha[1], pi/2), (alpha[2], 0), (alpha[3], 0), (alpha[4], 0)
]

# Valeurs numériques des paramètres DH

replacementNum = [
    (L0, 0.1), (L1, 0.1), (L2, 0.1), (L3, 0.1), (mu, 0.1), (nu, 0.1), (L4, 0.1)
]

# Initialisation des matrices de transformation homogènes rigides

T = []

# Matrice de transformation finale

Tfinal = eye(4)

# Calcul de la matrice finale

for i in range(5):


    expre = Matrix([
        [cos(theta[i]), -1*sin(theta[i])*cos(alpha[i]), sin(theta[i])*sin(alpha[i]), r[i]*cos(theta[i])],
        [sin(theta[i]), cos(theta[i])*cos(alpha[i]), -1*cos(theta[i])*sin(alpha[i]), r[i]*sin(theta[i])],
        [0, sin(alpha[i]), cos(alpha[i]), d[i]],
        [0, 0, 0, 1]
    ])

    T.append(expre)

    # Calcul de la matrice finale
    Tfinal = Tfinal * T[i]


print("Matrice T")

print(Tfinal)

print("\n")

# Substiution des paramètres DH

Tfinal = Tfinal.subs(replacementDH)


print("Matrice T avec substitution")

print(Tfinal)

print("\n")

# Calcul de la jacobienne du robot

J_deriv = [[0,0,0,0],[0,0,0,0],[0,0,0,0]]

J_vecto = Matrix([])

J_cols = []

# On va calculer la jacobienne par deux façons: dérivation et produit vectoriel

# dérivation

# r = q*T avec q = [[q0],[q1],[q2],[q3],[1]]
# puis J[i][j] = dr[i]/dq[j]

# produit vectoriel

# On veut récupérer iTe en faisant 0Ti^-1 * 0Te
# On veut ire

# On cherche rc = [[x],[y],[z],[1]]

qCalcul = Matrix([[q[0]],[q[1]],[q[2]],[q[3]]])

rCalcul = Tfinal*qCalcul


# for i in range(5):



#     #ON VEUT ir4

#     # Calcul de 0Ti pour obtenir 0ri
#     T_courant = eye(4)
#     for j in range(i,5):
#         T_courant = T[i]*T_courant

#     print("ça inverse 4 fois")
#     #iTe = T_courant**(-1)*Tfinal

#     # On récupère 0ri dans la matrice de transformation
#     # On récupère ir4 dans la matrice de transformation
#     #ire = iTe[0:3, 3:4]

#     # On récupère 0Ri pour obtenir 0zi
#     #R0i = T_courant[0:3, 0:3]

#     #z0i = R0i*Matrix([[0],[0],[1]])

#     #J_cols.append(z0i.cross(ire)) 

#     #J_vecto = J_vecto.col_insert(i,J_cols[i])


for i in range(4):
    for o in range(3):
        J_deriv[o][i] = diff(rCalcul[o],q[i])
        print('ça dérive 12 fois')

print("Matrice J dérivée")

J_deriv = Matrix(J_deriv)
J_deriv= J_deriv.subs(replacementDH)

f = open("j.txt","w")
f.write(str(J_deriv))
f.close()
print(J_deriv)

# print("\n")

# print("\n")
 

#print("Matrice J vectorielle")

#print(J_vecto)

# print("\n")

# print("\n")

# #Substitution J 

# #J_vecto = J_vecto.subs(replacementDH)

# print("Dimensions de J:")
# print(shape(J_deriv))


# print("Matrice J deriv avec simplification")

# print(simplify(J_deriv))

# print("\n")

# print("\n")


print("Matrice J vecto avec substitution")

#print(J_vecto)

print("\n")

print("\n")

for i in range(len(T)):

    T[i] = T[i].subs(replacementDH)

    print("matrice T" + str(i) + " avec substituion:")

    print(T[i])

    print("\n")

    print("\n")


# Matrice J inverse

#print("Matrice J pseudo inverse")

#JpseudoInv = J.T*((J*J.T)**(-1))

#print(JpseudoInv)

print("\n")

print("\n")

# Substition des paramètres par leur valeur numérique

# Tnum = Tfinal.subs(replacementNum)
# Jnum = J.subs(replacementNum)

# print("Matrice T numérique")

# print(Tnum)

# print("\n")

# print("\n")

# print("Matrice J numérique")

# print(Jnum)

# print("\n")

# print("\n")

# Matrice J inverse

#print("Matrice J pseudo inverse")

#JpseudoInv = Jnum.T*((Jnum*Jnum.T)**(-1))

#print(JpseudoInv)