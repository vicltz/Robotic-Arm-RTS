# Robotic Arm RTS

Ce code permet le contrôle du bras robot de l'association **Robotique Telecom Strasbourg (RTS)** visible sur la photo **DHFINAL.png** . Il utilise des servomoteurs **Dynamixel**.

Il implémente le contrôleur de robot dont le principe est décrit sous forme de schéma dans le fichier **controlleur.png** (Le schéma provient du cours de Alexandre Girard de l'Université de Sherbrooke).

Ce contrôleur permet la commande en position du robot via un contrôle de vitesse au niveau des moteurs . Le code implanté dans le fichier **controle_bras.py** permet le contrôle en position a l'aide des 4 moteurs du robot. 



**controle_bras.py**: 

Contrôle en position du bras à l'aide des 4 moteurs (redondance), sans contrôle de l'orientation de l'outil.

Les auto-collisions ainsi que les singularités ne sont pas automatiquement évités ou détectés. 

Nécessite le SDK Dynamixel pour fonctionner.



**controle_bras2.py**

Contrôle en position du bras à l'aide des 3 premiers moteurs (pas de redondance), il est alors possible d'orienter l'outil à part avec le dernier moteur.

**<u>Attention: Ce code n'est pas fonctionnel et provoque un comportement erratique du robot.</u>**

Nécessite le SDK Dynamixel pour fonctionner.



**Sympy1.py**

Utilise la librairie de calcul symbolique Sympy ainsi que les paramètres de Denavit-Hartenberg visible sur l'image **DHFINAL.png** afin de calculer les expressions des différentes expressions des matrices de transformations ainsi que de la matrice Jacobienne (pour le contrôle du fichier **controle_bras.py** avec redondance). Une fois le script lancé, les matrices peuvent être récupérées dans le terminal ainsi que dans le fichier j.txt, puis être incluses dans le fichier **controle_bras.py** implémentant le contrôleur.

Nécessite la librairie Sympy pour fonctionner.



**Sympy2.py** 

Utilise la librairie de calcul symbolique Sympy ainsi que les paramètres de Denavit-Hartenberg visible sur l'image **DHFINAL.png** afin de calculer les expressions des différentes expressions des matrices de transformations ainsi que de la matrice Jacobienne (pour le contrôle du fichier **controle_bras2.py** sans redondance, sans tenir compte du dernier moteur). Une fois le script lancé, les matrices peuvent être récupérées dans le terminal ainsi que dans le fichier j.txt, puis être incluses dans le fichier **controle_bras2.py** implémentant le contrôleur.

Nécessite la librairie Sympy pour fonctionner.



SDK Dynamixel: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

Cours de robotique d'Alexandre Girard: https://www.alexandregirard.com/PDF/Notes_Robotique_UdeS.pdf

Librairie Sympy: https://www.sympy.org/en/index.html