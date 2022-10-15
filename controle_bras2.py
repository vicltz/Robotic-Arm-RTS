
# Importation de msvcrt pour récupérer les touches du clavier
import os
import msvcrt
from math import *
import numpy as np

if os.name == 'nt':
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# DOCUMENTATION MOTEURS: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
# DOCUMENTATION SDK: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_porthandler/#python-porthandler


# Fonctions permettant de passer des vitesses et coordonées usuelles aux coordonées et vitesses Dynamixel

def dxToRadSec(dx):
    # Transforme la vitesse Dynamixel en vitesse rad/s
    return dx*0.229*2*pi/60

    # Réciproque
def radSecToDx(radSec):
    return round(radSec*60/(0.229*2*pi))

# Retourne une position relative Dynamixel (entre 0 et 4095) depuis les positions absolue Dynamixel (0-N*4095)
def getRelativePos(pos):
    return (pos%4095)

# Retourne la position en degrée depuis une position relative dynamixel (0-4095)
def getDegreesFromRelativePos(relativePos):
    return relativePos/4095*360

# Retourne la position en radians depuis une position relative dynamixel (0-4095)
def getRadfromRelativePos(relativePos):
    return relativePos/4095*2*pi


# Matrice Jacobienne du robot en position

def J(q,L0,L1,L2,L3,mu,nu):
    return np.array([[0, -L1*np.sin(q[1][0]), -2*L2*np.sin(mu + q[2][0])*np.cos(mu + q[2][0]) - L2*np.sin(mu + q[2][0]), -L3*(-np.sin(nu + q[3][0])**2 + np.cos(nu + q[3][0])**2)*np.sin(nu + q[3][0]) - 2*L3*np.sin(nu + q[3][0])*np.cos(nu + q[3][0])**2 - 2*L3*np.sin(nu + q[3][0])*np.cos(nu + q[3][0]) - L3*np.sin(nu + q[3][0])], [0, 0, -L2*np.sin(mu + q[2][0])**2 + L2*np.cos(mu + q[2][0])**2 + L2*np.cos(mu + q[2][0]), L3*(-np.sin(nu + q[3][0])**2 + np.cos(nu + q[3][0])**2)*np.cos(nu + q[3][0]) - 2*L3*np.sin(nu + q[3][0])**2*np.cos(nu + q[3][0]) - L3*np.sin(nu + q[3][0])**2 + L3*np.cos(nu + q[3][0])**2 + L3*np.cos(nu + q[3][0])], [0, 0, 0, 0]])

def T(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[np.cos(mu - q[1])*np.sin(mu + q[2])*np.cos(q[0]) + np.cos(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0]), -np.sin(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0]) + np.sin(mu + q[2])*np.cos(mu - q[1])*np.cos(q[0]), np.sin(q[0]), L2*np.cos(mu - q[1])*np.cos(q[0]) + L3*np.sin(mu - q[1])*np.sin(mu + q[2])*np.cos(q[0]) + L3*np.cos(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0])], [np.sin(mu - q[1])*np.sin(mu + q[2])*np.sin(q[0]) + np.sin(q[0])*np.cos(mu - q[1])*np.cos(mu + q[2]), -np.sin(mu - q[1])*np.sin(q[0])*np.cos(mu + q[2]) + np.sin(mu + q[2])*np.sin(q[0])*np.cos(mu - q[1]), -np.cos(q[0]), L2*np.sin(q[0])*np.cos(mu - q[1]) + L3*np.sin(mu - q[1])*np.sin(mu + q[2])*np.sin(q[0]) + L3*np.sin(q[0])*np.cos(mu - q[1])*np.cos(mu + q[2])], [np.sin(mu - q[1])*np.cos(mu + q[2]) - np.sin(mu + q[2])*np.cos(mu - q[1]), np.sin(mu - q[1])*np.sin(mu + q[2]) + np.cos(mu - q[1])*np.cos(mu + q[2]), 0, L0 + L1 + L2*np.sin(mu - q[1]) + L3*np.sin(mu - q[1])*np.cos(mu + q[2]) - L3*np.sin(mu + q[2])*np.cos(mu - q[1])], [0, 0, 0, 1]],dtype=np.float64)
    

def J2(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[-np.sin(mu - q[1])*np.sin(mu + q[2])*np.sin(q[0]) - np.sin(q[0])*np.cos(mu - q[1])*np.cos(mu + q[2]), np.sin(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0]) - np.sin(mu + q[2])*np.cos(mu - q[1])*np.cos(q[0]), np.sin(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0]) - np.sin(mu + q[2])*np.cos(mu - q[1])*np.cos(q[0])], [np.sin(mu - q[1])*np.sin(q[0])*np.cos(mu + q[2]) - np.sin(mu + q[2])*np.sin(q[0])*np.cos(mu - q[1]), np.sin(mu - q[1])*np.sin(mu + q[2])*np.cos(q[0]) + np.cos(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0]), np.sin(mu - q[1])*np.sin(mu + q[2])*np.cos(q[0]) + np.cos(mu - q[1])*np.cos(mu + q[2])*np.cos(q[0])], [np.cos(q[0]), 0, 0]],dtype=np.float64)
    
def T0(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, L0], [0, 0, 0, 1]],dtype=np.float64)

def T1(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[np.cos(q[0]), 0, np.sin(q[0]), 0], [np.sin(q[0]), 0, -np.cos(q[0]), 0], [0, 1, 0, L1], [0, 0, 0, 1]],dtype=np.float64)

def T2(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[np.cos(mu - q[1]), -np.sin(mu - q[1]), 0, L2*np.cos(mu - q[1])], [np.sin(mu - q[1]), np.cos(mu - q[1]), 0, L2*np.sin(mu - q[1])], [0, 0, 1, 0], [0, 0, 0, 1]],dtype=np.float64)

def T3(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[np.cos(mu + q[2]), np.sin(mu + q[2]), 0, L3*np.cos(mu + q[2])], [-np.sin(mu + q[2]), np.cos(mu + q[2]), 0, -L3*np.sin(mu + q[2])], [0, 0, 1, 0], [0, 0, 0, 1]],dtype=np.float64)

def T4(q,L0,L1,L2,L3,L4,mu,nu):
    return np.array([[np.cos(q[3]), np.sin(q[3]), 0, L4*np.cos(q[3])], [-np.sin(q[3]), np.cos(q[3]), 0, -L4*np.sin(q[3])], [0, 0, 1, 0], [0, 0, 0, 1]],dtype=np.float64)

# SDK Dynamixel contenant les méthodes permettant de communiquer avec les moteurs

from dynamixel_sdk import *                    

# Table d'adresse de contrôle des moteurs
ADDR_PRO_TORQUE_ENABLE      = 64             
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132
ADDR_GOAL_VELOCITY          = 104
ADDR_PROFILE_VELOCITY       = 112
ADDR_MOVING                 = 122
ADDR_PRESENT_VELOCITY       = 128
ADDR_VELOCITY_TRAJECOTRY    = 136
ADDR_OPERATING_MODE         = 11 # EEPROM

# Version du protocole 
PROTOCOL_VERSION            = 2.0               

# Default setting             
BAUDRATE                    = 1000000             
DEVICENAME                  = 'COM7'   


# Paramètres à modifier

TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                
DXL_MINIMUM_POSITION_VALUE  = 10           
DXL_MAXIMUM_POSITION_VALUE  = 4000            
DXL_MOVING_STATUS_THRESHOLD = 20               

DXL_OPERATING_MODE = 1

DXL_DESIRED_SPEED = 0

# Les moteurs sont en position neutre à la position de 180 degrés
MOTOR_OFFSET = 180 

# indices des moteurs
DXL_IDS = [0,1,2,3]
DXL_IDS2 = [0,1,2]


# Valeurs numériques des paramètres du robot (voir image avec les paramètres DH)

# longueurs en cm
L0 = 3.5
L1 = 3
L2 = 11.1
L3 = 12.7
L4 = 2.3

# Angles en degrés entre parenthèses
mu  = (77.47)*2*pi/360
nu = -mu

# Paramètre de gain de la boucle de commande

Lambda = 0.05


# Consigne en position (en cm) depuis le repère initial à la base du robot (voir photo DHFINAL)

xC = 12.
yC = 0.
zC = 17.

consigne = np.array([[xC],[yC],[zC]])

# Position de l'effecteur

x = 0.
y = 0.
z = 0.

# coordonées articulaires à remplir avec les mesures des codeurs
q = np.array([[0.],[0.],[0.],[0.]])

# Vecteur position de l'effecteur
r = np.array([[x],[y],[z]])

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


for id in DXL_IDS:


    
     # SET VELOCITY MODE
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, DXL_OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel " + str(id) + " has been successfully set to velocity mode")


    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel " + str(id) + " has been successfully connected")


# Boucle de commande du robot

print("Appuyez sur une touche pour quitter")

done = False


while not done:


    # La boucle se termine si une touche est pressée
    if msvcrt.kbhit():
        done = True



    for id in DXL_IDS:
        # Lecture de la position courante des moteurs
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        q[id][0] = (getDegreesFromRelativePos(getRelativePos(dxl_present_position - MOTOR_OFFSET/360*4095)))*2*pi/360


    # print("\n")
    # print("q:")
    # print(q)
    # print("\n")
        
    current_J = J2(q,L0,L1,L2,L3,L4,mu,nu)
    current_T = T(q,L0,L1,L2,L3,L4,mu,nu)

    newJ = [[0,0,0],[0,0,0],[0,0,0]]

    for i in range(3):
        for j in range(3):
            newJ[i][j] = current_J[i][j]


    r = current_T[0:3, 3:4]

    #DEBUG

    print("\n")
    print("T:")
    print(current_T)
    print("\n")

    print("\n")
    print("r:")
    print(r)
    print("\n")
    print("q:")
    print(q)
    print("\n")
    print("J:")
    print(newJ)

    erreur = consigne - r

    vitesseConsigne = Lambda*erreur

    print("\n")
    print("erreur:")
    print(erreur)
    print("\n")

    print("\n")
    print("vitesseConsigne:")
    print(vitesseConsigne)
    print("\n")
    

    # Il n'est pas possible que J ai une ligne nulle !

    #JpseudoInv = Jnum.T*((Jnum*Jnum.T)**(-1))
    JpseudoInv = np.linalg.pinv(newJ)

    print("\n")
    print("jPseudoinv")
    print(JpseudoInv)
    print("\n")



    # Consigne en vitesse aux joints
    qC = np.dot(JpseudoInv,vitesseConsigne)


    # print("\n")
    # print("qC:")
    # print(qC)
    # print("\n")


    currentT0 = T0(q,L0,L1,L2,L3,L4,mu,nu)

    currentT1 = T1(q,L0,L1,L2,L3,L4,mu,nu)

    currentT2 = T2(q,L0,L1,L2,L3,L4,mu,nu)

    currentT3 = T3(q,L0,L1,L2,L3,L4,mu,nu)

    

    print("\n")
    print("T0:")
    print(currentT0)
    print("\n")

    print("\n")
    print("T1:")
    print(currentT1)
    print("\n")


    print("\n")
    print("T2:")
    print(currentT2)
    print("\n")

    print("\n")
    print("T3:")
    print(currentT3)
    print("\n")

    T01 = np.matmul(currentT0,currentT1)

    T012 = np.matmul(T01,currentT2)

    T0123 = np.matmul(T012,currentT3)


    print("\n")
    print("T01:")
    print(T01)
    print("\n")

    print("\n")
    print("T012:")
    print(T012)
    print("\n")

    print("\n")
    print("T0123:")
    print(T0123)
    print("\n")




    for id in DXL_IDS2:
        # Write goal speed
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, radSecToDx(qC[id][0]))
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_VELOCITY, radSecToDx(qC[id][0]))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


for id in DXL_IDS:
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    print("Dynamixel " + str(id) + " has been successfully disconnected")

# Close port
portHandler.closePort()