import time
from MPU9250 import MPU9250
from lps22hbtr import LPS22HB

# Initialisation des constantes
mpu = MPU9250()
lps = LPS22HB()
p1 = 1013.25
z1 = 0
min_time = 5000
flt_time = 15000

# Fonction d'allumage du buzzer d'indication
def start_buzzer():
    pass

# Activation du parachute
def parachute():
    pass

# Donne l'accélération totale
def accel(ax,ay,az):
    return sqrt(ax**2 + ay**2 + az**2)

# Renvoie l'altitude à partir de la pression
def alt(press):
    return 44330.8*(1-(press/p1)**0.190289)

# Teste si l'altitude décroît sur 5 valeurs (donne le voisinage de l'alt max)
def max_alt(tab):
    h0 = tab[0]
    h1 = tab[1]
    i = 1
    while h0 > h1 && i < 5:
        h0 = tab[i]
        h1 = tab[i+1]
        i+=1
    return i=4

# Donne la stabilité de l'altitude sur 10 points (définie comme la variatio entre deux moyennes)
def alt_cst(tab):
    avg1 = 0
    avg2 = 0
    for i in range(5):
        avg1 = avg1 + tab[i]/5
    for i in range(5,10):
        avg2 = avg2 + tab[i]/5
    return abs(avg1 - avg2) <= 1

# Gère la fusée pendant l'acquisition des données
def dataWrite(ax,ay,az,p,y,r):
    file = open("data.csv", "w")
    t1 = time.ticks_ms()
    dt = 0
    i = 0
    alts = [0,0,0,0,0,0,0,0,0,0]	# Initialisation des altitudes avec le capteur de pression
    for j in range(10):
        press = lps.getData()[0]
        alts[i] = alt(press)
        time.sleep_ms(200)
        
    while True:
        t2 = time.ticks_ms()
        dt = time.ticks_diff(t2,t1) # Dater la ligne
        
        [ax,ay,az,p,y,r] = mpu.getData() # Données de l'IMU
        press = lps.getData()[0]
        alt = alt(press)
        alts[i%10] = alt # Altitude
        
        # Écriture de la ligne des données
        line = str(dt) +","+ str(alt) +","+ str(ax)+","+ str(ay)+","+ str(az) +","+ str(p) +","+ str(y)+","+ str(r)
        
        file.write(line)
        #print(line)		debug 
        
        if (dt > min_time) && (dt < flt_time || max_alt(alts)): # Vérifie les conds d'ouverture du parachute
            parachute()
        
        if i%10 = 0 && alt_cst(alts): # Fait quitter la boucle si on est au sol
            pass
        i = i+1
        time.sleep_ms(500)
    file.close()

# Boucle principale d'avant-vol
while True:
    [ax,ay,az,p,y,r] = mpu.getData()
    accel = accel(ax,ay,az)
    if accel > 15: # Mise en place de l'acquisition 
            dataWrite(ax,ay,az,p,y,r)
            pass