import time 
import math
from machine import Pin, PWM

#
def anguloRecorrido(x, y):
    # Calcula el ángulo en radianes usando atan2
    theta = math.atan2(y, x)
    
    # Calcula el recorrido (magnitud) usando el teorema de Pitágoras
    recorrido = math.sqrt((y ** 2) + (x ** 2))
    
    # Devuelve ambos valores
    return theta, recorrido



def velocidadesRuedas(V, W):
    # Modelo cinemático de velocidades en las ruedas
    fi1 = -V - W
    fi2 = V - W
    
    return fi1, fi2

def controlP(et, ed):
    # Fuzzificación de las variables
    # Función ng (saturación izquierda)
    alpha_ng = -0.3
    beta_ng = 0
    if et <= alpha_ng:
        ng = 1
    elif alpha_ng < et <= beta_ng:
        ng = (et - beta_ng) / (alpha_ng - beta_ng)
    else:
        ng = 0

    # Función z triangular (et)
    alpha_z = -0.5
    beta_z = 0
    gama_z = 0.5
    if alpha_z <= et <= beta_z:
        z = (et - alpha_z) / (beta_z - alpha_z)
    elif beta_z < et <= gama_z:
        z = (gama_z - et) / (gama_z - beta_z)
    else:
        z = 0

    # Función pg (saturación derecha)
    alpha_pg = 0
    beta_pg = 0.3
    if et <= alpha_pg:
        pg = 0
    elif alpha_pg < et <= beta_pg:
        pg = (et - alpha_pg) / (beta_pg - alpha_pg)
    else:
        pg = 1

    # Función n (Saturación Izquierda) para ed
    alpha_n = -0.25
    beta_n = 0
    if ed <= alpha_n:
        n = 1
    elif alpha_n < ed <= beta_n:
        n = (ed - beta_n) / (alpha_n - beta_n)
    else:
        n = 0

    # Función z triangular (ed)
    alpha_zd = -0.15
    beta_zd = 0
    gama_zd = 0.15
    if alpha_zd <= ed <= beta_zd:
        zd = (ed - alpha_zd) / (beta_zd - alpha_zd)
    elif beta_zd < ed <= gama_zd:
        zd = (gama_zd - ed) / (gama_zd - beta_zd)
    else:
        zd = 0

    # Función p (Saturación Derecha) para ed
    alpha_p = 0
    beta_p = 0.25
    if ed <= alpha_p:
        p = 0
    elif alpha_p < ed <= beta_p:
        p = (ed - alpha_p) / (beta_p - alpha_p)
    else:
        p = 1

    # Evaluación de reglas
    # Regla 1: if et es ng entonces V es z y W es n
    Vz = z  # Regla 1
    Wn = ng
    # Regla 2: If et is z and ed is z then V=z y W=z
    Vz = max(Vz, min(z, zd))  # Regla 2
    Wz = min(z, zd)
    # Regla 3: If et is pg then V is z and W is p 
    Vz = min(Vz, pg)  # Regla 3
    Wp = pg
    # Regla 4: If et is z and ed is n then V is n and W is z 
    Vn = min(z, n)  # Regla 4
    Wz = min(z, n)
    # Regla 5: If et is z and ed is p then V is p and W is z
    Vp = min(z, p)  # Regla 5
    Wz = max(Wz, min(z, p))

    # Defuzzificación
    beta_Vn = -0.5
    beta_Vz = 0
    beta_Vp = 0.5
    beta_Wn = -3.1416
    beta_Wz = 0
    beta_Wp = 3.1416

    # Evitar división por cero en la desborrosificación
    if (Vn + Vz + Vp) == 0:
        V = 0  # Valor por defecto
    else:
        V = (Vn * beta_Vn + Vz * beta_Vz + Vp * beta_Vp) / (Vn + Vz + Vp)

    if (Wn + Wz + Wp) == 0:
        W = 0  # Valor por defecto
    else:
        W = (Wn * beta_Wn + Wz * beta_Wz + Wp * beta_Wp) / (Wn + Wz + Wp)

    # Imprimir los valores de V y W
    print(f"Valor de V: {V}")
    print(f"Valor de W: {W}")

    return V, W


#-------------------------------------------------------------------------------------------------------------------------------------------
# Declarar los puertos para los encoders
A1 = Pin(17, Pin.IN, Pin.PULL_UP)  # Canal A del encoder de la rueda 1
B1 = Pin(16, Pin.IN, Pin.PULL_UP)  # Canal B del encoder de la rueda 1
A2 = Pin(11, Pin.IN, Pin.PULL_UP)  # Canal A del encoder de la rueda 2
B2 = Pin(10, Pin.IN, Pin.PULL_UP)  # Canal B del encoder de la rueda 2


# Variables para contar los pulsos
CO1 = 0  # Cuentas del encoder 1
CO2 = 0  # Cuentas del encoder 2

# Funciones de manejo del encoder 1
def A1_CHANGE(pin):
    global CO1
    if A1.value() == 1 and B1.value() == 0:
        CO1 += 1
    elif A1.value() == 0 and B1.value() == 1:
        CO1 += 1
    else:
        CO1 -= 1

def B1_CHANGE(pin):
    global CO1
    if A1.value() == 1 and B1.value() == 0:
        CO1 -= 1
    elif A1.value() == 0 and B1.value() == 1:
        CO1 -= 1
    else:
        CO1 += 1

# Funciones de manejo del encoder 2
def A2_CHANGE(pin):
    global CO2
    if A2.value() == 1 and B2.value() == 0:
        CO2 += 1
    elif A2.value() == 0 and B2.value() == 1:
        CO2 += 1
    else:
        CO2 -= 1

def B2_CHANGE(pin):
    global CO2
    if A2.value() == 1 and B2.value() == 0:
        CO2 -= 1
    elif A2.value() == 0 and B2.value() == 1:
        CO2 -= 1
    else:
        CO2 += 1
        
# Configurar interrupciones para los pines del encoder 1
A1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=A1_CHANGE)
B1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=B1_CHANGE)

# Configurar interrupciones para los pines del encoder 2
A2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=A2_CHANGE)
B2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=B2_CHANGE)
    
def motorControl(speed, direction, direccion, pwm):
    pwm.freq(1000)  # Frecuencia de 600 Hz para el PWM de la rueda
    if direction == "adelante":
        direccion.value(0)  # Establece la dirección hacia adelante
    elif direction == "atras":
        direccion.value(1)  # Establece la dirección hacia atrás
    
    # Establecer la velocidad (0-100%)
    duty = int(speed * 65535 / 100)
    pwm.duty_u16(duty)
    return 0

#------------------------------------------------------------------------------------------------------------------------------------------------------------------
def avanzarAngulo(A1, B1, A2, B2, fi1, fi2, dir1, dir2, pwm1, pwm2):
    # Inicializamos el conteo de pulsos para ambas ruedas
    global CO1, CO2
    CO1 = 0  # Reiniciar el contador del encoder 1
    CO2 = 0  # Reiniciar el contador del encoder 2

    # Configuramos la velocidad de los motores
    motorControl(abs(fi1), "adelante" if fi1 > 0 else "atras", dir1, pwm1)  # Controlar motor 1
    motorControl(abs(fi2+1.5), "adelante" if fi2 > 0 else "atras", dir2, pwm2)  # Controlar motor 2
    # Calculamos la cantidad de pulsos necesarios para recorrer la distancia objetivo
    pulsos_objetivo = 200  # Ajustar según la configuración del encoder

    # Ciclo para contar pulsos hasta alcanzar la distancia objetivo
    while abs(CO1) < pulsos_objetivo and abs(CO2) < pulsos_objetivo:        
        # Verificamos si ya se alcanzó el límite de pulsos en ambas ruedas
        if abs(CO1) >= pulsos_objetivo or abs(CO2) >= pulsos_objetivo:
            break  # Si alguna rueda alcanza el límite, salimos del ciclo

        time.sleep_ms(10)  # Pausa breve para evitar sobrecargar el procesador

    # Detenemos ambos motores una vez alcanzados los pulsos objetivo
    motorControl(speed=0, direction="adelante", direccion=dir1, pwm=pwm1)
    motorControl(speed=0, direction="adelante", direccion=dir2, pwm=pwm2)

    return

def avanzarDistancia(A1, B1, A2, B2, fi1, fi2, dir1, dir2, pwm1, pwm2):
    # Inicializamos el conteo de pulsos para ambas ruedas
    global CO1, CO2
    CO1 = 0  # Reiniciar el contador del encoder 1
    CO2 = 0  # Reiniciar el contador del encoder 2

    # Configuramos la velocidad de los motores
    motorControl(abs(fi1), "adelante" if fi1 > 0 else "atras", dir1, pwm1)  # Controlar motor 1
    motorControl(abs(fi2+1.5), "adelante" if fi2 > 0 else "atras", dir2, pwm2)  # Controlar motor 2
    # Calculamos la cantidad de pulsos necesarios para recorrer la distancia objetivo
    pulsos_objetivo = 600  # Ajustar según la configuración del encoder

    # Ciclo para contar pulsos hasta alcanzar la distancia objetivo
    while abs(CO1) < pulsos_objetivo and abs(CO2) < pulsos_objetivo:        
        # Verificamos si ya se alcanzó el límite de pulsos en ambas ruedas
        if abs(CO1) >= pulsos_objetivo or abs(CO2) >= pulsos_objetivo:
            break  # Si alguna rueda alcanza el límite, salimos del ciclo

        time.sleep_ms(10)  # Pausa breve para evitar sobrecargar el procesador

    # Detenemos ambos motores una vez alcanzados los pulsos objetivo
    motorControl(speed=0, direction="adelante", direccion=dir1, pwm=pwm1)
    motorControl(speed=0, direction="adelante", direccion=dir2, pwm=pwm2)

    return 


def main():
    time.sleep(2)
    # Pines para el puente H (controlar la dirección de los motores)
    dir2 = Pin(13, Pin.OUT)  # Pin para la dirección de la rueda 1
    dir1 = Pin(15, Pin.OUT)  # Pin para la dirección de la rueda 2

    # Configuración del PWM para controlar la velocidad de las ruedas
    pwm1 = PWM(Pin(14))  # PWM para el motor de la rueda 1
    pwm2 = PWM(Pin(18))

    # Características del robot
    r = 0.0162  # radio de las ruedas
    long = 0.15  # Longitud entre ruedas
    perTotal = (long*2)*math.pi
    perEntreAngulo = perTotal/(2*math.pi)

    # Condiciones iniciales
    xi = 0
    yi = 0
    th = 0

    # Punto deseado
    xd = -0.25
    yd = -0.25

    # Calculo tomando en cuenta el punto inicial
    xd = xd - xi
    yd = yd - yi

    ready = 0

    thd, distancia = anguloRecorrido(xd, yd)  # Llamar a la función que calcula el ángulo y distancia
    totalRecorrido = perEntreAngulo * thd
    print(f"distancia {distancia:.2f}")
    
    if thd < 0:
        thd = 2 * math.pi + thd
        
    while thd > 0.04:
        print(f"th: {thd}")
        V, W = controlP(thd, 0)
        print(f"V: {V}, W: {W}")

        w1 = r/long
        w1 = w1 * W
        # Calcular velocidades en las ruedas
        fi1, fi2 = velocidadesRuedas(0, w1)
        
        if fi1 < 10:
            fi1 = 10
        if fi2 < 10:
            fi2 = 10
        avanzarAngulo(A1, B1, A2, B2, fi1, fi2, dir1, dir2, pwm1, pwm2)
        print(f"fi1: {fi1:.2f}")
        print(f"fi2 {fi2:.2f}")
        thd = thd - (((math.pi*(r*2))/6.3)/(long*math.pi))*2*math.pi
        time.sleep(0.1)
    print("Buen avance")
    
    while distancia > 0.1:
        print(f"th: {thd}")
        V, W = controlP(0, distancia)
        print(f"V: {V}, W: {W}")
        
        r1 = 1/r
        vR = V * r1

        # Calcular velocidades en las ruedas
        fi1, fi2 = velocidadesRuedas(vR, 0)
        
        print(f"V: {V}, W: {W}")
        print(f"fi1: {fi1:.2f}")
        print(f"fi2 {fi2:.2f}")
        print("1")
        avanzarDistancia(A1, B1, A2, B2, fi1, fi2, dir1, dir2, pwm1, pwm2)
        distancia = distancia - (((3.1416*(r*2)))/2.2)
        print(f"{distancia}")


        

if __name__ == "__main__":
    main()
    