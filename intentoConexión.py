from machine import Pin, UART
import time

# Configuración de UART (puerto serie)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))  # UART0 usando GPIO0 para TX y GPIO1 para RX

# Configuración de un LED como indicador
led = Pin(25, Pin.OUT)  # LED en la Raspberry Pi Pico

# Función para enviar datos vía Bluetooth
def enviar_datos(dato):
    uart.write(dato)
    print("Enviado:", dato)

# Función para recibir datos vía Bluetooth
def recibir_datos():
    if uart.any():
        datos = uart.read().decode('utf-8')  # Leemos y decodificamos los datos
        print("Recibido:", datos)
        return datos
    return None

# Bucle principal
while True:
    led.toggle()  # Parpadea el LED cada vez que corre el bucle
    
    # Envía un mensaje cada 5 segundos
    enviar_datos("Hola desde Raspberry Pi Pico\n")
    
    # Lee datos entrantes
    mensaje = recibir_datos()
    if mensaje:
        if "ON" in mensaje:
            led.value(1)  # Enciende el LED si recibe "ON"
            enviar_datos("LED encendido\n")
        elif "OFF" in mensaje:
            led.value(0)  # Apaga el LED si recibe "OFF"
            enviar_datos("LED apagado\n")
    
    time.sleep(5)  # Espera 5 segundos antes de volver a enviar
