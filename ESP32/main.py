# ==============================================
# Analizador de vibraciones distribuido
# ESP32 (MicroPython)
#
# - Lee tramas "VIB,..." por UART desde RP2040
# - Envía comandos de velocidad al RP2040
# - Crea un AP Wi-Fi
# - Servidor HTTP
# ==============================================

import network
import usocket
import machine
import utime
import _thread
import ujson

# ##################################
# Configuración UART
# ##################################

# ID de UART y parámetros
UART_ID   = 2       
UART_BAUD = 115200

# Pines TX/RX del ESP32 que van conectados al RP2040
UART_TX_PIN = 17       
UART_RX_PIN = 16       

# ##################################
# Variables globales
# ##################################

uart   = None         # Dispositivo UART hacia RP2040
ap     = None         # Punto de acceso WiFi
s_web  = None         # Socket del servidor web

lkUART = _thread.allocate_lock()  # Lock para proteger acceso a UART
lkVib  = _thread.allocate_lock()  # Lock para proteger estructura de mediciones

# Estructura con la última medición recibida desde el RP2040
ultima_vib = {
    "frec_pico":   0.0,
    "amp_pico":    0.0,
    "rms_bajo":    0.0,
    "rms_medio":   0.0,
    "rms_alto":    0.0, 
    "t_fft_ms":    0.0, # tiempo de la FFT en ms
    "ts_ms":       0,   # tiempo en ms desde la última actualización
    "vel_motor":   0    # último valor de velocidad enviado al RP2040 
}

# ##################################
# Configuración de UART
# ##################################

def setupUART():
    global uart

    tx = machine.Pin(UART_TX_PIN, machine.Pin.OUT)
    rx = machine.Pin(UART_RX_PIN, machine.Pin.IN)

    uart = machine.UART(
        UART_ID,
        baudrate = UART_BAUD,
        tx = UART_TX_PIN,   
        rx = UART_RX_PIN,
        timeout = 100,      # Tiempo máximo (ms) que esperará lectura completa
        timeout_char = 10   # Tiempo máximo entre caracteres
    )
    print("UART ESP32 lista @", UART_BAUD)
# end def

# ##################################
# Configuración WiFi en modo AP
# ##################################

def setupWiFiAP():
    global ap
    ap = network.WLAN(network.AP_IF)
    ap.active(False)

    # SSID y contraseña del AP
    ap.config(
        ssid     = "VIB-ESP32",
        security = network.AUTH_WPA2_PSK,
        key      = "12345678",
    )

    ap.active(True)

    ap.ifconfig(('192.168.1.1', '255.255.255.0',
                 '192.168.1.1', '192.168.1.1'))

    # Nombre 
    network.hostname('Motor')
    print("AP configurado (SSID: VIB-ESP32, IP: 192.168.1.1)")
# end def

# ##################################
# Configuración de sockets (servidor HTTP)
# ##################################

def setupSockets():
    global s_web

    # Esperar a que el AP esté activo
    while not ap.active():
        utime.sleep(0.25)
    print("AP inicializado correctamente")

    s_web = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    ip = ap.ifconfig()[0]
    s_web.bind((ip, 80))
    s_web.listen(1)
    print(f"Servidor HTTP escuchando en {ip}:80")
# end def

# ##################################
# Envío de velocidad al RP2040
# ##################################

def enviar_velocidad_motor(vel):

    # Se envía un comando de velocidad al RP2040 en el formato

    try:
        vel_int = int(vel)
    except:
        return False

    if vel_int < 0:
        vel_int = 0
    if vel_int > 100:
        vel_int = 100

    linea = "VEL,{}\n".format(vel_int)

    # Región crítica UART
    with lkUART:
        try:
            uart.write(linea.encode("utf-8"))
        except:
            return False

    # Se guarda el último valor enviado 
    with lkVib:
        ultima_vib["vel_motor"] = vel_int

    print("UART ESP32 =>", linea.strip())
    return True
# end def

# ##################################
# Tratamiento de líneas VIB de RP2040
# ##################################

def parse_vib_linea(s):
    """
    Esperamos líneas del tipo:
        VIB,frec_pico,amp_pico,rms_bajo,rms_medio,rms_alto,t_fft
    """
    if not s.startswith("VIB,"):
        return

    partes = s.split(",")
    if len(partes) != 7:
        return

    try:
        frec_pico  = float(partes[1])
        amp_pico   = float(partes[2])
        rms_bajo   = float(partes[3])
        rms_medio  = float(partes[4])
        rms_alto   = float(partes[5])
        t_fft_ms   = float(partes[6])
    except:
        return

    ts = utime.ticks_ms()

    with lkVib:
        ultima_vib["frec_pico"]  = frec_pico
        ultima_vib["amp_pico"]   = amp_pico
        ultima_vib["rms_bajo"]   = rms_bajo
        ultima_vib["rms_medio"]  = rms_medio
        ultima_vib["rms_alto"]   = rms_alto
        ultima_vib["t_fft_ms"]   = t_fft_ms
        ultima_vib["ts_ms"]      = ts
# end def


def uartReaderTask(arg):
    print("UART reader task: corriendo")
    while True:
        utime.sleep(0.001)

        # Región crítica UART para lectura
        with lkUART:
            linea = uart.readline()

        if not linea:
            continue

        try:
            s = linea.decode("utf-8").strip()
        except:
            continue

        if not s:
            continue

        print(f"RP2040: {s}")
        parse_vib_linea(s)
# end def

# ##################################
# Utilidades para parsear URI 
# ##################################

def indexOf(s, c, offset=0):
    for i in range(offset, len(s)):
        if s[i] == c:
            return i
    return -1
# end def


def fetchUriParams(sreq):
    print(f"fetchUriParams over {sreq}")
    up = {}

    qmpos = indexOf(sreq, '?')
    if qmpos == -1:
        print("up:", up)
        return up

    parts = sreq[qmpos+1:].split('&')
    for p in parts:
        pparts = p.split('=', 2)
        key = pparts[0]
        value = pparts[1] if len(pparts) > 1 else None
        up[key] = value

    print("up:", up)
    return up
# end def

# ##################################
# Rutas auxiliares: /data y /vel
# ##################################

def serveData():
    with lkVib:
        data = dict(ultima_vib)

    body = ujson.dumps(data)
    payload = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Cache-Control: no-cache\r\n\r\n"
        + body
    )
    return payload
# end def


def serveVel(sreq):
    up = fetchUriParams(sreq)

    if "vel" not in up or up["vel"] is None:
        return (
            "HTTP/1.1 400 Bad Request\r\n"
            "Content-Type: text/plain\r\n\r\n"
            "BAD REQUEST\n"
        )

    ok = enviar_velocidad_motor(up["vel"])

    if ok:
        return (
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n\r\n"
            "OK\n"
        )
    else:
        return (
            "HTTP/1.1 500 Internal Server Error\r\n"
            "Content-Type: text/plain\r\n\r\n"
            "ERROR\n"
        )
# end def

# ##################################
# Generación de la página principal
# ##################################

def webpage():
    try:
        with open("index.html", "r") as f:
            body = f.read()
    except:
        return (
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n\r\n"
            "Page not found\n"
        )

    with lkVib:
        data = dict(ultima_vib)

    body = body.replace("<!--frec_pico-->",  f"{data['frec_pico']:.2f}",  1)
    body = body.replace("<!--amp_pico-->",   f"{data['amp_pico']:.2f}",   1)
    body = body.replace("<!--rms_bajo-->",   f"{data['rms_bajo']:.2f}",   1)
    body = body.replace("<!--rms_medio-->",  f"{data['rms_medio']:.2f}",  1)
    body = body.replace("<!--rms_alto-->",   f"{data['rms_alto']:.2f}",   1)
    body = body.replace("<!--vel_motor-->",  str(data["vel_motor"]),      1)

    payload = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n\r\n"
        + body
    )
    return payload
# end def

# ##################################
# Servidor web 
# ##################################

def serveWeb():

    # Acepta una conexión, y en base al request respondemos

    cnn, addr = s_web.accept()
    print(f"Client connected from {str(addr)}")

    request = cnn.recv(1024)
    if not request:
        cnn.close()
        return

    try:
        sreq = request.decode("utf-8")
    except:
        cnn.close()
        return

    # Mostramos solo la primera línea del request
    first_line_end = indexOf(sreq, '\n')
    if first_line_end == -1:
        first_line = sreq
    else:
        first_line = sreq[:first_line_end]
    print(f"Request: {first_line}")

    if not sreq.startswith("GET "):
        cnn.send("HTTP/1.1 400 Bad Request\r\n\r\nBad request\n")
        cnn.close()
        return

    end_path = indexOf(sreq, " ", 5)
    if end_path == -1:
        end_path = len(sreq)
    sreq_path = sreq[4:end_path]  

    # Rutas
    if sreq_path.startswith("/?") or sreq_path.startswith("/index.htm"):
        payload = webpage()
    elif sreq_path == "/" or sreq_path.startswith("/index.html"):
        payload = webpage()
    elif sreq_path.startswith("/data"):
        payload = serveData()
    elif sreq_path.startswith("/vel"):
        payload = serveVel(sreq_path)
    else:
        payload = (
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n\r\n"
            "Not found\n"
        )

    cnn.send(payload)
    cnn.close()
# end def

# ##################################
# Main
# ##################################

def main():
    setupUART()
    setupWiFiAP()
    setupSockets()

    # Declaramos el hilo para leer UART de forma concurrente (tramas VIB,...)
    _thread.start_new_thread(uartReaderTask, [None])

    # Ponemos el servidor HTTP en el hilo principal
    while True:
        serveWeb()
# end def

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import sys
        print("--- Excepción en main() ---")
        sys.print_exception(e)
        print("---------------------------")


