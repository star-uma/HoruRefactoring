"""
horu_control.py
===============
Sustituto del modelo Simulink del PC para el robot Horu.
Lee el mando Xbox via Bluetooth y envia datagramas UDP al Arduino MKR WiFi 1010.

Datagrama (18 bytes, 9 x int16 little-endian):
  [0] id              -> siempre 1 (identifica al PC como emisor)
  [1] action_left_train  -> velocidad tren izquierdo  (dps * 100 / GAIN)
  [2] action_right_train -> velocidad tren derecho    (dps * 100 / GAIN)
  [3] o0  \
  [4] o1   > posiciones/sentidos para ESP32 (modos 2-4)
  [5] o2   |
  [6] o3  /
  [7] mode     -> modo de funcionamiento activo (1..5)
  [8] code_error -> 0 = sin error

Requisitos:
  pip install pygame

Uso:
  python horu_control.py
"""

import pygame
import socket
import struct
import time
import sys

# ─── CONFIGURACION ────────────────────────────────────────────────────────────

ARDUINO_IP   = "192.168.10.101"
ARDUINO_PORT = 8885
SEND_RATE_HZ = 50          # frecuencia de envio (paquetes por segundo)
MAX_SPEED_DPS = 40         # velocidad maxima en grados/segundo (igual que Simulink)
GAIN          = 30         # ganancia del joystick (rango [-1,1] -> [-30,30] dps)

# Mapeo de ejes del mando Xbox (puede variar segun driver/SO)
# Comprueba con --test si los valores no responden bien
AXIS_LEFT_Y  = 1   # Joystick izquierdo vertical  (adelante/atras)
AXIS_RIGHT_X = 2   # Joystick derecho  horizontal (giro)

# Mapeo de botones Xbox
BTN_RB    = 5   # Bumper derecho  -> cambiar modo
BTN_A     = 0   # Boton A         -> modo 2: posicion 225 deg
BTN_B     = 1   # Boton B         -> modo 2: posicion 180 deg
BTN_X     = 2   # Boton X         -> modo 2: posicion 135 deg
BTN_Y     = 3   # Boton Y         -> modo 2: posicion 90 deg
BTN_START = 7   # Start           -> modo 5: nivelado automatico (mantener)

# Ejes de trigger (pygame los expone como ejes analogicos -1..1)
AXIS_TRIGGER_L = 4   # Trigger izquierdo
AXIS_TRIGGER_R = 5   # Trigger derecho

# HAT (cruceta digital)
HAT_INDEX = 0   # Hat 0 = cruceta principal

# Deadzone del joystick (ignorar valores menores a este umbral)
DEADZONE = 0.08

# ─── ESTADO GLOBAL ────────────────────────────────────────────────────────────

modo_actual = 1          # arranca en modo traccion
rb_anterior = False      # para detectar flanco ascendente del RB
last_Y      = [180, 180, 180, 180]  # ultima posicion objetivo modo 2

# ─── UTILIDADES ───────────────────────────────────────────────────────────────

def aplicar_deadzone(valor: float, zona: float = DEADZONE) -> float:
    """Elimina el ruido de reposo del joystick."""
    if abs(valor) < zona:
        return 0.0
    # Reescala para que empiece en 0 justo al salir de la zona muerta
    signo = 1.0 if valor > 0 else -1.0
    return signo * (abs(valor) - zona) / (1.0 - zona)

def saturar(valor: float, maximo: float) -> float:
    return max(-maximo, min(maximo, valor))

def clamp_int16(valor: int) -> int:
    return max(-32768, min(32767, valor))

def leer_hat_como_vector(joy: pygame.joystick.Joystick) -> list:
    """Devuelve [UP, DOWN, LEFT, RIGHT] como 0/1 segun la cruceta."""
    if joy.get_numhats() == 0:
        return [0, 0, 0, 0]
    hx, hy = joy.get_hat(HAT_INDEX)
    return [
        1 if hy ==  1 else 0,   # UP
        1 if hy == -1 else 0,   # DOWN
        1 if hx == -1 else 0,   # LEFT
        1 if hx ==  1 else 0,   # RIGHT
    ]

def calcular_traccion(joy: pygame.joystick.Joystick) -> tuple:
    """
    Modo 1: calcula action_left_train y action_right_train.
    Replica exactamente la logica del subsistema Simulink del PC:
      - Joystick izq Y  -> velocidad lineal
      - Joystick der X  -> diferencia (giro)
      - Ganancia x30, saturacion +-40 dps, escalado x100 para CAN
    """
    ly = aplicar_deadzone(-joy.get_axis(AXIS_LEFT_Y))   # invertir: arriba = positivo
    rx = aplicar_deadzone( joy.get_axis(AXIS_RIGHT_X))

    vel_base = ly * GAIN
    diferencial = rx * GAIN

    vel_izq = saturar(vel_base - diferencial, MAX_SPEED_DPS)
    vel_der = saturar(vel_base + diferencial, MAX_SPEED_DPS)

    # El Arduino aplica *-1 al tren izquierdo internamente,
    # asi que aqui enviamos el valor sin invertir (igual que Simulink)
    action_left  = clamp_int16(int(vel_izq * 100))
    action_right = clamp_int16(int(vel_der * 100))

    return action_left, action_right

def calcular_modo_2(joy: pygame.joystick.Joystick) -> list:
    """
    Modo 2: posicion absoluta de orugas via botones A/B/X/Y.
    Devuelve vector Y [o0,o1,o2,o3] con el angulo objetivo en grados.
    """
    global last_Y
    botones = [joy.get_button(BTN_A), joy.get_button(BTN_B),
               joy.get_button(BTN_X), joy.get_button(BTN_Y)]
    angulos = [225, 180, 135, 90]
    for i, pulsado in enumerate(botones):
        if pulsado:
            last_Y = [angulos[i]] * 4
            break
    return list(last_Y)

def calcular_modo_3(joy: pygame.joystick.Joystick) -> list:
    """
    Modo 3: inclinacion manual de las cuatro orugas a la vez.
    Devuelve vector Y [o0..o3] con sentidos de giro -1/0/1.
    """
    data_0 = leer_hat_como_vector(joy)
    if   data_0 == [1, 0, 0, 0]:   return [-1,  1, -1,  1]   # inclinar adelante
    elif data_0 == [0, 1, 0, 0]:   return [ 1, -1,  1, -1]   # inclinar atras
    elif data_0 == [0, 0, 1, 0]:   return [ 1,  1, -1, -1]   # inclinar izquierda
    elif data_0 == [0, 0, 0, 1]:   return [-1, -1,  1,  1]   # inclinar derecha
    else:                           return [ 0,  0,  0,  0]

def calcular_modo_4(joy: pygame.joystick.Joystick) -> list:
    """
    Modo 4: control incremental por pares de orugas.
    Cruceta selecciona el par, triggers controlan sentido.
    """
    data_0  = leer_hat_como_vector(joy)
    trig_r  = joy.get_axis(AXIS_TRIGGER_R) > 0.3   # trigger derecho = bajar
    trig_l  = joy.get_axis(AXIS_TRIGGER_L) > 0.3   # trigger izquierdo = subir

    sentido = 1 if trig_r else (-1 if trig_l else 0)
    if sentido == 0:
        return [0, 0, 0, 0]

    if   data_0 == [1, 0, 0, 0]:   return [-sentido,  sentido,  0,  0]   # par delantero
    elif data_0 == [0, 1, 0, 0]:   return [ 0,  0,  sentido, -sentido]   # par trasero
    elif data_0 == [0, 0, 1, 0]:   return [ 0,  sentido,  0, -sentido]   # par izquierdo
    elif data_0 == [0, 0, 0, 1]:   return [-sentido,  0,  sentido,  0]   # par derecho
    else:                           return [0, 0, 0, 0]

def construir_datagrama(modo: int, joy: pygame.joystick.Joystick) -> bytes:
    """
    Ensambla el datagrama de 18 bytes (9 x int16 LE) para el Arduino.
    """
    id_pc = 1
    action_l, action_r = 0, 0
    Y = [0, 0, 0, 0]
    code_error = 0

    if modo == 1:
        action_l, action_r = calcular_traccion(joy)
    elif modo == 2:
        Y = calcular_modo_2(joy)
    elif modo == 3:
        Y = calcular_modo_3(joy)
    elif modo == 4:
        Y = calcular_modo_4(joy)
    elif modo == 5:
        pass   # modo nivelado: el Arduino toma el control con la IMU

    # Formato: '<9h' = 9 shorts con signo, little-endian
    return struct.pack('<9h',
        id_pc,
        clamp_int16(action_l),
        clamp_int16(action_r),
        clamp_int16(int(Y[0])),
        clamp_int16(int(Y[1])),
        clamp_int16(int(Y[2])),
        clamp_int16(int(Y[3])),
        modo,
        code_error
    )

def mostrar_estado(modo: int, joy: pygame.joystick.Joystick, fps: float):
    """Imprime estado compacto en consola sin hacer scroll."""
    nombres_modo = {
        1: "Traccion (diferencial)",
        2: "Posicion absoluta",
        3: "Incremental x4",
        4: "Incremental x2",
        5: "Nivelado automatico",
    }
    ly = -joy.get_axis(AXIS_LEFT_Y)
    rx =  joy.get_axis(AXIS_RIGHT_X)
    a_l, a_r = calcular_traccion(joy) if modo == 1 else (0, 0)
    print(
        f"\r  Modo {modo}: {nombres_modo.get(modo,'?'):<26}"
        f"  LY={ly:+.2f} RX={rx:+.2f}"
        f"  L={a_l:+6d}  R={a_r:+6d}"
        f"  [{fps:.0f} pkt/s]   ",
        end="", flush=True
    )

# ─── MODO TEST (python horu_control.py --test) ────────────────────────────────

def modo_test():
    """Muestra los indices de ejes y botones para calibrar el mapeo."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No se detecta ningun mando. Conecta el Xbox via Bluetooth.")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Mando detectado: {joy.get_name()}")
    print(f"Ejes: {joy.get_numaxes()}  Botones: {joy.get_numbuttons()}  Hats: {joy.get_numhats()}")
    print("\nMueve ejes y pulsa botones para ver sus indices. Ctrl+C para salir.\n")
    try:
        while True:
            pygame.event.pump()
            ejes    = [f"A{i}={joy.get_axis(i):+.2f}" for i in range(joy.get_numaxes())]
            botones = [str(i) for i in range(joy.get_numbuttons()) if joy.get_button(i)]
            hats    = [f"H{i}={joy.get_hat(i)}" for i in range(joy.get_numhats())]
            print(f"\r  {' '.join(ejes)}  BTN:[{','.join(botones) or '-'}]  {' '.join(hats)}   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nTest finalizado.")

# ─── BUCLE PRINCIPAL ──────────────────────────────────────────────────────────

def main():
    global modo_actual, rb_anterior

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No se detecta ningun mando.")
        print("        Conecta el mando Xbox via Bluetooth y vuelve a ejecutar.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[OK] Mando detectado: {joy.get_name()}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[OK] Socket UDP listo -> {ARDUINO_IP}:{ARDUINO_PORT}")
    print(f"[OK] Frecuencia de envio: {SEND_RATE_HZ} paquetes/s")
    print("\nControles:")
    print("  Joystick IZQ vertical  -> avanzar/retroceder  (Modo 1)")
    print("  Joystick DER horizontal-> girar               (Modo 1)")
    print("  RB                     -> cambiar modo (1->2->3->4->5->1)")
    print("  A/B/X/Y                -> posicion absoluta   (Modo 2)")
    print("  Cruceta + Triggers     -> control orugas      (Modos 3/4)")
    print("  Start (mantener)       -> nivelado automatico (Modo 5)")
    print("\nCtrl+C para salir.\n")

    intervalo  = 1.0 / SEND_RATE_HZ
    ultimo_env = 0.0

    try:
        while True:
            pygame.event.pump()

            # ── Cambio de modo con RB (flanco ascendente) ──────────────────
            rb_actual = bool(joy.get_button(BTN_RB))
            if rb_actual and not rb_anterior:
                modo_actual = (modo_actual % 5) + 1   # cicla 1->2->3->4->5->1
                print(f"\n  >> Modo cambiado a {modo_actual}")
            rb_anterior = rb_actual

            # ── Modo 5: activo solo mientras se mantiene START ──────────────
            modo_envio = modo_actual
            if joy.get_button(BTN_START) and modo_actual == 4:
                modo_envio = 5

            # ── Construir y enviar datagrama a la frecuencia configurada ────
            ahora = time.monotonic()
            if ahora - ultimo_env >= intervalo:
                datos = construir_datagrama(modo_envio, joy)
                sock.sendto(datos, (ARDUINO_IP, ARDUINO_PORT))
                ultimo_env = ahora
                fps = 1.0 / intervalo
                mostrar_estado(modo_envio, joy, fps)

            # Cede tiempo al SO sin bloquear demasiado
            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n\n[INFO] Saliendo... enviando parada de seguridad.")
        # Envia un paquete de parada (velocidad 0, modo 1)
        parada = struct.pack('<9h', 1, 0, 0, 0, 0, 0, 0, 1, 0)
        for _ in range(5):   # varios intentos por si se pierde algun UDP
            sock.sendto(parada, (ARDUINO_IP, ARDUINO_PORT))
            time.sleep(0.02)
        sock.close()
        pygame.quit()
        print("[OK] Parada enviada. Programa terminado.")

# ─── PUNTO DE ENTRADA ─────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        modo_test()
    else:
        main()