# =============================================================================
# test_sensori.py  -  Controlla che i dati delle camere arrivino da CARLA
# =============================================================================
# Usalo quando 01_registra.py si blocca dopo "Auto nel traffico".
# Non carica mappe e non crea traffico: una sola camera piccola, 3 prove.
# Avvio:  python3 test_sensori.py
# =============================================================================

import queue
import socket

import carla

HOST = "host.docker.internal"    # lo STESSO HOST che hai messo in 01_registra.py


# --- PROVA 1: le due "porte" di CARLA sono raggiungibili? -------------------
# CARLA usa DUE porte: 2000 per i comandi, 2001 per mandare le immagini.
print("PROVA 1: porte di CARLA")
for porta in [2000, 2001]:
    try:
        s = socket.create_connection((HOST, porta), timeout=5)
        s.close()
        print("  porta", porta, ": OK")
    except OSError as errore:
        print("  porta", porta, ": NON raggiungibile ->", errore)

# --- PROVA 2: ci colleghiamo? -----------------------------------------------
print("PROVA 2: collegamento")
client = carla.Client(HOST, 2000)
client.set_timeout(30)
world = client.get_world()       # la mappa gia' aperta, non ne carichiamo un'altra
print("  OK, mappa aperta:", world.get_map().name)

# --- PROVA 3: una camera piccola manda le immagini? -------------------------
print("PROVA 3: una camera 320x180")
impostazioni = world.get_settings()
impostazioni.synchronous_mode = True
impostazioni.fixed_delta_seconds = 0.05
world.apply_settings(impostazioni)

bp = world.get_blueprint_library().find("sensor.camera.rgb")
bp.set_attribute("image_size_x", "320")
bp.set_attribute("image_size_y", "180")
posizione = world.get_spectator().get_transform()     # dove sta guardando la finestra di CARLA
camera = world.spawn_actor(bp, posizione)
coda = queue.Queue()
camera.listen(coda.put)

try:
    arrivate = 0
    for i in range(10):
        world.tick()
        try:
            coda.get(timeout=10)
            arrivate += 1
        except queue.Empty:
            print("  passo", i, ": nessuna immagine in 10 s")
            break
    print("  immagini arrivate:", arrivate, "su 10")
    if arrivate == 10:
        print("RISULTATO: i sensori funzionano. Il problema e' il peso: prova 01_registra.py")
        print("           con USA_SEGMENTAZIONE = False e USA_LIDAR = False.")
    else:
        print("RISULTATO: le immagini non arrivano. Se la PROVA 1 dice che la porta 2001")
        print("           NON e' raggiungibile, il problema e' la porta 2001 (vedi LEGGIMI).")
finally:
    camera.stop()
    camera.destroy()
    impostazioni.synchronous_mode = False
    impostazioni.fixed_delta_seconds = None
    world.apply_settings(impostazioni)
