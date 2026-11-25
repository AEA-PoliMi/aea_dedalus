# =============================================================================
# 04_vista_strada.py  -  Mette la vista di CARLA ad altezza strada, ferma
# =============================================================================
# A cosa serve: il PC non riesce a registrare i sensori, ma riesce a mostrare
# CARLA. Questo script mette la "telecamera" della finestra di CARLA (lo
# spettatore) in un punto della strada, all'altezza della camera dell'R2,
# guardando avanti, e fa passare il traffico. Tu registri lo schermo
# (su Windows: Win + Alt + R) e il video lo usiamo con YOLO su Kaggle.
#
# Non crea sensori: e' leggero come muovere la vista a mano.
# Avvio:  python3 04_vista_strada.py      Per fermare: Ctrl + C
# =============================================================================

import random
import time

import carla

# ----------------------------- IMPOSTAZIONI ----------------------------------
HOST = "host.docker.internal"   # come in 01_registra.py
MAPPA = "Town02"                # Town02 e' leggera; prova Town10HD_Opt se regge
CARICA_MAPPA = False            # True = carica MAPPA (lento); False = usa quella aperta
PUNTO = 10                      # quale punto di partenza della mappa usare (prova numeri diversi)
ALTEZZA = 1.5                   # metri da terra, come la camera sull'auto
INCLINAZIONE = -8               # gradi verso il basso, come in 01_registra.py
NUMERO_AUTO = 20                # auto del traffico
IGNORA_SEMAFORI = True          # cosi' le auto passano di continuo
DURATA_SECONDI = 90             # dopo quanto si ferma da solo
# -----------------------------------------------------------------------------

client = carla.Client(HOST, 2000)
client.set_timeout(120)
if CARICA_MAPPA:
    print("Carico la mappa", MAPPA, "...")
    world = client.load_world(MAPPA)
else:
    world = client.get_world()
print("Mappa:", world.get_map().name)

# Modalita' normale (non sincrona): CARLA va avanti da solo, come quando lo guardi
impostazioni = world.get_settings()
impostazioni.synchronous_mode = False
impostazioni.fixed_delta_seconds = None
world.apply_settings(impostazioni)

traffico = client.get_trafficmanager(8000)
traffico.set_synchronous_mode(False)

punti = world.get_map().get_spawn_points()
punto = punti[PUNTO % len(punti)]

# 1) la vista: nel punto scelto, ad altezza camera, guardando lungo la strada
vista = carla.Transform(
    carla.Location(x=punto.location.x, y=punto.location.y, z=punto.location.z + ALTEZZA),
    carla.Rotation(pitch=INCLINAZIONE, yaw=punto.rotation.yaw))
world.get_spectator().set_transform(vista)
print("Vista messa nel punto", PUNTO % len(punti), "ad altezza", ALTEZZA, "m")

# 2) il traffico (non nel punto della vista, per non nascerci sopra)
altri = punti[:PUNTO % len(punti)] + punti[PUNTO % len(punti) + 1:]
random.shuffle(altri)
modelli = world.get_blueprint_library().filter("vehicle.*")
auto = []
try:
    for p in altri[:NUMERO_AUTO]:
        a = world.try_spawn_actor(random.choice(modelli), p)
        if a is not None:
            a.set_autopilot(True, 8000)
            if IGNORA_SEMAFORI:
                traffico.ignore_lights_percentage(a, 100)
            auto.append(a)
    print("Auto nel traffico:", len(auto))
    print("Ora registra lo schermo. Mi fermo da solo tra", DURATA_SECONDI, "secondi (o Ctrl + C).")

    inizio = time.time()
    while time.time() - inizio < DURATA_SECONDI:
        world.get_spectator().set_transform(vista)   # se la vista si sposta, la rimettiamo
        time.sleep(0.5)
finally:
    print("Cancello le auto...")
    for a in auto:
        a.destroy()
    print("Fatto.")
