# =============================================================================
# 03_demo_live.py  -  Demo DAL VIVO in CARLA: l'auto guida e YOLO riconosce
# =============================================================================
# E' 01_registra.py + 02_riconosci.py insieme, ma senza salvare i dati:
# mostra una finestra e salva un video per la presentazione.
#
# Come si usa:  avvia CARLA, poi   python 03_demo_live.py
# Premi ESC nella finestra per fermare prima.
# =============================================================================

import math
import queue
import random
import time

import carla
import cv2
import numpy as np
from ultralytics import YOLO

# ----------------------------------------------------------------------------
# IMPOSTAZIONI
# ----------------------------------------------------------------------------
MAPPA = "Town02"                # PC deboli (2-4 GB di VRAM): Town02. PC potenti: "Town10HD_Opt"
METEO = "ClearNoon"
PUNTO_PARTENZA = 60
SECONDI = 60                     # durata della demo
NUMERO_AUTO_TRAFFICO = 20
CARICA_MAPPA = True             # False se CARLA e' CONDIVISO (container del team): usa il mondo gia' aperto,
                                # perche' caricare una mappa resetta il mondo per TUTTI quelli collegati
HOST = "localhost"
LARGHEZZA, ALTEZZA, FOV = 960, 540, 90.0
MODELLO = "yolo26n.pt"
MOSTRA_FINESTRA = True           # False se lavori da remoto (container del team): niente finestra, solo il video
CLASSI_UTILI = {"person": "pedone", "bicycle": "veicolo", "car": "veicolo",
                "motorcycle": "veicolo", "bus": "veicolo", "truck": "veicolo"}
COLORI = {"veicolo": (0, 165, 255), "pedone": (255, 0, 255)}


# --- le stesse funzioni degli altri due file ---------------------------------
def crea_camera(world, blueprints, tipo, auto):
    bp = blueprints.find(tipo)
    bp.set_attribute("image_size_x", str(LARGHEZZA))
    bp.set_attribute("image_size_y", str(ALTEZZA))
    bp.set_attribute("fov", str(FOV))
    posizione = carla.Transform(carla.Location(x=1.5, z=1.5), carla.Rotation(pitch=-8))
    camera = world.spawn_actor(bp, posizione, attach_to=auto)
    coda = queue.Queue()
    camera.listen(coda.put)
    return camera, coda


def immagine_in_array(img):
    a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
    return a[:, :, :3].copy()


def profondita_in_metri(img):
    a = immagine_in_array(img).astype(np.float64)
    b, g, r = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    return ((r + g * 256 + b * 65536) / (256 ** 3 - 1) * 1000).astype(np.float32)


def distanza_del_riquadro(profondita, x1, y1, x2, y2):
    w, h = x2 - x1, y2 - y1
    pezzo = profondita[int(y1 + 0.3 * h):int(y2 - 0.3 * h) + 1, int(x1 + 0.3 * w):int(x2 - 0.3 * w) + 1]
    return float(np.median(pezzo)) if pezzo.size else None


def spostamento_laterale(x_pixel, distanza):
    f = LARGHEZZA / (2 * math.tan(math.radians(FOV) / 2))
    return (x_pixel - LARGHEZZA / 2) * distanza / f


# ----------------------------------------------------------------------------
# PROGRAMMA PRINCIPALE
# ----------------------------------------------------------------------------
modello = YOLO(MODELLO)
client = carla.Client(HOST, 2000)
client.set_timeout(120)
world = client.load_world(MAPPA) if CARICA_MAPPA else client.get_world()
impostazioni = world.get_settings()
impostazioni.synchronous_mode = True
impostazioni.fixed_delta_seconds = 0.05
world.apply_settings(impostazioni)
traffico = client.get_trafficmanager(8000)
traffico.set_synchronous_mode(True)
world.set_weather(getattr(carla.WeatherParameters, METEO))

blueprints = world.get_blueprint_library()
punti = world.get_map().get_spawn_points()
attori = []
video = cv2.VideoWriter("demo_live.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 10, (LARGHEZZA, ALTEZZA))

try:
    auto = world.spawn_actor(blueprints.find("vehicle.tesla.model3"), punti[PUNTO_PARTENZA % len(punti)])
    attori.append(auto)
    for punto in random.sample(punti, min(NUMERO_AUTO_TRAFFICO, len(punti))):
        altra = world.try_spawn_actor(random.choice(blueprints.filter("vehicle.*")), punto)
        if altra is not None:
            altra.set_autopilot(True, 8000)
            attori.append(altra)
    cam_rgb, coda_rgb = crea_camera(world, blueprints, "sensor.camera.rgb", auto)
    cam_prof, coda_prof = crea_camera(world, blueprints, "sensor.camera.depth", auto)
    attori += [cam_rgb, cam_prof]
    auto.set_autopilot(True, 8000)

    tempi = []
    for passo in range(int(SECONDI / 0.05)):
        world.tick()
        img_rgb = coda_rgb.get(timeout=10)
        img_prof = coda_prof.get(timeout=10)
        if passo % 2 != 0:                       # analizziamo 1 passo su 2 (10 immagini al secondo)
            continue
        img = immagine_in_array(img_rgb)
        profondita = profondita_in_metri(img_prof)

        inizio = time.time()
        risultato = modello(img, conf=0.3, verbose=False)[0]
        ms = (time.time() - inizio) * 1000
        tempi.append(ms)

        for riquadro in risultato.boxes:
            nome_yolo = modello.names[int(riquadro.cls)]
            if nome_yolo not in CLASSI_UTILI:
                continue
            x1, y1, x2, y2 = [int(v) for v in riquadro.xyxy[0].tolist()]
            distanza = distanza_del_riquadro(profondita, x1, y1, x2, y2)
            testo = nome_yolo
            if distanza is not None:
                testo += f" {distanza:.1f} m"
                if abs(spostamento_laterale((x1 + x2) / 2, distanza)) < 1.75:
                    testo += " | DAVANTI"
            colore = COLORI[CLASSI_UTILI[nome_yolo]]
            cv2.rectangle(img, (x1, y1), (x2, y2), colore, 2)
            cv2.putText(img, testo, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colore, 2)

        cv2.putText(img, f"YOLO: {ms:.0f} ms", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        video.write(img)
        if MOSTRA_FINESTRA:
            cv2.imshow("DEDALUS - demo percezione", img)
            if cv2.waitKey(1) == 27:             # 27 = tasto ESC
                break

    if len(tempi) > 1:
        media = sum(tempi[1:]) / len(tempi[1:])
        print(f"Tempo medio YOLO: {media:.1f} ms  ->  {1000 / media:.1f} immagini al secondo")
    print("Video salvato: demo_live.mp4")

finally:
    video.release()
    if MOSTRA_FINESTRA:
        cv2.destroyAllWindows()
    for attore in attori:
        if attore.type_id.startswith("sensor"):
            attore.stop()
    client.apply_batch([carla.command.DestroyActor(a) for a in attori])
    impostazioni.synchronous_mode = False
    impostazioni.fixed_delta_seconds = None
    world.apply_settings(impostazioni)
    traffico.set_synchronous_mode(False)
