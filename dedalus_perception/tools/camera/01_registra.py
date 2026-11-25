# =============================================================================
# 01_registra.py  -  Registra una SESSIONE di dati da CARLA
# =============================================================================
# Cosa fa:
#   1. si collega a CARLA e carica la mappa
#   2. mette un'auto (la "nostra") e un po' di traffico
#   3. monta sull'auto 4 sensori: camera a colori, camera di profondita',
#      camera di segmentazione, LiDAR 3D
#   4. l'auto guida da sola (autopilot) e noi salviamo i dati
#
# Come si usa:
#   - avvia CARLA
#   - cambia le IMPOSTAZIONI qui sotto (nome della sessione, meteo, ...)
#   - scrivi nel terminale:   python 01_registra.py
#
# Ogni sessione finisce in una cartella:  dati/<NOME_SESSIONE>/
# =============================================================================

import json
import os
import queue
import random
import time

import carla
import cv2
import numpy as np

# ----------------------------------------------------------------------------
# IMPOSTAZIONI (cambia solo queste)
# ----------------------------------------------------------------------------
NOME_SESSIONE = "train_s02"     # il nome dice anche il gruppo: train_..., val_..., test_...
METEO = "ClearNoon"             # altri: CloudyNoon, WetNoon, ClearSunset, HardRainNoon
PUNTO_PARTENZA = 1              # numero del punto di partenza nella mappa (prova 1, 30, 60)
NUMERO_FOTO = 200               # quante immagini salvare
NUMERO_AUTO_TRAFFICO = 20      # meno auto = PC meno affaticato
IGNORA_SEMAFORI = True          # True: le nostre auto non si fermano ai rossi (in CARLA durano ~30 s)
SEME = 1                        # "seme" casuale: stesso seme = stessa registrazione

MAPPA = "Town02"                # PC deboli (2-4 GB di VRAM): Town02. PC potenti: "Town10HD_Opt"
CARICA_MAPPA = True             # False se CARLA e' CONDIVISO (container del team): usa il mondo gia' aperto,
                                # perche' caricare una mappa resetta il mondo per TUTTI quelli collegati
HOST = "localhost"              # IP del PC con CARLA (localhost = questo PC)
LARGHEZZA, ALTEZZA = 960, 540   # risoluzione camera
FOV = 90.0                      # angolo di vista orizzontale (lavoriamo solo in CARLA: deciso dal team)

# Auto e posizione della camera (concordate con Oguz)
VEICOLO = "vehicle.tesla.model3"
CAM_X = 1.5                     # metri davanti al CENTRO dell'auto
CAM_Z = 1.5                     # metri da terra
CAM_PITCH = -8.0                # gradi: negativo = guarda in giu'
LIDAR_X, LIDAR_Z = 0.0, 2.0     # LiDAR sul tetto, sopra il centro dell'auto
SALVA_OGNI = 5                  # salva 1 immagine ogni 5 passi (4 al secondo)

# PC deboli: spegni i sensori che non servono subito (per la demo bastano foto + profondita')
USA_SEGMENTAZIONE = True        # False = niente etichette (piu' leggero)
USA_LIDAR = True                # False = niente LiDAR (molto piu' leggero)


# ----------------------------------------------------------------------------
# FUNZIONI DI AIUTO
# ----------------------------------------------------------------------------
def crea_camera(world, blueprints, tipo, auto):
    """Crea una camera di CARLA attaccata all'auto e una 'coda' dove arrivano le immagini."""
    bp = blueprints.find(tipo)
    bp.set_attribute("image_size_x", str(LARGHEZZA))
    bp.set_attribute("image_size_y", str(ALTEZZA))
    bp.set_attribute("fov", str(FOV))
    # posizione rispetto al centro dell'auto (valori nelle IMPOSTAZIONI)
    posizione = carla.Transform(carla.Location(x=CAM_X, z=CAM_Z), carla.Rotation(pitch=CAM_PITCH))
    camera = world.spawn_actor(bp, posizione, attach_to=auto)
    coda = queue.Queue()
    camera.listen(coda.put)       # ogni immagine nuova viene messa nella coda
    return camera, coda


def crea_lidar(world, blueprints, auto):
    """LiDAR 3D a 32 canali sul tetto, come deciso dal coordinatore."""
    bp = blueprints.find("sensor.lidar.ray_cast")
    bp.set_attribute("channels", "32")
    bp.set_attribute("range", "50")                 # metri
    bp.set_attribute("upper_fov", "10")             # gradi sopra l'orizzonte
    bp.set_attribute("lower_fov", "-30")            # gradi sotto l'orizzonte
    bp.set_attribute("rotation_frequency", "20")    # un giro completo a ogni passo (20 passi/s)
    bp.set_attribute("points_per_second", "576000")
    posizione = carla.Transform(carla.Location(x=LIDAR_X, z=LIDAR_Z))
    lidar = world.spawn_actor(bp, posizione, attach_to=auto)
    coda = queue.Queue()
    lidar.listen(coda.put)
    return lidar, coda


def immagine_in_array(img):
    """Trasforma un'immagine di CARLA in un array NumPy (altezza x larghezza x 3 colori)."""
    a = np.frombuffer(img.raw_data, dtype=np.uint8)
    a = a.reshape((img.height, img.width, 4))   # CARLA da' 4 canali: Blu, Verde, Rosso, Alfa
    return a[:, :, :3].copy()                   # teniamo solo B, G, R (formato di OpenCV)


def profondita_in_metri(img):
    """La camera di profondita' codifica la distanza nei 3 colori. Formula dalla documentazione di CARLA."""
    a = immagine_in_array(img).astype(np.float64)
    b, g, r = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    normalizzato = (r + g * 256 + b * 256 * 256) / (256 * 256 * 256 - 1)
    return (normalizzato * 1000).astype(np.float32)   # metri (massimo 1000)


def misure_auto(auto):
    """Misure ESATTE dell'auto lette da CARLA (metri, rispetto al centro dell'auto)."""
    bb = auto.bounding_box
    misure = {"length": round(2 * bb.extent.x, 3), "width": round(2 * bb.extent.y, 3),
              "height": round(2 * bb.extent.z, 3), "bbox_center_x": round(bb.location.x, 3)}
    # posizione delle ruote -> passo e asse posteriore (CARLA le da' nel mondo, in centimetri)
    try:
        inversa = np.array(auto.get_transform().get_inverse_matrix())
        xs = []
        for ruota in auto.get_physics_control().wheels:
            p = np.array([ruota.position.x / 100, ruota.position.y / 100, ruota.position.z / 100, 1.0])
            xs.append(float((inversa @ p)[0]))
        xs.sort()                                  # le 2 piu' indietro = asse posteriore
        misure["rear_axle_from_center"] = round((xs[0] + xs[1]) / 2, 3)
        misure["front_axle_from_center"] = round((xs[2] + xs[3]) / 2, 3)
        misure["wheelbase"] = round(misure["front_axle_from_center"] - misure["rear_axle_from_center"], 3)
    except Exception as e:                         # se la versione di CARLA non lo permette, pazienza
        print("Non riesco a leggere le ruote:", e)
    return misure


def errore_corsia(auto, mappa, asse_posteriore):
    """Verita' di CARLA per la corsia: distanza laterale e angolo rispetto al CENTRO della corsia,
    misurati nel punto base_link (centro dell'asse posteriore, come nel README di aea_dedalus).
    Convenzione ROS: e_y > 0 = auto a SINISTRA del centro, e_psi > 0 = muso girato a SINISTRA."""
    t = auto.get_transform()
    avanti = t.get_forward_vector()                 # spostiamo il punto dal centro all'asse posteriore
    t.location = t.location + carla.Location(x=avanti.x * asse_posteriore, y=avanti.y * asse_posteriore)
    wp = mappa.get_waypoint(t.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    dx, dy = t.location.x - wp.transform.location.x, t.location.y - wp.transform.location.y
    destra = wp.transform.get_right_vector()      # CARLA: y verso DESTRA
    e_y = -(dx * destra.x + dy * destra.y)         # meno: in base_link y e' verso sinistra
    dpsi = (t.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180
    e_psi = -np.radians(dpsi)                      # meno: in CARLA lo yaw positivo gira a destra
    return e_y, e_psi, wp.lane_width


# ----------------------------------------------------------------------------
# PROGRAMMA PRINCIPALE
# ----------------------------------------------------------------------------
cartella = os.path.join("dati", NOME_SESSIONE)
for sotto in ["rgb", "profondita", "segmentazione", "lidar"]:   # (le cartelle vuote non danno fastidio)
    os.makedirs(os.path.join(cartella, sotto), exist_ok=True)

random.seed(SEME)
client = carla.Client(HOST, 2000)
client.set_timeout(120)
if CARICA_MAPPA:
    print("Carico la mappa", MAPPA, "...")
    world = client.load_world(MAPPA)
else:
    world = client.get_world()                   # il mondo che c'e' gia'
    print("Uso la mappa gia' aperta:", world.get_map().name)

# Modalita' SINCRONA: il mondo va avanti di un passo solo quando lo diciamo noi.
# Cosi' tutti i sensori fotografano lo stesso istante e la registrazione e' ripetibile.
impostazioni = world.get_settings()
impostazioni.synchronous_mode = True
impostazioni.fixed_delta_seconds = 0.05          # un passo = 0.05 s  (20 passi al secondo)
world.apply_settings(impostazioni)

traffico = client.get_trafficmanager(8000)       # il "pilota automatico" di CARLA
traffico.set_synchronous_mode(True)
traffico.set_random_device_seed(SEME)
world.set_weather(getattr(carla.WeatherParameters, METEO))

blueprints = world.get_blueprint_library()
punti = world.get_map().get_spawn_points()
attori = []                                      # tutto quello che creiamo, per cancellarlo alla fine

try:
    # 1) la nostra auto
    auto = world.spawn_actor(blueprints.find(VEICOLO), punti[PUNTO_PARTENZA % len(punti)])
    attori.append(auto)

    # 2) il traffico: altre auto in punti a caso, con il pilota automatico
    n = PUNTO_PARTENZA % len(punti)              # % = resto: evita numeri piu' grandi dei punti esistenti
    altri_punti = punti[:n] + punti[n + 1:]
    random.shuffle(altri_punti)
    modelli_auto = blueprints.filter("vehicle.*")
    for punto in altri_punti[:NUMERO_AUTO_TRAFFICO]:
        altra = world.try_spawn_actor(random.choice(modelli_auto), punto)
        if altra is not None:                    # None = posto occupato, pazienza
            altra.set_autopilot(True, 8000)
            if IGNORA_SEMAFORI:
                traffico.ignore_lights_percentage(altra, 100)   # 100% = ignora sempre il rosso
            attori.append(altra)
    print("Auto nel traffico:", len(attori) - 1)

    # 3) i sensori
    cam_rgb, coda_rgb = crea_camera(world, blueprints, "sensor.camera.rgb", auto)
    cam_prof, coda_prof = crea_camera(world, blueprints, "sensor.camera.depth", auto)
    attori += [cam_rgb, cam_prof]
    if USA_SEGMENTAZIONE:
        cam_seg, coda_seg = crea_camera(world, blueprints, "sensor.camera.semantic_segmentation", auto)
        attori.append(cam_seg)
    if USA_LIDAR:
        lidar, coda_lidar = crea_lidar(world, blueprints, auto)
        attori.append(lidar)

    # 4) la nostra auto guida da sola
    auto.set_autopilot(True, 8000)
    traffico.auto_lane_change(auto, False)       # niente cambi di corsia: resta nella sua
    if IGNORA_SEMAFORI:
        traffico.ignore_lights_percentage(auto, 100)

    # qualche passo a vuoto per partire
    print("Sensori creati, aspetto le prime immagini ...")
    for i in range(40):
        world.tick()
        try:
            coda_rgb.get(timeout=10); coda_prof.get(timeout=10)
            if USA_SEGMENTAZIONE:
                coda_seg.get(timeout=10)
            if USA_LIDAR:
                coda_lidar.get(timeout=10)
        except queue.Empty:
            print("ERRORE: CARLA non manda i dati dei sensori (aspettato 10 s).")
            print("Prova prima: python3 test_sensori.py  (vedi LEGGIMI, sezione 'Si blocca')")
            raise SystemExit(1)
    print("Le immagini arrivano, comincio a registrare.")

    # camera_info.json: tutti i numeri della camera e dell'auto, salvati in automatico
    info = {"session": NOME_SESSIONE, "map": world.get_map().name.split("/")[-1], "weather": METEO,
            "ego_vehicle": VEICOLO, "width": LARGHEZZA, "height": ALTEZZA, "fov_deg": FOV,
            "cam_x_from_center": CAM_X, "cam_y_from_center": 0.0, "cam_z": CAM_Z, "cam_pitch_deg": CAM_PITCH,
            "base_link_from_center": None,           # riempito sotto con l'asse posteriore misurato
            "lidar": {"x_from_center": LIDAR_X, "z": LIDAR_Z, "channels": 32, "range_m": 50,
                      "upper_fov_deg": 10, "lower_fov_deg": -30},
            "vehicle": misure_auto(auto),
            "frames": {"base_link": "origin on the ground under the centre of the REAR AXLE, x forward, y left, z up",
                       "carla_note": "CARLA uses x forward, y RIGHT, z up; camera/LiDAR values are relative "
                                     "to the CARLA vehicle centre"},
            "info_csv": "e_y_m > 0: car LEFT of lane centre; e_psi_rad > 0: nose turned LEFT (ROS convention)"}
    asse_posteriore = info["vehicle"].get("rear_axle_from_center", -1.4)   # -1.4 = stima se CARLA non lo da'
    info["base_link_from_center"] = asse_posteriore
    with open(os.path.join(cartella, "camera_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print("Salvato camera_info.json, auto:", info["vehicle"])
    mappa = world.get_map()

    # 5) registrazione
    righe = ["foto,tempo_s,velocita_m_s,x,y,yaw_deg,e_y_m,e_psi_rad,larghezza_corsia_m"]   # CSV con la verita' di CARLA
    salvate, passo = 0, 0
    while salvate < NUMERO_FOTO:
        world.tick()                             # il mondo va avanti di 0.05 s
        passo += 1
        img_rgb = coda_rgb.get(timeout=10)       # prendiamo i dati di QUESTO passo
        img_prof = coda_prof.get(timeout=10)
        if USA_SEGMENTAZIONE:
            img_seg = coda_seg.get(timeout=10)
        if USA_LIDAR:
            dati_lidar = coda_lidar.get(timeout=10)

        v = auto.get_velocity()
        velocita = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
        if passo % SALVA_OGNI != 0 or velocita < 0.5:
            continue                             # non salviamo: troppo presto o auto ferma (semaforo)

        nome = f"{salvate:06d}"                  # 000000, 000001, ...
        cv2.imwrite(os.path.join(cartella, "rgb", nome + ".png"), immagine_in_array(img_rgb))
        np.save(os.path.join(cartella, "profondita", nome + ".npy"), profondita_in_metri(img_prof))
        if USA_SEGMENTAZIONE:
            # segmentazione: il numero della classe (strada, strisce, auto, pedone...) e' nel canale ROSSO
            classi = immagine_in_array(img_seg)[:, :, 2]
            cv2.imwrite(os.path.join(cartella, "segmentazione", nome + ".png"), classi)
            # versione colorata, solo da guardare
            img_seg.save_to_disk(os.path.join(cartella, "segmentazione", nome + "_colori.png"),
                                 carla.ColorConverter.CityScapesPalette)
        if USA_LIDAR:
            # LiDAR: una lista di punti, ognuno con x, y, z, intensita'
            punti_lidar = np.frombuffer(dati_lidar.raw_data, dtype=np.float32).reshape(-1, 4)
            np.save(os.path.join(cartella, "lidar", nome + ".npy"), punti_lidar)

        tr = auto.get_transform()
        e_y, e_psi, larghezza_corsia = errore_corsia(auto, mappa, asse_posteriore)
        righe.append(f"{nome},{img_rgb.timestamp:.2f},{velocita:.2f},{tr.location.x:.3f},{tr.location.y:.3f},"
                     f"{tr.rotation.yaw:.2f},{e_y:.3f},{e_psi:.4f},{larghezza_corsia:.2f}")
        salvate += 1
        if salvate % 20 == 0:
            print("Salvate", salvate, "di", NUMERO_FOTO)

    with open(os.path.join(cartella, "info.csv"), "w") as f:
        f.write("\n".join(righe))
    print("Fatto! Dati in", cartella)

finally:
    # SEMPRE alla fine (anche se c'e' un errore): togliamo tutto e rimettiamo CARLA normale
    for attore in attori:
        if attore.type_id.startswith("sensor"):
            attore.stop()
    client.apply_batch([carla.command.DestroyActor(a) for a in attori])
    impostazioni.synchronous_mode = False
    impostazioni.fixed_delta_seconds = None
    world.apply_settings(impostazioni)
    traffico.set_synchronous_mode(False)
