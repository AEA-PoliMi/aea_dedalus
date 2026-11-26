# =============================================================================
# 02_riconosci.py  -  Trova veicoli e pedoni nelle immagini e calcola la distanza
# =============================================================================
# Non serve CARLA: lavora sulle immagini gia' registrate con 01_registra.py.
# Funziona anche su Kaggle.
#
# Cosa fa, per ogni immagine:
#   1. YOLO (gia' addestrato) trova i riquadri di auto e persone
#   2. dalla immagine di PROFONDITA' leggiamo quanto e' lontano ogni riquadro
#   3. con il modello della camera calcoliamo anche se e' a destra o a sinistra
#   4. disegniamo tutto sull'immagine e salviamo
#   5. misuriamo quanti millisecondi ci mette
#
# Come si usa:   python 02_riconosci.py
# Prima, una volta sola:   pip install ultralytics
# =============================================================================

import math
import os
import time

import cv2
import numpy as np
from ultralytics import YOLO

# ----------------------------------------------------------------------------
# IMPOSTAZIONI
# ----------------------------------------------------------------------------
CARTELLA = "dati/test_s07"       # la sessione da analizzare
CARTELLA_RISULTATI = "risultati_test_s07"   # dove salvare immagini, video e CSV
                                 # (su Kaggle i dati sono in sola lettura: i risultati vanno in /kaggle/working/...)
MODELLO = "yolo26n.pt"           # n = nano (il piu' veloce). Prova anche yolo26s.pt
CONFIDENZA_MINIMA = 0.3          # sotto questo valore ignoriamo il riquadro
FOV = 90.0                       # deve essere lo stesso di 01_registra.py
LARGHEZZA_CORSIA = 3.5           # metri: per dire se un ostacolo e' "davanti a noi"

# Le classi di YOLO (dataset COCO) che ci interessano, e come le chiamiamo noi
CLASSI_UTILI = {"person": "pedone", "bicycle": "veicolo", "car": "veicolo",
                "motorcycle": "veicolo", "bus": "veicolo", "truck": "veicolo"}
COLORI = {"veicolo": (0, 165, 255), "pedone": (255, 0, 255)}   # arancione, magenta (B, G, R)


# ----------------------------------------------------------------------------
# FUNZIONI
# ----------------------------------------------------------------------------
def distanza_del_riquadro(profondita, x1, y1, x2, y2):
    """Distanza (metri) di un oggetto: mediana della profondita' nel CENTRO del riquadro.

    Usiamo solo il centro perche' ai bordi del riquadro c'e' spesso lo sfondo.
    La mediana (non la media) ignora i pochi pixel strani.
    """
    larghezza, altezza = x2 - x1, y2 - y1
    cx1 = int(x1 + 0.3 * larghezza)
    cx2 = int(x2 - 0.3 * larghezza)
    cy1 = int(y1 + 0.3 * altezza)
    cy2 = int(y2 - 0.3 * altezza)
    pezzo = profondita[cy1:cy2 + 1, cx1:cx2 + 1]
    if pezzo.size == 0:
        return None
    return float(np.median(pezzo))


def spostamento_laterale(x_pixel, distanza, larghezza_immagine):
    """Modello 'pinhole' della camera: da pixel + distanza a metri verso destra (+) o sinistra (-).

    f = lunghezza focale in pixel, cx = centro dell'immagine.
    X = (u - cx) * Z / f
    """
    f = larghezza_immagine / (2 * math.tan(math.radians(FOV) / 2))
    cx = larghezza_immagine / 2
    return (x_pixel - cx) * distanza / f


# ----------------------------------------------------------------------------
# PROGRAMMA PRINCIPALE
# ----------------------------------------------------------------------------
modello = YOLO(MODELLO)          # la prima volta scarica il modello da internet
cartella_out = CARTELLA_RISULTATI
os.makedirs(cartella_out, exist_ok=True)

nomi = sorted(f[:-4] for f in os.listdir(os.path.join(CARTELLA, "rgb")) if f.endswith(".png"))
print("Immagini da analizzare:", len(nomi))

tempi = []                                           # millisecondi per immagine
righe = ["foto,classe,confidenza,distanza_m,laterale_m,davanti_a_noi"]
video = None

for nome in nomi:
    img = cv2.imread(os.path.join(CARTELLA, "rgb", nome + ".png"))
    profondita = np.load(os.path.join(CARTELLA, "profondita", nome + ".npy"))

    # 1) YOLO
    inizio = time.time()
    risultato = modello(img, conf=CONFIDENZA_MINIMA, verbose=False)[0]
    ms = (time.time() - inizio) * 1000
    tempi.append(ms)

    # 2-3) per ogni riquadro trovato
    for riquadro in risultato.boxes:
        nome_yolo = modello.names[int(riquadro.cls)]
        if nome_yolo not in CLASSI_UTILI:
            continue                                 # es. semafori, panchine: non ci interessano
        classe = CLASSI_UTILI[nome_yolo]
        confidenza = float(riquadro.conf)
        x1, y1, x2, y2 = [int(v) for v in riquadro.xyxy[0].tolist()]

        distanza = distanza_del_riquadro(profondita, x1, y1, x2, y2)
        if distanza is None:
            testo = f"{nome_yolo} {confidenza:.2f}"
            laterale, davanti = None, False
        else:
            laterale = spostamento_laterale((x1 + x2) / 2, distanza, img.shape[1])
            davanti = abs(laterale) < LARGHEZZA_CORSIA / 2
            testo = f"{nome_yolo} {confidenza:.2f} | {distanza:.1f} m"
            if davanti:
                testo += " | DAVANTI"
        if distanza is None:
            righe.append(f"{nome},{classe},{confidenza:.2f},,,")
        else:
            righe.append(f"{nome},{classe},{confidenza:.2f},{distanza:.2f},{laterale:.2f},{davanti}")

        # 4) disegno
        colore = COLORI[classe]
        cv2.rectangle(img, (x1, y1), (x2, y2), colore, 2)
        cv2.putText(img, testo, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colore, 2)

    cv2.putText(img, f"{nome}  |  {ms:.0f} ms", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imwrite(os.path.join(cartella_out, nome + ".jpg"), img)

    if video is None:                                # anche un video, per la presentazione
        video = cv2.VideoWriter(os.path.join(cartella_out, "video.mp4"),
                                cv2.VideoWriter_fourcc(*"mp4v"), 4, (img.shape[1], img.shape[0]))
    video.write(img)

video.release()
with open(os.path.join(cartella_out, "rilevamenti.csv"), "w") as f:
    f.write("\n".join(righe))

# 5) velocita' (la prima immagine e' lenta perche' il modello si sta "scaldando": la saltiamo)
tempi_utili = tempi[1:] if len(tempi) > 1 else tempi
media = sum(tempi_utili) / len(tempi_utili)
print(f"Tempo medio: {media:.1f} ms per immagine  ->  {1000 / media:.1f} immagini al secondo")
print("Risultati in", cartella_out)
