# DEDALUS · Percezione · versione semplice

Tre script Python da leggere **dall'alto in basso**. Ognuno ha in cima un blocco
`IMPOSTAZIONI`: è l'unica parte da cambiare.

| File | Cosa fa | Serve CARLA? |
|---|---|---|
| `01_registra.py` | Guida in CARLA e salva immagini, profondità, segmentazione e LiDAR | Sì |
| `02_riconosci.py` | Trova auto e pedoni nelle immagini salvate e calcola la distanza | **No** (va anche su Kaggle) |
| `03_demo_live.py` | Demo dal vivo: l'auto guida in CARLA e YOLO riconosce in tempo reale | Sì |

---

## Installazione (una volta sola)

```
pip install carla==0.9.16 numpy opencv-python ultralytics
```
Serve Python 3.10, 3.11 o 3.12. Su Kaggle basta `pip install ultralytics`.

---

## 1. Registrare i dati (`01_registra.py`)

Ogni volta che lo lanci registri **una sessione**, cioè un giro in macchina.
Per il dataset v0 servono 3 sessioni. Prima di ogni lancio cambia le impostazioni in cima:

| Lancio | `NOME_SESSIONE` | `PUNTO_PARTENZA` | `METEO` | `SEME` |
|---|---|---|---|---|
| 1° | `train_s01` | 1 | `ClearNoon` | 1 |
| 2° | `val_s06` | 30 | `ClearNoon` | 6 |
| 3° | `test_s07` | 60 | `ClearNoon` | 7 |

```
python 01_registra.py
```

**PC con poca memoria video (2–4 GB di VRAM)**: lascia `MAPPA = "Town02"` (piccola e leggera)
e avvia CARLA con `-quality-level=Low`. Con un PC potente usa `"Town10HD_Opt"`, la mappa del kickoff.
Se CARLA si blocca, abbassa `LARGHEZZA, ALTEZZA` a `640, 360` (lo stesso FOV: la camera "vede" uguale, con meno dettagli)
e `NUMERO_AUTO_TRAFFICO` a 10.
Ancora troppo pesante? Metti `USA_LIDAR = False` e `USA_SEGMENTAZIONE = False`: restano foto + profondità,
che bastano per la demo (YOLO + distanza). LiDAR ed etichette si registrano dopo, su un PC più potente o nel container.

Risultato:
```
dati/train_s01/
   rgb/000000.png               foto a colori
   profondita/000000.npy        distanza in metri di ogni pixel (array NumPy)
   segmentazione/000000.png     numero della classe di ogni pixel (strada, auto, pedone...)
   segmentazione/000000_colori.png   la stessa cosa a colori, da guardare
   lidar/000000.npy             punti 3D: una riga per punto, colonne x, y, z, intensità
   info.csv                     tempo, velocità e posizione dell'auto per ogni foto
```

**Semafori**: in CARLA un rosso dura circa 30 secondi. Lo script non salva le foto ad auto ferma,
ma la registrazione si allunga. Con `IGNORA_SEMAFORI = True` le auto dello script passano col rosso
(il pilota automatico evita comunque gli scontri): più movimento, registrazione più veloce.

**Le etichette automatiche** sono le immagini di `segmentazione/`: CARLA ci dice gratis
che cosa c'è in ogni pixel. Nel mondo reale bisognerebbe colorarle a mano.

**Diviso per sessione**: il nome della cartella dice il gruppo (`train_`, `val_`, `test_`).
Una sessione intera sta in **un solo** gruppo. Perché? Due foto di fila sono quasi
uguali: se una finisce nel train e quella dopo nel test, il test "ha già visto la risposta"
e il punteggio è falso.

---

## 2. Riconoscere auto e pedoni (`02_riconosci.py`)

Cambia `CARTELLA` (es. `dati/test_s07`) e lancia:
```
python 02_riconosci.py
```
Risultato nella cartella `CARTELLA_RISULTATI` (di default `risultati_test_s07/`):
- un'immagine `.jpg` per foto, con i riquadri e la distanza;
- `video.mp4`: tutte le immagini in sequenza, per la presentazione;
- `rilevamenti.csv`: classe, confidenza, distanza, spostamento laterale, "davanti a noi";
- nel terminale: **millisecondi per immagine** e **immagini al secondo**.

### Come funziona, passo per passo
1. **YOLO** (già addestrato su migliaia di foto reali) restituisce dei riquadri:
   `x1, y1, x2, y2` in pixel, la classe (`car`, `person`…) e la confidenza (0–1).
2. **Il riquadro non dice la distanza.** La leggiamo dall'immagine di profondità:
   prendiamo il **centro** del riquadro (ai bordi c'è lo sfondo) e la **mediana** dei valori.
3. **Destra o sinistra?** Modello della camera *pinhole*:
   `X = (u − cx) · Z / f`, dove `u` è la colonna del pixel, `cx` il centro dell'immagine,
   `Z` la distanza e `f` la focale in pixel: `f = larghezza / (2 · tan(FOV/2))`.
4. **Davanti a noi?** Se `|X|` è minore di metà corsia (1,75 m), l'ostacolo è sulla nostra strada.

### Cosa guardare per la presentazione
- **Dove funziona**: auto vicine e grandi.
- **Dove sbaglia**: pedoni lontani (pochi pixel), auto coperte da altre, pioggia o sole basso
  (registra una sessione con `METEO = "HardRainNoon"`). Inoltre YOLO ha imparato da foto vere,
  mentre CARLA è un videogioco.
- **Velocità**: il numero nel terminale. Scrivete su che scheda video l'avete misurata.

---

### Su Kaggle
1. Carica la cartella `dati/` compressa come **Dataset** privato (Kaggle la estrae da solo).
2. Apri un **Notebook**: *Settings → Accelerator → GPU T4*, *Internet → On*, e aggiungi il dataset con *Add Input*.
3. In una cella: `!pip install ultralytics`, poi carica `02_riconosci.py` (o incollalo in una cella) e cambia in cima:
   `CARTELLA = "/kaggle/input/<nome-dataset>/dati/test_s07"` e `CARTELLA_RISULTATI = "/kaggle/working/risultati_test_s07"`.
   (Il percorso esatto lo vedi nel pannello *Input* a destra: tasto destro → copia il percorso.)
4. Lancia `!python 02_riconosci.py`. Poi, per scaricare tutto: `!zip -r risultati.zip /kaggle/working/risultati_test_s07`
   e scarica `risultati.zip` dal pannello *Output*.

## 3. Demo dal vivo (`03_demo_live.py`)
```
python 03_demo_live.py
```
Si apre una finestra con l'auto che guida e i riquadri. `ESC` per fermare.
Il video viene salvato in `demo_live.mp4`.

---

## Cosa passiamo al team Planning & Control
Per ogni ostacolo: **classe**, **confidenza**, **distanza**, **spostamento laterale**,
**davanti a noi sì/no** e **l'istante** della foto (sono le colonne di `rilevamenti.csv`).
In ROS 2 diventeranno un messaggio sul topic `/perception/objects`.

Mancano ancora (prossimi passi): la **dimensione** dell'ostacolo, la sua **velocità**
(serve seguirlo da una foto all'altra: *tracking*), e l'uso del **LiDAR 3D** per misurare meglio
le distanze lontane.

## Nel container del team (dedalus-devenv)
Gli script girano **dentro il container `ros2`**, che ha già `carla`, NumPy e OpenCV.
- Mettete la cartella `dedalus_semplice` nella vostra **workspace** (`/ws`), così i `dati/` restano sul PC host
  e li vedete in VS Code.
- **CARLA è condiviso** da tutti quelli collegati allo stesso PC: metti `CARICA_MAPPA = False`
  (usa la mappa già aperta) oppure avvisa gli altri prima di cambiarla.
- **Uno alla volta**: questi script mettono CARLA in modalità sincrona. Se qualcun altro sta facendo girare
  un suo script sincrono sullo stesso CARLA, vi bloccate a vicenda. Mettetevi d'accordo.
- **YOLO non è nel container**: `02_riconosci.py` fatelo girare su Kaggle (come previsto). Per `03_demo_live.py`
  nel container serve `pip install --user --break-system-packages ultralytics` (grande, e si perde se il container
  viene ricreato) e `MOSTRA_FINESTRA = False`.

## Se qualcosa non va
- `ModuleNotFoundError: carla` → `pip install carla==0.9.16` con Python 3.10–3.12.
- Il programma resta fermo su "Carico la mappa" → CARLA non è avviato, oppure `HOST` è sbagliato.
- `queue.Empty` → un sensore non ha mandato dati: riavvia CARLA e riprova.
- Errore a metà → lo script rimette comunque CARLA in modalità normale (il blocco `finally`).

## Si blocca dopo "Auto nel traffico" (cartelle create ma vuote)

Vuol dire che CARLA esegue i comandi ma **le immagini delle camere non arrivano**.
CARLA usa due porte: **2000** per i comandi (funziona, infatti carica la mappa e crea il traffico)
e **2001** per mandare le immagini.

1. Lancia `python3 test_sensori.py` (metti dentro lo stesso `HOST` di `01_registra.py`).
2. Se dice **porta 2001 NON raggiungibile**: il container non arriva alla porta 2001.
   - Su Windows: Firewall di Windows → "Consenti app" → CarlaUE4 / CarlaUnreal spuntato sia **Privato** che **Pubblico**.
   - Se CARLA gira in Docker: deve essere avviato con `-p 2000-2002:2000-2002` (non solo 2000).
3. Se le porte sono OK ma le immagini non arrivano, o arrivano solo con la camera piccola:
   la scheda video non ce la fa. In `01_registra.py` metti `USA_SEGMENTAZIONE = False`,
   `USA_LIDAR = False` e, se serve, `LARGHEZZA, ALTEZZA = 640, 360`.
