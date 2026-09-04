# AgroRisk – Simulazione ROS2/Webots di un sistema cooperativo drone-rover

Simulazione di un sistema cooperativo drone-rover per la difesa fitosanitaria di
precisione, sviluppata in ROS2 (Jazzy) e Webots. A partire da un'allerta generata
dal sistema di supporto alla decisione AgroRisk, un drone raggiunge la zona
segnalata e ne verifica lo stato vegetativo; se il rischio è confermato, un rover
interviene con un trattamento localizzato. L'intero ciclo viene registrato in un
dataset di log in formato CSV.

## Ambiente verificato

| Componente         | Versione   |
| ------------------ | ---------- |
| Ubuntu             | 24.04 LTS  |
| ROS2               | Jazzy      |
| Webots             | R2025a     |
| webots_ros2_driver | 2025.0.0   |

La procedura descritta in questo README è stata verificata da installazione
pulita in una macchina virtuale VMware con Ubuntu 24.04 LTS, indipendente
dall'ambiente di sviluppo originale. La VM usata per il test disponeva di 4 GB di
RAM, 2 vCPU e rete NAT: sono le caratteristiche dell'ambiente di prova, non un
requisito minimo del progetto.

## Prerequisiti

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Webots R2025a
- `webots_ros2_driver` 2025.0.0 (pacchetto `ros-jazzy-webots-ros2-driver`)
- `git`, `colcon`, `rosdep`

## Installazione delle dipendenze

1. Installare **ROS2 Jazzy** seguendo la
   [guida ufficiale](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
   poi gli strumenti di build:

   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-rosdep
   ```

2. Installare **Webots R2025a** dal
   [pacchetto `.deb` ufficiale](https://github.com/cyberbotics/webots/releases/tag/R2025a)
   (installazione predefinita in `/usr/local/webots`). Dopo aver scaricato il
   file:

   ```bash
   cd ~/Downloads
   sudo apt install ./webots_2025a_amd64.deb -y
   ```

3. Installare il **driver ROS2 per Webots**:

   ```bash
   sudo apt install ros-jazzy-webots-ros2-driver
   ```

4. Inizializzare `rosdep` (solo la prima volta sul sistema):

   ```bash
   sudo rosdep init
   rosdep update
   ```

## Clone del repository

```bash
git clone https://github.com/nuccelliviola/agrorisk-ros2-simulation.git
cd agrorisk-ros2-simulation
```

## Build del workspace

Dalla radice del repository clonato:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Tutti i comandi ROS2 del progetto vanno eseguiti dalla cartella `ros2_ws/` del
repository. In ogni nuovo terminale, prima dei comandi `ros2`, ripetere:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Esecuzione della simulazione

Dalla cartella `ros2_ws/` del repository (proseguendo nello stesso terminale della
build si è già nella posizione corretta; in un nuovo terminale spostarsi prima in
`ros2_ws/`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agro_bringup agro_bringup_launch.py
```

Un unico launch file avvia i componenti in sequenza:

1. Webots (mondo `agro_field.wbt`) e i controller di drone e rover
2. `mission_logger`
3. `mission_manager`
4. `agro_alert_publisher`

Prima di passare a ogni stadio, il launch verifica nel grafo ROS2 la presenza
delle subscription richieste dai componenti già avviati (i controller Webots su
`/agro/mission_cmd`, `/agro/treatment_done` e `/agro/risk_confirmed`;
`mission_logger` su `/agro/events`; `mission_manager` su `/agro/alert`,
`/agro/events` e `/agro/risk_confirmed`). Ogni verifica ha un timeout: se una
subscription non compare entro il tempo previsto, il bringup viene interrotto.
`agro_alert_publisher`, che genera le allerte da cui parte la simulazione, viene
avviato per ultimo.

All'avvio, il driver Webots stampa il messaggio:

```
WARNING: No valid Webots directory specified in ROS2_WEBOTS_HOME and WEBOTS_HOME,
fallback to default installation folder /usr/local/webots.
```

È il comportamento atteso quando Webots è installato nella posizione predefinita
(`/usr/local/webots`): il driver la individua automaticamente e non è necessario
impostare la variabile d'ambiente `WEBOTS_HOME`.

## Comportamento atteso

`agro_alert_publisher` pubblica una sequenza fissa e deterministica di quattro
allerte (una ogni 15 secondi, la prima 15 secondi dopo l'avvio del nodo):

| Allerta | Zona   | risk_level | Esito                                     |
| ------- | ------ | ---------- | ----------------------------------------- |
| A001    | ZONA_B | high       | Avvia la missione `M001`                  |
| A002    | ZONA_C | high       | Accodata durante `M001`, poi avvia `M002` |
| A003    | ZONA_A | low        | Sotto soglia: nessuna missione            |
| A004    | ZONA_D | medium     | Sotto soglia: nessuna missione            |

Svolgimento:

- **A001 → M001 (rischio non confermato).** Il drone raggiunge ZONA_B ed esegue
  la verifica dell'indice di stress vegetativo simulato: il valore letto è `0.72`,
  sopra la soglia di `0.5`, quindi il rischio **non** è confermato. Il rover non
  viene attivato. Il rientro alla base del drone chiude `M001`.
- **A002 → coda FIFO → M002 (rischio confermato).** A002 arriva mentre `M001` è
  attiva e viene messa in coda (FIFO); al termine di `M001` viene estratta e
  avvia `M002`. Il drone raggiunge ZONA_C, legge un indice di stress di `0.34`
  (sotto soglia) e **conferma** il rischio. Il drone rientra, ma `M002` resta
  attiva: il rover raggiunge ZONA_C, esegue il trattamento localizzato e rientra
  alla base. È il rientro del **rover** a chiudere `M002`.
- **A003 e A004** hanno `risk_level` inferiore a `high`: vengono solo registrate
  nel dataset, senza avviare alcuna missione.

Al termine dell'esecuzione il dataset contiene **18 righe di evento più
l'intestazione** e **nessun evento `mission_error`**.

I tempi assoluti (durata delle missioni, attesa in coda, durata del trattamento)
dipendono dalla macchina e non costituiscono un valore di riferimento fisso. La
parte deterministica è la struttura dello scenario: la sequenza delle allerte, il
numero di eventi, la gestione della coda FIFO e l'esito dei due rami.

## Dataset di log

Il dataset viene salvato in:

```
~/agrorisk_dataset/log_<timestamp>.csv
```

Il percorso è configurabile tramite il parametro `log_path` del nodo
`mission_logger`. A ogni esecuzione `mission_logger` crea un nuovo file CSV con
timestamp nel nome e con la riga di intestazione. Ogni riga successiva rappresenta
un evento registrato. Colonne, nell'ordine:

```
timestamp_start
timestamp_end
duration_seconds
event_type
alert_id
mission_id
zone_id
drone_id
source
risk_level
pest
indice_stress
action
x
y
z
note
```

Il campo `source` indica quale nodo ha prodotto l'evento e può valere `manager`,
`drone` o `rover`. Nello scenario di riferimento compaiono due eventi
`return_to_base`, uno prodotto dal drone e uno dal rover: è normale, e i due si
distinguono proprio tramite il campo `source`.

## Avvio manuale dei componenti (sviluppo/debug)

Modalità alternativa, utile per osservare separatamente l'output di ciascun nodo.
I componenti vanno avviati in quattro terminali, in quest'ordine, tutti dalla
cartella `ros2_ws/` del repository. In ogni terminale, prima dei comandi `ros2`,
eseguire:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Poi, un comando per terminale:

```bash
# Terminale 1 – Webots e controller di drone e rover
ros2 launch agro_webots agro_webots_launch.py

# Terminale 2 – mission_logger
ros2 run agro_mission mission_logger

# Terminale 3 – mission_manager
ros2 run agro_mission mission_manager

# Terminale 4 – agro_alert_publisher (per ultimo: genera le allerte)
ros2 run agro_mission agro_alert_publisher
```

Attendere che Webots abbia caricato completamente la scena prima di avviare i
nodi dei terminali successivi.

## Pulizia tra sessioni

Se una simulazione precedente non si è chiusa correttamente, chiudere la finestra
di Webots e terminare gli eventuali nodi AgroRisk ancora attivi (`mission_manager`,
`mission_logger`, `agro_alert_publisher`, i controller di drone e rover) prima di
riavviare. Evitare comandi di terminazione generici, che possono interferire con
altre sessioni ROS2 o Webots in corso.

## Struttura del progetto

Il workspace ROS2 (`ros2_ws/src/`) contiene tre package:

- `agro_mission` — nodi indipendenti da Webots: `agro_alert_publisher`,
  `mission_manager`, `mission_logger`;
- `agro_webots` — mondo Webots (`agro_field.wbt`) e controller dei robot:
  `drone_controller_webots`, `rover_controller_webots`;
- `agro_bringup` — il launch file `agro_bringup_launch.py`, che orchestra l'avvio
  sequenziale di tutti i componenti.

I nodi comunicano tramite topic:

- `agro_alert_publisher` — genera la sequenza deterministica di allerte simulate
  da AgroRisk;
- `mission_manager` — coordina una missione alla volta: gestisce la coda FIFO
  delle allerte `high` e, quando il drone conferma il rischio, mantiene la
  missione attiva fino al rientro del rover prima di servire l'allerta successiva;
- `mission_logger` — registra tutti gli eventi nel dataset CSV;
- `drone_controller_webots` — controlla il drone nella simulazione Webots (volo,
  verifica, effetti visivi);
- `rover_controller_webots` — controlla il rover per l'intervento localizzato.
