# AgroRisk – Simulazione ROS2/Webots di un sistema cooperativo drone-rover

Simulazione di un sistema cooperativo drone-rover per la difesa fitosanitaria di
precisione, sviluppata in ROS2 (Jazzy) e Webots. A partire da un'allerta generata
dal sistema di supporto alla decisione AgroRisk, un drone verifica lo stato
vegetativo della zona segnalata; se il rischio è confermato, un rover interviene
con un trattamento localizzato. L'intero ciclo viene registrato in un dataset di log.

## Requisiti

- WSL2 con Ubuntu 24.04
- ROS2 Jazzy
- Webots

## Esecuzione della simulazione

Sono disponibili due modalità: il bringup unificato, consigliato per eseguire una
simulazione completa, e l'avvio manuale dei singoli componenti, utile durante lo
sviluppo e il debug.

### Build del workspace

Dalla radice del repository clonato:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

`source install/setup.bash` va eseguito in ogni nuovo terminale prima di lanciare
comandi ROS2. Se `colcon` non è installato:
`sudo apt install python3-colcon-common-extensions`.

### A. Modalità consigliata: bringup unificato

```bash
cd ros2_ws
source install/setup.bash
ros2 launch agro_bringup agro_bringup_launch.py
```

Un unico launch file avvia i componenti in sequenza:

1. Webots e i controller di drone e rover
2. `mission_logger`
3. `mission_manager`
4. `agro_alert_publisher`

Prima di passare a ogni stadio successivo, il launch verifica la presenza nel
grafo ROS2 delle principali interfacce richieste dai componenti già avviati:

- controller Webots: `drone_controller_webots` su `/agro/mission_cmd` e
  `/agro/treatment_done`, `rover_controller_webots` su `/agro/risk_confirmed`;
- `mission_logger` su `/agro/events`;
- `mission_manager` su `/agro/alert`, `/agro/events` e `/agro/risk_confirmed`.

Ogni verifica ha un timeout: se una di queste subscription non compare nel grafo
entro il tempo previsto, il bringup viene interrotto. `agro_alert_publisher`, che
genera le allerte da cui parte la simulazione, viene avviato per ultimo, dopo che
tutte le subscription elencate sopra risultano presenti nel grafo ROS2.

### B. Avvio manuale dei componenti (sviluppo/debug)

Questa modalità permette di osservare separatamente l'output di ciascun
componente, facilitando debug e diagnosi; non è quella consigliata per una
normale esecuzione completa.

I componenti vanno avviati in quattro terminali separati, in quest'ordine. In
ogni terminale, dopo essersi posizionati nel workspace, eseguire
`source install/setup.bash`.

**Terminale 1 — Webots e controller di drone e rover**

```bash
ros2 launch agro_webots agro_webots_launch.py
```

Attendere che Webots si avvii completamente e che la scena sia caricata prima di
procedere con i terminali successivi.

**Terminale 2 — `mission_logger`**

```bash
ros2 run agro_mission mission_logger
```

**Terminale 3 — `mission_manager`**

```bash
ros2 run agro_mission mission_manager
```

**Terminale 4 — `agro_alert_publisher`**

```bash
ros2 run agro_mission agro_alert_publisher
```

`agro_alert_publisher` va lanciato per ultimo: è il nodo che genera le allerte che
innescano l'intera sequenza.

### Pulizia tra sessioni (troubleshooting)

Se una simulazione precedente non si è chiusa correttamente, chiudere la
finestra di Webots e terminare gli eventuali nodi AgroRisk rimasti attivi
(`mission_manager`, `mission_logger`, `agro_alert_publisher`, i controller di
drone e rover) prima di riavviare. Evitare comandi di terminazione generici,
che possono interferire con altri progetti ROS2 o sessioni Webots in corso.

## Struttura del progetto

Il workspace ROS2 (`ros2_ws/src/`) contiene tre package:

- `agro_mission` — nodi puri, indipendenti da Webots: `agro_alert_publisher`,
  `mission_manager`, `mission_logger`;
- `agro_webots` — mondo Webots (`agro_field.wbt`) e controller dei robot:
  `drone_controller_webots`, `rover_controller_webots`;
- `agro_bringup` — solo il launch file `agro_bringup_launch.py`, che orchestra
  l'avvio sequenziale di tutti i componenti (modalità A).

I nodi comunicano tramite topic:

- `agro_alert_publisher` — genera una sequenza deterministica di allerte simulate da AgroRisk;
- `mission_manager` — coordina una missione alla volta: gestisce la FIFO delle
  allerte `high` e, quando il drone conferma il rischio, mantiene la missione
  attiva fino al rientro del rover prima di servire l'allerta successiva;
- `mission_logger` — registra tutti gli eventi in un dataset in formato CSV;
- `drone_controller_webots` — controlla il drone nella simulazione Webots (volo, verifica, effetti visivi);
- `rover_controller_webots` — controlla il rover per l'intervento localizzato.

Il dataset di log viene salvato nella cartella `~/agrorisk_dataset/`.
