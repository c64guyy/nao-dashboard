# NAO Dashboard

Dashboard desktop per monitorare in tempo reale un robot **NAO** (SoftBank / Aldebaran): giunti, temperature, sensori, batteria e log, tutto in un'unica finestra.

È un'applicazione **di sola lettura**: legge i dati dal robot ma non lo muove e non ne cambia lo stato.

<!-- Aggiungi qui uno screenshot, ad es.: ![Screenshot](docs/screenshot.png) -->

## Funzionalità

- **Panoramica**: nome e versione del robot, postura, stato dei motori, stato di Autonomous Life, batteria, giunto più caldo, numero di giunti rilassati e avvisi attivi.
- **Giunti**: tabella raggruppata per parte del corpo (testa, braccia, gambe) con angolo, limiti min/max, posizione nel range, rigidità, temperatura, corrente e stato (OK / AVVISO / CRITICO). Le righe si colorano quando si superano le soglie.
- **Temperature nel tempo**: grafico storico filtrabile per gruppo di giunti (o solo il giunto selezionato in tabella) e per finestra temporale (5 min, 15 min, 1 ora, tutto), con le linee delle soglie di avviso e critica.
- **Sensori**: IMU (angoli, accelerometro, giroscopio), sensori di pressione dei piedi (FSR), sonar, sensori tattili di testa e mani, paraurti, pulsante sul petto, batteria.
- **Log**: messaggi del robot (via `LogManager`) ed eventi ALMemory (cadute, tocchi, pulsante, cambio postura, caricatore...), con filtro per livello e per testo, auto-scroll e salvataggio su file `.txt`.
- **Avvisi di temperatura**: soglie configurabili (predefinite 60 °C e 70 °C); ogni superamento viene scritto nel log una sola volta.
- **Registrazione CSV**: opzionale, salva a ogni lettura batteria, temperature, angoli e correnti di tutti i giunti in `nao_logs/nao_robot_AAAAMMGG_HHMMSS.csv`.

## Requisiti

- Python 3.8+ con Tkinter (incluso nell'installer ufficiale di Python per Windows e macOS; su Linux può servire `sudo apt install python3-tk`)
- [matplotlib](https://matplotlib.org/)
- L'SDK Python di NAOqi (modulo `qi`)
- Un robot NAO raggiungibile in rete (NAOqi 2.x)

## Installazione

```bash
git clone https://github.com/<tuo-utente>/nao_dashboard.git
cd nao_dashboard

python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

Poi installa il modulo `qi` per il tuo sistema:

| Sistema        | Comando                        |
|----------------|--------------------------------|
| Linux / macOS  | `pip install qi`               |
| Windows        | `pip install libqi-windows`    |

> Se il modulo `qi` non è installato l'app si avvia comunque, ma alla pressione di **Connetti** mostra un errore.

## Utilizzo

```bash
python3 nao_dashboard.py
```

1. Inserisci l'**IP del robot** (premi il pulsante sul petto del NAO per sentirlo pronunciare) e la **porta** (predefinita `9559`).
2. Imposta l'**intervallo di lettura** in secondi (minimo 0,2 s) e, se vuoi, le soglie di temperatura.
3. Spunta **Attiva sonar** se ti servono i valori dei sonar e **Registra CSV** se vuoi salvare i dati.
4. Premi **Connetti**.

Cliccando su un giunto nella tabella lo si seleziona; scegliendo *Giunto selezionato* nel menu del grafico se ne vede solo la temperatura. Le colonne sono ridimensionabili trascinando i divisori.

## Dati salvati

- `nao_logs/*.csv`: una riga per lettura con colonne `timestamp`, `battery_pct`, `temp_c_<giunto>`, `angle_deg_<giunto>`, `current_a_<giunto>`. La cartella viene creata nella directory da cui si avvia lo script.
- Log: con il pulsante **Salva...** nel pannello Log.

## Note

- Attivando il sonar l'app si iscrive ad `ALSonar` con il nome `NaoDashboard` e si disiscrive alla disconnessione.
- Se un servizio non è disponibile (ad es. `ALAutonomousLife` o `LogManager`) il relativo campo resta `--` e il resto della dashboard continua a funzionare.

## Licenza

Distribuito con licenza MIT. Vedi il file [LICENSE](LICENSE).
