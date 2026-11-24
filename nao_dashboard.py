#!/usr/bin/env python3
"""
NAO Dashboard - tutto su una sola pagina (Windows / Linux / macOS)

Sezioni:
  - Panoramica: postura, motori, batteria, giunto piu' caldo, avvisi
  - Giunti: tabella con angolo, posizione nel range, rigidita', temperatura, corrente
  - Temperature: grafico storico per gruppo di giunti
  - Sensori: IMU, piedi (FSR), sonar, tattili, batteria
  - Log: messaggi del robot + eventi (cadute, tocchi, pulsante...)

Cliccando una riga della tabella si seleziona il giunto, che si puo' poi
visualizzare da solo nel grafico. E' un'app di sola lettura: non muove il robot.

Dipendenze:  pip install matplotlib   +   qi (Linux / macOS)  /  libqi-windows (Windows)
"""

import csv
import functools
import math
import os
import queue
import threading
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
import tkinter as tk

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.dates as mdates
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import qi
except ImportError:
    qi = None

# ----------------------------------------------------------------- Configurazione
SD = "Device/SubDeviceList"

# Per ogni gruppo: (etichetta, chiave ALMemory, tipo di formattazione)
SENSOR_GROUPS = {
    "Batteria": [
        ("Carica", f"{SD}/Battery/Charge/Sensor/Value", "pct"),
        ("Corrente", f"{SD}/Battery/Current/Sensor/Value", ("A", 2)),
        ("Temperatura", f"{SD}/Battery/Temperature/Sensor/Value", ("°C", 0)),
        ("Caricatore collegato", "BatteryPowerPluggedChanged", "bool"),
    ],
    "Sensori inerziali (IMU)": [
        ("Angolo X (rollio)", f"{SD}/InertialSensor/AngleX/Sensor/Value", "deg"),
        ("Angolo Y (beccheggio)", f"{SD}/InertialSensor/AngleY/Sensor/Value", "deg"),
        ("Accelerometro X", f"{SD}/InertialSensor/AccelerometerX/Sensor/Value", ("m/s²", 2)),
        ("Accelerometro Y", f"{SD}/InertialSensor/AccelerometerY/Sensor/Value", ("m/s²", 2)),
        ("Accelerometro Z", f"{SD}/InertialSensor/AccelerometerZ/Sensor/Value", ("m/s²", 2)),
        ("Giroscopio X", f"{SD}/InertialSensor/GyroscopeX/Sensor/Value", ("rad/s", 2)),
        ("Giroscopio Y", f"{SD}/InertialSensor/GyroscopeY/Sensor/Value", ("rad/s", 2)),
        ("Giroscopio Z", f"{SD}/InertialSensor/GyroscopeZ/Sensor/Value", ("rad/s", 2)),
    ],
    "Piede sinistro (FSR)": [
        ("Anteriore sx", f"{SD}/LFoot/FSR/FrontLeft/Sensor/Value", ("kg", 2)),
        ("Anteriore dx", f"{SD}/LFoot/FSR/FrontRight/Sensor/Value", ("kg", 2)),
        ("Posteriore sx", f"{SD}/LFoot/FSR/RearLeft/Sensor/Value", ("kg", 2)),
        ("Posteriore dx", f"{SD}/LFoot/FSR/RearRight/Sensor/Value", ("kg", 2)),
        ("Peso totale", f"{SD}/LFoot/FSR/TotalWeight/Sensor/Value", ("kg", 2)),
    ],
    "Piede destro (FSR)": [
        ("Anteriore sx", f"{SD}/RFoot/FSR/FrontLeft/Sensor/Value", ("kg", 2)),
        ("Anteriore dx", f"{SD}/RFoot/FSR/FrontRight/Sensor/Value", ("kg", 2)),
        ("Posteriore sx", f"{SD}/RFoot/FSR/RearLeft/Sensor/Value", ("kg", 2)),
        ("Posteriore dx", f"{SD}/RFoot/FSR/RearRight/Sensor/Value", ("kg", 2)),
        ("Peso totale", f"{SD}/RFoot/FSR/TotalWeight/Sensor/Value", ("kg", 2)),
    ],
    "Sonar (serve 'Attiva sonar')": [
        ("Sonar sinistro", f"{SD}/US/Left/Sensor/Value", ("m", 2)),
        ("Sonar destro", f"{SD}/US/Right/Sensor/Value", ("m", 2)),
    ],
    "Tattili e contatti": [
        ("Testa - davanti", "FrontTactilTouched", "bool"),
        ("Testa - centro", "MiddleTactilTouched", "bool"),
        ("Testa - dietro", "RearTactilTouched", "bool"),
        ("Mano sinistra - dorso", "HandLeftBackTouched", "bool"),
        ("Mano sinistra - sinistra", "HandLeftLeftTouched", "bool"),
        ("Mano sinistra - destra", "HandLeftRightTouched", "bool"),
        ("Mano destra - dorso", "HandRightBackTouched", "bool"),
        ("Mano destra - sinistra", "HandRightLeftTouched", "bool"),
        ("Mano destra - destra", "HandRightRightTouched", "bool"),
        ("Paraurti sinistro", "LeftBumperPressed", "bool"),
        ("Paraurti destro", "RightBumperPressed", "bool"),
        ("Pulsante petto", "ChestButtonPressed", "bool"),
        ("Contatto con il suolo", "FootContact", "bool"),
    ],
}

SENSOR_KEYS = [k for items in SENSOR_GROUPS.values() for (_, k, _) in items]
K_CHARGE = f"{SD}/Battery/Charge/Sensor/Value"
K_CURRENT = f"{SD}/Battery/Current/Sensor/Value"
K_PLUGGED = "BatteryPowerPluggedChanged"

# Eventi ALMemory scritti nel log
EVENTS = [
    "robotHasFallen", "ChestButtonPressed", "FrontTactilTouched", "MiddleTactilTouched",
    "RearTactilTouched", "LeftBumperPressed", "RightBumperPressed",
    "BatteryPowerPluggedChanged", "PostureChanged", "robotIsWakeUp",
]

LEVEL_NAMES = {1: "FATAL", 2: "ERROR", 3: "WARN", 4: "INFO", 5: "VERB", 6: "DEBUG"}
LEVEL_CHOICES = [("Errori", 2), ("Avvisi", 3), ("Info", 4), ("Verbose", 5), ("Debug", 6)]

JOINT_GROUPS = {
    "Tutti": None,
    "Giunto selezionato": "SEL",
    "Testa": ("Head",),
    "Braccio sinistro": ("LShoulder", "LElbow", "LWrist", "LHand"),
    "Braccio destro": ("RShoulder", "RElbow", "RWrist", "RHand"),
    "Gamba sinistra": ("LHip", "LKnee", "LAnkle"),
    "Gamba destra": ("RHip", "RKnee", "RAnkle"),
}

NEUTRAL = "#b0bec5"
GREEN, YELLOW, RED = (76, 175, 80), (255, 193, 7), (229, 57, 53)


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def blend(c1, c2, f):
    f = clamp(f)
    return "#%02x%02x%02x" % tuple(int(a + (b - a) * f) for a, b in zip(c1, c2))


def ramp(f):
    """0 = verde, 0.5 = giallo, 1 = rosso."""
    return blend(GREEN, YELLOW, f * 2) if f < 0.5 else blend(YELLOW, RED, (f - 0.5) * 2)


def fmt(kind, v):
    if v is None:
        return "--"
    if kind == "bool":
        return "SÌ" if v > 0.5 else "no"
    if kind == "deg":
        return f"{math.degrees(v):.1f} °"
    if kind == "pct":
        return f"{v * 100:.0f} %"
    unit, dec = kind
    return f"{v:.{dec}f} {unit}"


def to_float(v):
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


# ----------------------------------------------------------------------- Worker
class Worker(threading.Thread):
    """Legge il robot a intervalli regolari e inoltra tutto alla GUI tramite coda."""

    def __init__(self, ip, port, interval, sonar, log_level, out_q):
        super().__init__(daemon=True)
        self.ip, self.port, self.interval = ip, port, interval
        self.sonar, self.log_level, self.q = sonar, log_level, out_q
        self.stop_event = threading.Event()
        self.session = None
        self.listener = None
        self.log_conn = None
        self.subs = []

    def _svc(self, name):
        try:
            return self.session.service(name)
        except Exception:
            return None

    @staticmethod
    def _try(fn):
        try:
            return fn()
        except Exception:
            return None

    @staticmethod
    def _read_keys(mem, keys):
        try:
            return list(mem.getListData(keys))
        except Exception:
            out = []
            for k in keys:
                try:
                    out.append(mem.getData(k))
                except Exception:
                    out.append(None)
            return out

    # -- log del robot
    def _on_log(self, m):
        try:
            if isinstance(m, dict):
                level, cat, text = m.get("level"), m.get("category"), m.get("message")
            else:
                level, cat, text = m[1], m[3], m[5]
            self.q.put(("log", (datetime.now(), int(level), str(cat), str(text).strip())))
        except Exception:
            pass

    def _on_event(self, key, value):
        self.q.put(("log", (datetime.now(), 4, "evento", f"{key} = {value}")))

    def apply_verbosity(self, level):
        """Cambia il livello di dettaglio dei log inviati dal robot."""
        if not self.listener:
            return
        attempts = (
            lambda: self.listener.setVerbosity(level),
            lambda: self.listener.setLevel(level),
            lambda: self.listener.verbosity.set(level),
            lambda: self.listener.setProperty("verbosity", level),
        )
        for attempt in attempts:
            try:
                attempt()
                return
            except Exception:
                continue

    def _setup_logs(self):
        try:
            lm = self.session.service("LogManager")
            self.listener = lm.getListener()
            self.apply_verbosity(self.log_level)
            self.log_conn = self.listener.onLogMessage.connect(self._on_log)
        except Exception as e:
            self.listener = None
            self.q.put(("log", (datetime.now(), 3, "app", f"Log del robot non disponibili: {e}")))

    def _setup_events(self, mem):
        for key in EVENTS:
            try:
                sub = mem.subscriber(key)
                cid = sub.signal.connect(functools.partial(self._on_event, key))
                self.subs.append((sub, cid))
            except Exception:
                pass

    def _cleanup(self, sonar):
        for sub, cid in self.subs:
            self._try(lambda: sub.signal.disconnect(cid))
        if self.listener and self.log_conn is not None:
            self._try(lambda: self.listener.onLogMessage.disconnect(self.log_conn))
        if sonar:
            self._try(lambda: sonar.unsubscribe("NaoDashboard"))

    def run(self):
        sonar = None
        try:
            self.session = qi.Session()
            self.session.connect(f"tcp://{self.ip}:{self.port}")
            mem = self.session.service("ALMemory")
            motion = self.session.service("ALMotion")
            names = list(motion.getBodyNames("Body"))
            limits = self._try(lambda: motion.getLimits("Body")) or [[-3.14, 3.14, 0, 0]] * len(names)
        except Exception as e:
            self.q.put(("error", f"Connessione fallita: {e}"))
            return

        system = self._svc("ALSystem")
        posture_svc = self._svc("ALRobotPosture")
        life = self._svc("ALAutonomousLife")

        info = {
            "name": self._try(lambda: system.robotName()),
            "version": self._try(lambda: system.systemVersion()),
        }
        self.q.put(("init", {"names": names, "limits": limits, "info": info}))
        self.q.put(("status", f"Connesso a {self.ip}:{self.port} ({len(names)} giunti)"))

        self._setup_logs()
        self._setup_events(mem)
        if self.sonar:
            sonar = self._svc("ALSonar")
            if sonar:
                self._try(lambda: sonar.subscribe("NaoDashboard"))

        temp_keys = [f"{SD}/{n}/Temperature/Sensor/Value" for n in names]
        cur_keys = [f"{SD}/{n}/ElectricCurrent/Sensor/Value" for n in names]
        keys = temp_keys + cur_keys + SENSOR_KEYS
        n = len(names)

        try:
            while not self.stop_event.is_set():
                angles = list(motion.getAngles("Body", True))
                stiff = self._try(lambda: list(motion.getStiffnesses("Body"))) or [None] * n
                vals = self._read_keys(mem, keys)
                sample = {
                    "t": datetime.now(),
                    "angles": angles,
                    "stiff": [to_float(x) for x in stiff],
                    "temps": [to_float(x) for x in vals[:n]],
                    "currents": [to_float(x) for x in vals[n:2 * n]],
                    "sensors": {k: to_float(v) for k, v in zip(SENSOR_KEYS, vals[2 * n:])},
                    "posture": self._try(lambda: posture_svc.getPostureFamily()),
                    "awake": self._try(lambda: motion.robotIsWakeUp()),
                    "life": self._try(lambda: life.getState()),
                }
                self.q.put(("sample", sample))
                self.stop_event.wait(self.interval)
        except Exception as e:
            self.q.put(("error", f"Lettura fallita (connessione persa?): {e}"))
        finally:
            self._cleanup(sonar)

    def stop(self):
        self.stop_event.set()


# -------------------------------------------------------------------------- GUI
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NAO Dashboard")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{min(1600, sw - 60)}x{min(940, sh - 100)}+20+20")
        self.minsize(1000, 600)

        style = ttk.Style(self)
        style.configure("Treeview", rowheight=20)
        style.configure("TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))

        self.q = queue.Queue()
        self.worker = None
        self.names, self.limits = [], []
        self.last = None
        self.sel = None
        self._sync = False
        self.joint_iid, self.group_iid, self.group_members = {}, {}, {}
        self.hist = deque(maxlen=20000)   # (datenum, [temps])
        self.logs = deque(maxlen=5000)    # (datetime, level, categoria, testo)
        self.alert_warn, self.alert_crit = set(), set()
        self.chart_dirty = False
        self.csv_file = self.csv_writer = None

        self._build_ui()
        self.after(200, self._poll_queue)
        self.after(300, self._set_sashes)
        self.after(2000, self._redraw_chart)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- Costruzione UI
    def _build_ui(self):
        top = ttk.Frame(self, padding=(8, 8, 8, 2))
        top.pack(fill="x")
        self.ip_var = tk.StringVar(value="192.168.1.100")
        self.port_var = tk.StringVar(value="9559")
        self.int_var = tk.StringVar(value="1")
        self.warn_var = tk.StringVar(value="60")
        self.crit_var = tk.StringVar(value="70")
        self.sonar_var = tk.BooleanVar(value=False)
        self.csv_var = tk.BooleanVar(value=True)

        def field(label, var, width):
            ttk.Label(top, text=label).pack(side="left", padx=(0, 3))
            ttk.Entry(top, textvariable=var, width=width).pack(side="left", padx=(0, 8))

        field("IP robot", self.ip_var, 15)
        field("Porta", self.port_var, 6)
        field("Intervallo (s)", self.int_var, 4)
        field("Avviso °C", self.warn_var, 4)
        field("Critico °C", self.crit_var, 4)
        ttk.Checkbutton(top, text="Attiva sonar", variable=self.sonar_var).pack(side="left", padx=4)
        ttk.Checkbutton(top, text="Registra CSV", variable=self.csv_var).pack(side="left", padx=4)
        self.btn = ttk.Button(top, text="Connetti", command=self.toggle_connection)
        self.btn.pack(side="left", padx=8)
        self.status = tk.StringVar(value="Non connesso")
        ttk.Label(top, textvariable=self.status).pack(side="left", padx=8)

        main = self.main = ttk.PanedWindow(self, orient="horizontal")
        main.pack(fill="both", expand=True, padx=6, pady=6)
        col_a = ttk.PanedWindow(main, orient="vertical")
        col_b = ttk.PanedWindow(main, orient="vertical")
        col_c = ttk.PanedWindow(main, orient="vertical")
        main.add(col_a, weight=5)
        main.add(col_b, weight=3)
        main.add(col_c, weight=3)

        col_a.add(self._build_overview(col_a), weight=1)
        col_a.add(self._build_joints(col_a), weight=6)
        col_b.add(self._build_chart(col_b), weight=3)
        col_b.add(self._build_log(col_b), weight=2)
        col_c.add(self._build_sensors(col_c), weight=1)

    def _build_overview(self, parent):
        f = ttk.LabelFrame(parent, text="Panoramica", padding=6)
        self.ov = {}
        items = [
            ("robot", "Robot"), ("posture", "Postura"),
            ("awake", "Motori"), ("life", "Life"),
            ("charge", "Batteria"), ("current", "Corr. batt."),
            ("plugged", "Caricatore"), ("hot", "Più caldo"),
            ("relaxed", "Rilassati"),
        ]
        for i, (key, title) in enumerate(items):
            r, c = divmod(i, 2)
            ttk.Label(f, text=title, foreground="gray").grid(row=r, column=c * 2, sticky="w", padx=(0, 6), pady=1)
            lab = ttk.Label(f, text="--", font=("TkDefaultFont", 10, "bold"))
            lab.grid(row=r, column=c * 2 + 1, sticky="w", padx=(0, 16), pady=1)
            self.ov[key] = lab
        self.ov_alerts = ttk.Label(f, text="Nessun avviso", foreground="green", wraplength=340, justify="left",
                                   font=("TkDefaultFont", 10, "bold"))
        self.ov_alerts.grid(row=(len(items) + 1) // 2, column=0, columnspan=4, sticky="w", pady=(6, 0))
        f.bind("<Configure>", lambda e: self.ov_alerts.config(wraplength=max(120, e.width - 24)))
        return f

    def _build_joints(self, parent):
        f = ttk.LabelFrame(parent, text="Giunti", padding=4)
        cols = ("angle", "min", "max", "range", "stiff", "temp", "cur", "state")
        self.jtree = ttk.Treeview(f, columns=cols, show="tree headings", height=10, selectmode="browse")
        self.jtree.heading("#0", text="Giunto")
        self.jtree.column("#0", width=150, minwidth=120)
        for c, title, w in [("angle", "Angolo", 70), ("min", "Min", 55), ("max", "Max", 55),
                            ("range", "Range", 55), ("stiff", "Rigidità", 65), ("temp", "Temp.", 60),
                            ("cur", "Corrente", 70), ("state", "Stato", 65)]:
            self.jtree.heading(c, text=title)
            self.jtree.column(c, width=w, minwidth=40, anchor="center")
        self.jtree.tag_configure("group", background="#e3e8ec", foreground="black", font=("TkDefaultFont", 9, "bold"))
        self.jtree.tag_configure("warn", background="#ffe2a0", foreground="black")
        self.jtree.tag_configure("crit", background="#ffb3b3", foreground="black")
        self.jtree.bind("<<TreeviewSelect>>", self._on_table_select)
        sb = ttk.Scrollbar(f, orient="vertical", command=self.jtree.yview)
        self.jtree.configure(yscrollcommand=sb.set)
        self.jtree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        return f

    def _build_chart(self, parent):
        f = ttk.LabelFrame(parent, text="Temperature nel tempo", padding=4)
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        self.grp_var = tk.StringVar(value="Tutti")
        self.cwin_var = tk.StringVar(value="Tutto")
        ttk.Label(bar, text="Gruppo").pack(side="left", padx=(0, 3))
        cb = ttk.Combobox(bar, textvariable=self.grp_var, values=list(JOINT_GROUPS), state="readonly", width=18)
        cb.pack(side="left", padx=(0, 10))
        ttk.Label(bar, text="Finestra").pack(side="left", padx=(0, 3))
        cw = ttk.Combobox(bar, textvariable=self.cwin_var, values=["Tutto", "5 min", "15 min", "1 ora"],
                          state="readonly", width=8)
        cw.pack(side="left")
        for c in (cb, cw):
            c.bind("<<ComboboxSelected>>", lambda e: self._mark_chart())

        self.fig = Figure(figsize=(5, 3), dpi=100, tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=4)
        return f

    def _build_sensors(self, parent):
        f = ttk.LabelFrame(parent, text="Sensori", padding=4)
        self.stree = ttk.Treeview(f, columns=("value",), show="tree headings", height=8)
        self.stree.heading("#0", text="Sensore")
        self.stree.heading("value", text="Valore")
        self.stree.column("#0", width=210)
        self.stree.column("value", width=110, anchor="center")
        self.stree.tag_configure("active", background="#bfeebf", foreground="black")
        self.sens_iid, self.sens_kind = {}, {}
        for group, items in SENSOR_GROUPS.items():
            gid = self.stree.insert("", "end", text=group, open="Piede" not in group)
            for label, key, kind in items:
                self.sens_iid[key] = self.stree.insert(gid, "end", text=label, values=("--",))
                self.sens_kind[key] = kind
        sb = ttk.Scrollbar(f, orient="vertical", command=self.stree.yview)
        self.stree.configure(yscrollcommand=sb.set)
        self.stree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        return f

    def _build_log(self, parent):
        f = ttk.LabelFrame(parent, text="Log", padding=4)
        bar = ttk.Frame(f)
        bar.pack(fill="x", pady=(0, 4))
        self.lvl_var = tk.StringVar(value="Info")
        self.filter_var = tk.StringVar()
        self.autoscroll = tk.BooleanVar(value=True)

        lv = ttk.Combobox(bar, textvariable=self.lvl_var, values=[n for n, _ in LEVEL_CHOICES],
                          state="readonly", width=8)
        lv.pack(side="left", padx=(0, 6))
        lv.bind("<<ComboboxSelected>>", self._on_level_change)
        e = ttk.Entry(bar, textvariable=self.filter_var, width=14)
        e.pack(side="left", padx=(0, 6))
        e.bind("<KeyRelease>", lambda ev: self._render_logs())
        ttk.Checkbutton(bar, text="Auto-scroll", variable=self.autoscroll).pack(side="left", padx=2)
        ttk.Button(bar, text="Pulisci", width=7, command=self._clear_logs).pack(side="left", padx=2)
        ttk.Button(bar, text="Salva...", width=7, command=self._save_logs).pack(side="left", padx=2)

        self.txt = tk.Text(f, wrap="word", state="disabled", font=("Courier", 9), height=8)
        sb = ttk.Scrollbar(f, orient="vertical", command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)
        for lvl, color in {1: "#b00020", 2: "#d32f2f", 3: "#e68900", 4: "#222222", 5: "#555555", 6: "#888888"}.items():
            self.txt.tag_configure(LEVEL_NAMES[lvl], foreground=color)
        return f

    # ------------------------------------------------------------------ Layout
    def _set_sashes(self):
        """Proporzioni iniziali delle tre colonne (modificabili trascinando i divisori)."""
        w = self.main.winfo_width()
        if w > 300:
            self.main.sashpos(0, int(w * 0.44))
            self.main.sashpos(1, int(w * 0.73))

    # ------------------------------------------------------------ Selezione giunto
    def select_joint(self, name, from_table=False):
        if name == self.sel and from_table:
            return
        self.sel = name
        if not from_table and name in self.joint_iid:
            self._sync = True
            self.jtree.selection_set(self.joint_iid[name])
            self.jtree.see(self.joint_iid[name])
            self._sync = False
        if JOINT_GROUPS.get(self.grp_var.get()) == "SEL":
            self.chart_dirty = True

    def _on_table_select(self, _event=None):
        if self._sync:
            return
        sel = self.jtree.selection()
        if sel:
            name = self.jtree.item(sel[0], "text")
            if name in self.names:   # ignora le righe dei gruppi
                self.select_joint(name, from_table=True)

    # ----------------------------------------------------------------- Connessione
    def toggle_connection(self):
        self.disconnect() if self.worker else self.connect()

    def connect(self):
        if qi is None:
            messagebox.showerror("Modulo mancante",
                                 "Modulo 'qi' non trovato.\nWindows: pip install libqi-windows\nLinux / macOS: pip install qi")
            return
        try:
            ip, port = self.ip_var.get().strip(), int(self.port_var.get())
            interval = max(0.2, float(self.int_var.get().replace(",", ".")))
        except ValueError:
            messagebox.showerror("Errore", "Controlla IP, porta e intervallo.")
            return
        level = dict(LEVEL_CHOICES)[self.lvl_var.get()]
        self.worker = Worker(ip, port, interval, self.sonar_var.get(), level, self.q)
        self.worker.start()
        self.btn.config(text="Disconnetti")
        self.status.set(f"Connessione a {ip}:{port}...")

    def disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self._close_csv()
        self.btn.config(text="Connetti")
        self.status.set("Non connesso")

    # ------------------------------------------------------------------------- CSV
    def _open_csv(self):
        os.makedirs("nao_logs", exist_ok=True)
        path = os.path.join("nao_logs", datetime.now().strftime("nao_robot_%Y%m%d_%H%M%S.csv"))
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        header = ["timestamp", "battery_pct"]
        for prefix in ("temp_c", "angle_deg", "current_a"):
            header += [f"{prefix}_{n}" for n in self.names]
        self.csv_writer.writerow(header)

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = self.csv_writer = None

    def _write_csv(self, s):
        if not self.csv_writer:
            return
        ch = s["sensors"].get(K_CHARGE)
        row = [s["t"].isoformat(timespec="seconds"), "" if ch is None else f"{ch * 100:.1f}"]
        row += ["" if v is None else v for v in s["temps"]]
        row += [f"{math.degrees(a):.2f}" for a in s["angles"]]
        row += ["" if v is None else v for v in s["currents"]]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    # ------------------------------------------------------------------- Coda eventi
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "sample":
                    self._on_sample(payload)
                elif kind == "init":
                    self._on_init(payload)
                elif kind == "log":
                    self._add_log(*payload)
                elif kind == "status":
                    self.status.set(payload)
                elif kind == "error":
                    self.disconnect()
                    self.status.set(payload)
                    messagebox.showwarning("NAO Dashboard", payload)
        except queue.Empty:
            pass
        self.after(200, self._poll_queue)

    def _on_init(self, d):
        self.names, self.limits = d["names"], d["limits"]
        info = d["info"]
        self.ov["robot"].config(text=f"{info.get('name') or '?'} (v{info.get('version') or '?'})")
        self.jtree.delete(*self.jtree.get_children())
        self.joint_iid, self.group_iid, self.group_members = {}, {}, {}
        group_prefixes = {g: pre for g, pre in JOINT_GROUPS.items() if isinstance(pre, tuple)}
        for i, name in enumerate(self.names):
            group = next((g for g, pre in group_prefixes.items() if name.startswith(pre)), "Altro")
            if group not in self.group_iid:
                self.group_iid[group] = self.jtree.insert("", "end", text=group, open=True,
                                                          values=("",) * 8, tags=("group",))
                self.group_members[group] = []
            self.group_members[group].append(i)
            lo, hi = self.limits[i][0], self.limits[i][1]
            self.joint_iid[name] = self.jtree.insert(
                self.group_iid[group], "end", text=name,
                values=("--", f"{math.degrees(lo):.0f}°", f"{math.degrees(hi):.0f}°", "--", "--", "--", "--", "--"))
        self.hist.clear()
        self.alert_warn.clear()
        self.alert_crit.clear()
        self.last = None
        self.sel = None
        if self.csv_var.get():
            self._open_csv()

    # ----------------------------------------------------------------- Aggiornamenti
    def _thresholds(self):
        try:
            return float(self.warn_var.get()), float(self.crit_var.get())
        except ValueError:
            return 60.0, 70.0

    def _on_sample(self, s):
        warn, crit = self._thresholds()
        self.last = s
        self.hist.append((mdates.date2num(s["t"]), s["temps"]))
        self.chart_dirty = True

        for i, name in enumerate(self.names):
            ang = s["angles"][i]
            lo, hi = self.limits[i][0], self.limits[i][1]
            frac = 0.5 if hi == lo else clamp((ang - lo) / (hi - lo))
            t, cur, st = s["temps"][i], s["currents"][i], s["stiff"][i]
            tag, state = (), "OK"
            if t is None:
                state = "--"
            elif t >= crit:
                tag, state = ("crit",), "CRITICO"
            elif t >= warn:
                tag, state = ("warn",), "AVVISO"
            vals = list(self.jtree.item(self.joint_iid[name], "values"))
            vals[0] = f"{math.degrees(ang):.1f}°"
            vals[3] = f"{frac * 100:.0f} %"
            vals[4] = "--" if st is None else f"{st * 100:.0f} %"
            vals[5] = "--" if t is None else f"{t:.0f} °C"
            vals[6] = "--" if cur is None else f"{cur:.2f} A"
            vals[7] = state
            self.jtree.item(self.joint_iid[name], tags=tag, values=vals)

        # riga di gruppo: temperatura massima e stato peggiore
        for group, members in self.group_members.items():
            temps = [s["temps"][i] for i in members if s["temps"][i] is not None]
            vals = ["", "", "", "", "", "", "", ""]
            tag = ("group",)
            if temps:
                tmax = max(temps)
                vals[5] = f"{tmax:.0f} °C"
                if tmax >= crit:
                    vals[7], tag = "CRITICO", ("group", "crit")
                elif tmax >= warn:
                    vals[7], tag = "AVVISO", ("group", "warn")
                else:
                    vals[7] = "OK"
            self.jtree.item(self.group_iid[group], values=vals, tags=tag)

        for key, v in s["sensors"].items():
            kind = self.sens_kind[key]
            self.stree.item(self.sens_iid[key], values=(fmt(kind, v),),
                            tags=("active",) if kind == "bool" and v is not None and v > 0.5 else ())

        self._update_overview(s, warn)
        self._check_alerts(s, warn, crit)
        self._write_csv(s)

    def _update_overview(self, s, warn):
        sens = s["sensors"]
        self.ov["posture"].config(text=str(s["posture"] or "--"))
        awake = s["awake"]
        self.ov["awake"].config(text="--" if awake is None else ("attivi" if awake else "rilassati"))
        self.ov["life"].config(text=str(s["life"] or "--"))
        self.ov["charge"].config(text=fmt("pct", sens.get(K_CHARGE)))
        self.ov["current"].config(text=fmt(("A", 2), sens.get(K_CURRENT)))
        self.ov["plugged"].config(text=fmt("bool", sens.get(K_PLUGGED)))

        temps = [(t, n) for t, n in zip(s["temps"], self.names) if t is not None]
        if temps:
            t, n = max(temps)
            self.ov["hot"].config(text=f"{n} {t:.0f} °C")
        relaxed = sum(1 for x in s["stiff"] if x is not None and x < 0.05)
        self.ov["relaxed"].config(text=f"{relaxed} / {len(self.names)}")

        hot = [f"{n} {t:.0f} °C" for t, n in temps if t >= warn]
        if hot:
            self.ov_alerts.config(text="Temperatura alta: " + ", ".join(hot), foreground="#d32f2f")
        else:
            self.ov_alerts.config(text="Nessun avviso", foreground="green")

    def _check_alerts(self, s, warn, crit):
        """Scrive nel log quando un giunto supera le soglie (una sola volta per superamento)."""
        for name, t in zip(self.names, s["temps"]):
            if t is None:
                continue
            if t >= crit and name not in self.alert_crit:
                self.alert_crit.add(name)
                self.alert_warn.add(name)
                self._add_log(datetime.now(), 2, "app", f"{name}: {t:.0f} °C, sopra la soglia critica ({crit:.0f})")
            elif t >= warn and name not in self.alert_warn:
                self.alert_warn.add(name)
                self._add_log(datetime.now(), 3, "app", f"{name}: {t:.0f} °C, sopra la soglia di avviso ({warn:.0f})")
            elif t < warn - 2:
                self.alert_warn.discard(name)
                self.alert_crit.discard(name)

    # ---------------------------------------------------------------- Grafico temperature
    def _mark_chart(self):
        self.chart_dirty = True
        self._redraw_chart(reschedule=False)

    def _redraw_chart(self, reschedule=True):
        if self.chart_dirty and self.hist and self.names:
            self.chart_dirty = False
            rows = list(self.hist)
            minutes = {"5 min": 5, "15 min": 15, "1 ora": 60}.get(self.cwin_var.get())
            if minutes:
                cutoff = rows[-1][0] - minutes / 1440.0
                rows = [r for r in rows if r[0] >= cutoff]
            if len(rows) > 2500:
                rows = rows[::len(rows) // 2500 + 1]
            prefixes = JOINT_GROUPS[self.grp_var.get()]
            if prefixes == "SEL":
                idxs = [self.names.index(self.sel)] if self.sel in self.names else []
            else:
                idxs = [i for i, n in enumerate(self.names) if prefixes is None or n.startswith(prefixes)]
            x = [r[0] for r in rows]
            warn, crit = self._thresholds()

            self.ax.clear()
            for i in idxs:
                y = [r[1][i] if r[1][i] is not None else float("nan") for r in rows]
                if any(v == v for v in y):
                    self.ax.plot(x, y, label=self.names[i], linewidth=1.2)
            self.ax.axhline(warn, color="orange", linestyle="--", linewidth=0.9)
            self.ax.axhline(crit, color="red", linestyle="--", linewidth=0.9)
            self.ax.set_ylabel("°C", fontsize=9)
            self.ax.tick_params(labelsize=8)
            self.ax.grid(True, alpha=0.3)
            self.ax.xaxis_date()
            self.ax.xaxis.set_major_locator(mdates.AutoDateLocator(minticks=2, maxticks=4))
            self.ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            handles = self.ax.get_legend_handles_labels()[0]
            if 0 < len(handles) <= 8:   # con molte linee la legenda coprirebbe il grafico
                self.ax.legend(loc="upper left", ncol=2, fontsize=7)
            self.canvas.draw_idle()
        if reschedule:
            self.after(2000, self._redraw_chart)

    # ------------------------------------------------------------------------- Log
    def _min_level(self):
        return dict(LEVEL_CHOICES)[self.lvl_var.get()]

    def _passes(self, entry):
        _, level, cat, text = entry
        if cat not in ("evento", "app") and level > self._min_level():
            return False
        needle = self.filter_var.get().strip().lower()
        return not needle or needle in text.lower() or needle in cat.lower()

    def _insert_log(self, entry):
        ts, level, cat, text = entry
        name = LEVEL_NAMES.get(level, "INFO")
        self.txt.insert("end", f"{ts:%H:%M:%S} {name:<5} {cat}: {text}\n", name)

    def _add_log(self, ts, level, cat, text):
        entry = (ts, level, cat, text)
        self.logs.append(entry)
        if self._passes(entry):
            self.txt.config(state="normal")
            self._insert_log(entry)
            self.txt.config(state="disabled")
            if self.autoscroll.get():
                self.txt.see("end")

    def _render_logs(self):
        self.txt.config(state="normal")
        self.txt.delete("1.0", "end")
        for entry in self.logs:
            if self._passes(entry):
                self._insert_log(entry)
        self.txt.config(state="disabled")
        if self.autoscroll.get():
            self.txt.see("end")

    def _on_level_change(self, _event=None):
        self._render_logs()
        if self.worker:
            level = self._min_level()
            threading.Thread(target=self.worker.apply_verbosity, args=(level,), daemon=True).start()

    def _clear_logs(self):
        self.logs.clear()
        self._render_logs()

    def _save_logs(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Testo", "*.txt")])
        if path:
            with open(path, "w", encoding="utf-8") as f:
                for ts, level, cat, text in self.logs:
                    f.write(f"{ts:%Y-%m-%d %H:%M:%S}  {LEVEL_NAMES.get(level, 'INFO'):<5} {cat}: {text}\n")

    def _on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
