#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ulog_auswertung.py
==================
Auswertungsskript fuer PX4-ULog-Dateien eines Testflugs mit Vision-gestuetzter
GNSS-freier Lokalisierung (External Vision / EKF2_EV_CTRL).

Schwerpunkte:
  * EKF-Zustaende (Position, Geschwindigkeit, Lage, Sensor-Biases)
  * Vision-Vorgabe (External-Vision) vs. EKF-Schaetzung vs. Offboard-Sollwert
  * Tatsaechlich geflogene Bahn (2D / 3D / Hoehe)
  * Fusions-Gesundheit (Innovationen, Test-Ratios, Fused/Rejected, Aussetzer)
  * EKF-Unsicherheit (Kovarianz) und Vertrauen in neue Messungen
  * Flugmodi, Arming, Land-Detector

In JEDEM Diagramm werden Arming-Zeitpunkt (gruen) und Landung (rot) markiert,
jedes Diagramm hat eine eigene Zeitachse und eine einheitliche Legende.

Aufruf:
    # (a) ganzer Flug:
    python3 ulog_auswertung.py <pfad/zur/datei.ulg>

    # (b) nur ein Zeitfenster (Sekunden seit Logstart), Plots werden skaliert:
    python3 ulog_auswertung.py <pfad/zur/datei.ulg> --tmin 240 --tmax 266

    # weitere Optionen:
    python3 ulog_auswertung.py <datei.ulg> --out ordner/ --instance 0 --dropout-schwelle 1.0

Welche Plots erzeugt werden, steuerst du im Konfigurationsblock PLOTS_AKTIV
direkt unter den Imports (True = an, False = aus).

Abhaengigkeiten:
    pip install pyulog numpy matplotlib
"""

import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")  # Headless, schreibt nur Dateien
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

try:
    from pyulog import ULog
except ImportError:
    sys.exit("Fehler: pyulog nicht installiert. -> pip install pyulog")


# ===========================================================================
# KONFIGURATION: Plots aktivieren (True) oder deaktivieren (False)
# ===========================================================================
PLOTS_AKTIV = {
    "01_position_vision_ekf_sollwert": True,
    "02_bahn_2d_3d": True,
    "03_geschwindigkeit": True,
    "04_lage": True,
    "05_test_ratios": True,
    "06_ev_innovationen": True,
    "07_ekf_biases": True,
    "08_fusion_status": True,
    "09_flugmodi": True,
    "10_aussetzer_resets": True,
    "11_kovarianz": True,
    "12_vertrauen": True,
}

# Einheitliche Darstellung
LEGENDE_LOC = "upper right"     # einheitliche Legendenposition pro Subplot
EREIGNIS_FARBEN = {"Armed": "green", "Landung": "red"}
# ===========================================================================


NAV_STATE_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 10: "ACRO", 12: "DESCEND",
    13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
    18: "AUTO_LAND", 19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND", 21: "ORBIT",
}


# ---------------------------------------------------------------------------
# Basis-Hilfsfunktionen
# ---------------------------------------------------------------------------
def lade_ulog(pfad):
    """Laedt eine ULog-Datei und gibt das ULog-Objekt zurueck.

    Args:
        pfad: Pfad zur .ulg-Datei.

    Returns:
        ULog-Objekt.
    """
    if not os.path.isfile(pfad):
        sys.exit(f"Fehler: Datei nicht gefunden: {pfad}")
    return ULog(pfad)


def hole(ulog, name, mid=0):
    """Gibt den Datensatz eines Topics (mit Multi-Instanz-ID) zurueck oder None.

    Args:
        ulog: ULog-Objekt.
        name: Topic-Name.
        mid: Multi-Instanz-ID (Standard 0).

    Returns:
        pyulog-Datenobjekt oder None.
    """
    for d in ulog.data_list:
        if d.name == name and d.multi_id == mid:
            return d
    return None


def zeit(ulog, datensatz):
    """Rechnet Zeitstempel in Sekunden seit Logstart um (uint64-sicher).

    Args:
        ulog: ULog-Objekt.
        datensatz: pyulog-Datenobjekt.

    Returns:
        numpy-Array mit Zeit in Sekunden.
    """
    t0 = np.int64(ulog.start_timestamp)
    return (datensatz.data["timestamp"].astype(np.int64) - t0) / 1e6


def quat_zu_euler(w, x, y, z):
    """Wandelt Quaternionen (w, x, y, z) in Euler-Winkel (roll, pitch, yaw) [rad].

    Args:
        w, x, y, z: Quaternion-Komponenten (skalar-zuerst).

    Returns:
        Tupel (roll, pitch, yaw) in Radiant.
    """
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def voller_zeitraum(ulog):
    """Ermittelt den gesamten Zeitraum des Logs (auf Basis vehicle_local_position).

    Args:
        ulog: ULog-Objekt.

    Returns:
        Tupel (t_start, t_ende) in Sekunden.
    """
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    return float(t[0]), float(t[-1])


def airborne_fenster(ulog):
    """Ermittelt das Zeitfenster, in dem die Drohne in der Luft war.

    Args:
        ulog: ULog-Objekt.

    Returns:
        Tupel (t_start, t_ende) in Sekunden oder (None, None).
    """
    ld = hole(ulog, "vehicle_land_detected")
    if ld is None:
        return None, None
    t = zeit(ulog, ld)
    luft = t[ld.data["landed"] == 0]
    if len(luft) == 0:
        return None, None
    return float(luft[0]), float(luft[-1])


def ereigniszeiten(ulog):
    """Ermittelt die Zeitpunkte von Arming und Landung.

    Args:
        ulog: ULog-Objekt.

    Returns:
        Dict {"Armed": t oder None, "Landung": t oder None} in Sekunden.
    """
    out = {"Armed": None, "Landung": None}
    vs = hole(ulog, "vehicle_status")
    if vs is not None:
        t = zeit(ulog, vs)
        m = (vs.data["arming_state"] == 2) & (t >= 0)
        if m.any():
            out["Armed"] = float(t[m][0])
    ld = hole(ulog, "vehicle_land_detected")
    if ld is not None:
        t = zeit(ulog, ld)
        landed = ld.data["landed"].astype(int)
        tr = np.where(np.diff(landed) == 1)[0] + 1  # Uebergang in der Luft -> gelandet
        if len(tr):
            out["Landung"] = float(t[tr[-1]])
    return out


def finde_ev_aussetzer(ulog, inst, schwelle_s, fenster=None):
    """Findet Aussetzer (Luecken) in der External-Vision-Fusion.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        schwelle_s: Mindestluecke in Sekunden ab der ein Aussetzer zaehlt.
        fenster: Optionales (t0, t1); nur Aussetzer mit Ueberlapp werden zurueckgegeben.

    Returns:
        Liste von (t_start, t_ende, dauer) in Sekunden.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    if ev is None:
        return []
    t = zeit(ulog, ev)
    tf = t[ev.data["fused"].astype(bool)]
    if len(tf) < 2:
        return []
    luecken = np.diff(tf)
    idx = np.where(luecken > schwelle_s)[0]
    res = [(float(tf[i]), float(tf[i + 1]), float(luecken[i])) for i in idx]
    if fenster is not None:
        res = [a for a in res if a[1] >= fenster[0] and a[0] <= fenster[1]]
    return res


def finde_resets(ulog, inst, fenster=None):
    """Ermittelt die Zeitpunkte der EKF-Resets.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        fenster: Optionales (t0, t1) zum Filtern.

    Returns:
        Dict {Bezeichnung: numpy-Array mit Reset-Zeitpunkten [s]}.
    """
    es = hole(ulog, "estimator_status", inst)
    if es is None:
        return {}
    t = zeit(ulog, es)
    felder = {"Pos NE": "reset_count_pos_ne", "Pos D": "reset_count_pod_d",
              "Vel NE": "reset_count_vel_ne", "Quat": "reset_count_quat"}
    out = {}
    for name, feld in felder.items():
        if feld in es.data:
            c = es.data[feld].astype(np.int64)
            rt = t[np.where(np.diff(c) > 0)[0] + 1]
            if fenster is not None:
                rt = rt[(rt >= fenster[0]) & (rt <= fenster[1])]
            out[name] = rt
    return out


def maske(t, fenster):
    """Boolesche Maske der Zeitpunkte innerhalb des Fensters.

    Args:
        t: Zeit-Array in Sekunden.
        fenster: (t0, t1).

    Returns:
        Boolesches numpy-Array.
    """
    return (t >= fenster[0]) & (t <= fenster[1])


# ---------------------------------------------------------------------------
# Darstellungs-Helfer (Marker, Legende, Zeitachse)
# ---------------------------------------------------------------------------
def _markiere_zeit(ax, ereignisse, fenster):
    """Zeichnet vertikale Marker fuer Arming/Landung in eine Zeitachse.

    Args:
        ax: matplotlib-Achse.
        ereignisse: Dict {Name: Zeit}.
        fenster: (t0, t1) zur Sichtbarkeitspruefung.
    """
    for name, t in ereignisse.items():
        if t is None or not (fenster[0] <= t <= fenster[1]):
            continue
        ln = ax.axvline(t, color=EREIGNIS_FARBEN.get(name, "gray"),
                        ls=":", lw=1.6, alpha=0.9, label=name)
        ln.set_gid("_marker")  # von der y-Autoskalierung ausschliessen


def _markiere_ort(ax, ulog, ereignisse, fenster, dim3=False):
    """Markiert die Arming-/Landungs-Position in einem raeumlichen Plot.

    Args:
        ax: matplotlib-Achse (2D oder 3D).
        ulog: ULog-Objekt.
        ereignisse: Dict {Name: Zeit}.
        fenster: (t0, t1) zur Sichtbarkeitspruefung.
        dim3: True fuer 3D-Achsen.
    """
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    for name, te in ereignisse.items():
        if te is None or not (fenster[0] <= te <= fenster[1]):
            continue
        i = int(np.argmin(np.abs(t - te)))
        x, y, z = vlp.data["x"][i], vlp.data["y"][i], vlp.data["z"][i]
        c = EREIGNIS_FARBEN.get(name, "gray")
        if dim3:
            ax.scatter([x], [y], [-z], c=c, s=70, depthshade=False, label=name)
        else:
            ax.plot([y], [x], marker="o", color=c, ms=10, ls="", label=name)


def _legende(ax, loc=LEGENDE_LOC):
    """Setzt eine einheitliche Legende, falls beschriftete Elemente vorhanden sind.

    Args:
        ax: matplotlib-Achse.
        loc: Legendenposition.
    """
    h, l = ax.get_legend_handles_labels()
    if h:
        ax.legend(h, l, loc=loc, fontsize=8, framealpha=0.9,
                  ncol=1 if len(h) <= 4 else 2)


def _autoscale_y(ax, fenster):
    """Skaliert die y-Achse auf die im Zeitfenster sichtbaren Datenlinien.

    Marker-Linien (gid '_marker') werden ignoriert.

    Args:
        ax: matplotlib-Achse.
        fenster: (t0, t1).
    """
    ymin, ymax = np.inf, -np.inf
    for ln in ax.get_lines():
        if ln.get_gid() == "_marker":
            continue
        x = np.asarray(ln.get_xdata(), dtype=float)
        y = np.asarray(ln.get_ydata(), dtype=float)
        if x.size == 0:
            continue
        m = (x >= fenster[0]) & (x <= fenster[1]) & np.isfinite(y)
        if m.any():
            ymin = min(ymin, float(np.min(y[m])))
            ymax = max(ymax, float(np.max(y[m])))
    if np.isfinite(ymin) and np.isfinite(ymax) and ymax > ymin:
        pad = 0.06 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)


def _finalisiere_zeit(ax, fenster, ereignisse, autoscale_y=True):
    """Vereinheitlicht eine Zeitachsen-Achse: Marker, x-Limits, x-Achse, Legende.

    Args:
        ax: matplotlib-Achse.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        autoscale_y: Ob die y-Achse auf das Fenster skaliert werden soll.
    """
    _markiere_zeit(ax, ereignisse, fenster)
    ax.set_xlim(*fenster)
    if autoscale_y:
        _autoscale_y(ax, fenster)
    ax.set_xlabel("Zeit [s]")                 # Zeitachse unter jedem Diagramm
    ax.tick_params(labelbottom=True)
    ax.grid(True, alpha=0.3)
    _legende(ax)


def _schattiere_aussetzer(ax, aussetzer, label=True):
    """Schattiert EV-Aussetzer als orange Bereiche.

    Args:
        ax: matplotlib-Achse.
        aussetzer: Liste von (t_start, t_ende, dauer).
        label: Ob ein Legendeneintrag gesetzt wird.
    """
    for i, (ta, te, _) in enumerate(aussetzer):
        ax.axvspan(ta, te, color="orange", alpha=0.25,
                   label="EV-Aussetzer" if (label and i == 0) else None)


# ---------------------------------------------------------------------------
# Plot-Funktionen  (einheitliche Signatur: ulog, inst, fenster, ereignisse, schwelle)
# ---------------------------------------------------------------------------
def plot_position_vision_ekf_sollwert(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Position (N/E/D): Vision-Beobachtung vs. EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt) Aussetzer-Schwelle.

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    ev_pos = hole(ulog, "estimator_aid_src_ev_pos", inst)
    ev_hgt = hole(ulog, "estimator_aid_src_ev_hgt", inst)

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    labels = ["Nord  x [m]", "Ost  y [m]", "Unten  z [m] (NED)"]
    t_est = zeit(ulog, vlp)
    est = [vlp.data["x"], vlp.data["y"], vlp.data["z"]]
    t_sp = zeit(ulog, ts) if ts is not None else None

    for i, ax in enumerate(axes):
        ax.plot(t_est, est[i], color="C0", lw=1.4, label="EKF-Schaetzung")
        if ts is not None:
            ax.plot(t_sp, ts.data[f"position[{i}]"], color="C3", lw=1.0,
                    ls="--", label="Sollwert")
        if i < 2 and ev_pos is not None:
            ax.plot(zeit(ulog, ev_pos), ev_pos.data[f"observation[{i}]"],
                    color="C2", lw=0.0, marker=".", ms=2.5, alpha=0.6,
                    label="Vision (EV)")
        if i == 2 and ev_hgt is not None:
            ax.plot(zeit(ulog, ev_hgt), ev_hgt.data["observation"],
                    color="C2", lw=0.0, marker=".", ms=2.5, alpha=0.6,
                    label="Vision (EV)")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Position: Vision-Vorgabe vs. EKF-Schaetzung vs. Sollwert",
                 fontsize=13, fontweight="bold")
    return fig


def plot_bahn_2d_3d(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die geflogene Bahn: Draufsicht (XY), Hoehenverlauf und 3D.

    Layout: oben links XY, oben rechts Hoehe(t), unten 3D ueber volle Breite
    (verhindert die Achsenbeschriftungs-Ueberschneidungen des alten Layouts).

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    t = zeit(ulog, vlp)
    m = maske(t, fenster)
    x, y, z = vlp.data["x"][m], vlp.data["y"][m], vlp.data["z"][m]

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.15],
                          hspace=0.30, wspace=0.22,
                          left=0.08, right=0.95, top=0.92, bottom=0.08)

    # Draufsicht XY
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(y, x, color="C0", lw=1.2, label="EKF-Bahn")
    if ts is not None:
        ms = maske(zeit(ulog, ts), fenster)
        ax1.plot(ts.data["position[1]"][ms], ts.data["position[0]"][ms],
                 color="C3", lw=0.9, ls="--", label="Sollwert")
    _markiere_ort(ax1, ulog, ereignisse, fenster)
    ax1.set_xlabel("Ost  y [m]")
    ax1.set_ylabel("Nord  x [m]")
    ax1.set_title("Draufsicht (XY)")
    ax1.axis("equal")
    ax1.grid(True, alpha=0.3)
    _legende(ax1, "best")

    # Hoehenverlauf (Zeitachse)
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.plot(t, -vlp.data["z"], color="C0", lw=1.3, label="EKF-Hoehe")
    if ts is not None:
        ax3.plot(zeit(ulog, ts), -ts.data["position[2]"], color="C3",
                 lw=0.9, ls="--", label="Soll-Hoehe")
    ax3.set_ylabel("Hoehe -z [m]")
    ax3.set_title("Hoehenverlauf")
    _finalisiere_zeit(ax3, fenster, ereignisse)

    # 3D ueber volle Breite
    ax2 = fig.add_subplot(gs[1, :], projection="3d")
    ax2.plot(x, y, -z, color="C0", lw=1.0, label="EKF-Bahn")
    _markiere_ort(ax2, ulog, ereignisse, fenster, dim3=True)
    ax2.set_xlabel("Nord x [m]", labelpad=10)
    ax2.set_ylabel("Ost y [m]", labelpad=10)
    ax2.zaxis.set_rotate_label(False)
    ax2.set_zlabel("Hoehe -z [m]", labelpad=10, rotation=90)
    ax2.tick_params(labelsize=8, pad=2)
    ax2.set_title("3D-Bahn")
    _legende(ax2, "upper left")

    fig.suptitle("Geflogene Bahn", fontsize=13, fontweight="bold")
    return fig


def plot_geschwindigkeit(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Geschwindigkeit (vx, vy, vz): EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ts = hole(ulog, "trajectory_setpoint")
    t = zeit(ulog, vlp)
    v = [vlp.data["vx"], vlp.data["vy"], vlp.data["vz"]]
    labels = ["vx (Nord) [m/s]", "vy (Ost) [m/s]", "vz (Unten) [m/s]"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    for i, ax in enumerate(axes):
        ax.plot(t, v[i], color="C0", lw=1.2, label="EKF")
        if ts is not None:
            ax.plot(zeit(ulog, ts), ts.data[f"velocity[{i}]"], color="C3",
                    lw=0.9, ls="--", label="Sollwert")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Geschwindigkeit: EKF vs. Sollwert", fontsize=13, fontweight="bold")
    return fig


def plot_lage(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Lage (Roll, Pitch, Yaw): EKF vs. Sollwert.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    att = hole(ulog, "vehicle_attitude")
    asp = hole(ulog, "vehicle_attitude_setpoint")
    t = zeit(ulog, att)
    r, p, yw = quat_zu_euler(att.data["q[0]"], att.data["q[1]"],
                             att.data["q[2]"], att.data["q[3]"])
    est = [np.degrees(r), np.degrees(p), np.degrees(yw)]
    sp = None
    if asp is not None:
        rs, ps, yws = quat_zu_euler(asp.data["q_d[0]"], asp.data["q_d[1]"],
                                    asp.data["q_d[2]"], asp.data["q_d[3]"])
        sp = [np.degrees(rs), np.degrees(ps), np.degrees(yws)]
        t_sp = zeit(ulog, asp)
    labels = ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"]

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    for i, ax in enumerate(axes):
        ax.plot(t, est[i], color="C0", lw=1.2, label="EKF")
        if sp is not None:
            ax.plot(t_sp, sp[i], color="C3", lw=0.9, ls="--", label="Sollwert")
        ax.set_ylabel(labels[i])
        _finalisiere_zeit(ax, fenster, ereignisse)
    fig.suptitle("Lage: EKF vs. Sollwert", fontsize=13, fontweight="bold")
    return fig


def plot_test_ratios(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die Innovation-Test-Ratios der EKF-Aiding-Quellen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    tr = hole(ulog, "estimator_innovation_test_ratios", inst)
    t = zeit(ulog, tr)
    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    for feld, name in [("ev_hpos[0]", "EV Pos x"), ("ev_hpos[1]", "EV Pos y"),
                       ("ev_vpos", "EV Pos z"), ("baro_vpos", "Baro Hoehe"),
                       ("heading", "Heading")]:
        if feld in tr.data:
            ax.plot(t, tr.data[feld], lw=1.0, label=name)
    ax.axhline(1.0, color="k", ls="--", lw=1.2, label="Schwelle 1.0")
    ax.set_ylabel("Test-Ratio [-]")
    ax.set_title("Innovation-Test-Ratios (Verwerfung ab 1.0)", fontweight="bold")
    _finalisiere_zeit(ax, fenster, ereignisse)
    return fig


def plot_ev_innovationen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet EV-Innovationen mit +/-1-Sigma-Grenzen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    ev_pos = hole(ulog, "estimator_aid_src_ev_pos", inst)
    ev_hgt = hole(ulog, "estimator_aid_src_ev_hgt", inst)
    ev_yaw = hole(ulog, "estimator_aid_src_ev_yaw", inst)
    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True,
                             constrained_layout=True)

    def _innov(ax, t, innov, var, titel):
        """Zeichnet eine Innovation mit +/-1-Sigma-Huellkurve und skaliert manuell."""
        sig = np.sqrt(np.abs(var))
        ax.fill_between(t, -sig, sig, color="C0", alpha=0.2, label="+/-1 sigma")
        ax.plot(t, innov, color="C1", lw=0.8, label="Innovation")
        ax.set_ylabel(titel)
        m = maske(t, fenster)
        if m.any():
            lim = 1.2 * np.nanmax(np.abs(np.concatenate([sig[m], innov[m]])))
            if np.isfinite(lim) and lim > 0:
                ax.set_ylim(-lim, lim)
        _finalisiere_zeit(ax, fenster, ereignisse, autoscale_y=False)

    if ev_pos is not None:
        t = zeit(ulog, ev_pos)
        _innov(axes[0], t, ev_pos.data["innovation[0]"],
               ev_pos.data["innovation_variance[0]"], "EV Pos x [m]")
        _innov(axes[1], t, ev_pos.data["innovation[1]"],
               ev_pos.data["innovation_variance[1]"], "EV Pos y [m]")
    if ev_hgt is not None:
        t = zeit(ulog, ev_hgt)
        _innov(axes[2], t, ev_hgt.data["innovation"],
               ev_hgt.data["innovation_variance"], "EV Hoehe [m]")
    if ev_yaw is not None:
        t = zeit(ulog, ev_yaw)
        _innov(axes[3], t, ev_yaw.data["innovation"],
               ev_yaw.data["innovation_variance"], "EV Yaw [rad]")
    fig.suptitle("External-Vision-Innovationen mit Konsistenzgrenzen",
                 fontsize=13, fontweight="bold")
    return fig


def plot_biases(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die geschaetzten Gyro- und Accel-Biases.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    st = hole(ulog, "estimator_states", inst)
    t = zeit(ulog, st)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), sharex=True,
                                   constrained_layout=True)
    for i, a in enumerate(["x", "y", "z"]):
        ax1.plot(t, np.degrees(st.data[f"states[{10 + i}]"]), lw=1.0,
                 label=f"Gyro-Bias {a}")
        ax2.plot(t, st.data[f"states[{13 + i}]"], lw=1.0, label=f"Accel-Bias {a}")
    ax1.set_ylabel("Gyro-Bias [deg/s]")
    ax2.set_ylabel("Accel-Bias [m/s^2]")
    _finalisiere_zeit(ax1, fenster, ereignisse)
    _finalisiere_zeit(ax2, fenster, ereignisse)
    fig.suptitle("EKF Sensor-Bias-Schaetzungen", fontsize=13, fontweight="bold")
    return fig


def plot_fusion_status(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet EV-Fusionsstatus, Beobachtungsvarianz und aktive Aiding-Quellen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (fuer Schattierung).

    Returns:
        matplotlib-Figure.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    flags = hole(ulog, "estimator_status_flags", inst)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)
    t = zeit(ulog, ev)

    axes[0].plot(t, ev.data["fused"], color="C2", lw=0.8, label="fused")
    axes[0].plot(t, ev.data["innovation_rejected"], color="C3", lw=0.8,
                 label="rejected")
    axes[0].set_ylabel("Status [0/1]")
    axes[0].set_ylim(-0.1, 1.1)
    _finalisiere_zeit(axes[0], fenster, ereignisse, autoscale_y=False)

    axes[1].plot(t, np.sqrt(ev.data["observation_variance[0]"]), lw=0.9,
                 label="sigma_obs x")
    axes[1].plot(t, np.sqrt(ev.data["observation_variance[1]"]), lw=0.9,
                 label="sigma_obs y")
    _schattiere_aussetzer(axes[1], aussetzer)
    axes[1].set_ylabel("EV sigma_obs [m]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    if flags is not None:
        tf = zeit(ulog, flags)
        for feld, name in [("cs_ev_pos", "EV Pos"), ("cs_ev_yaw", "EV Yaw"),
                           ("cs_ev_hgt", "EV Hgt"), ("cs_baro_hgt", "Baro Hgt"),
                           ("cs_in_air", "in air")]:
            if feld in flags.data:
                axes[2].plot(tf, flags.data[feld], lw=0.9, label=name)
        axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_ylabel("aktiv [0/1]")
    _finalisiere_zeit(axes[2], fenster, ereignisse, autoscale_y=False)
    fig.suptitle("EKF Fusions-Status (External Vision)", fontsize=13, fontweight="bold")
    return fig


def plot_flugmodi(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Nav-State, Arming-State und Land-Detector.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: (ungenutzt).

    Returns:
        matplotlib-Figure.
    """
    vs = hole(ulog, "vehicle_status")
    ld = hole(ulog, "vehicle_land_detected")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                   constrained_layout=True)
    t = zeit(ulog, vs)
    ax1.step(t, vs.data["nav_state"], where="post", color="C0", label="Flugmodus")
    werte = sorted(set(vs.data["nav_state"]))
    ax1.set_yticks(werte)
    ax1.set_yticklabels([NAV_STATE_NAMES.get(int(v), str(v)) for v in werte])
    ax1.set_ylabel("Flugmodus")
    _finalisiere_zeit(ax1, fenster, ereignisse, autoscale_y=False)

    ax2.step(t, vs.data["arming_state"], where="post", color="C1", label="Arming")
    if ld is not None:
        ax2.step(zeit(ulog, ld), ld.data["landed"], where="post",
                 color="C2", label="landed")
    ax2.set_yticks([0, 1, 2])
    ax2.set_ylabel("Arming / landed")
    _finalisiere_zeit(ax2, fenster, ereignisse, autoscale_y=False)
    fig.suptitle("Flugmodi und Arming", fontsize=13, fontweight="bold")
    return fig


def plot_aussetzer_resets(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet Hoehe, EV-Updates, EV-Aussetzer und EKF-Resets.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle in Sekunden.

    Returns:
        matplotlib-Figure.
    """
    vlp = hole(ulog, "vehicle_local_position")
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    tv = zeit(ulog, vlp)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    resets = finde_resets(ulog, inst, fenster)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), constrained_layout=True)

    def _zeichne(ax, tmin, tmax, titel):
        """Zeichnet Hoehe, EV-Updates, Aussetzer und Resets in ein Zeitfenster."""
        ax.plot(tv, -vlp.data["z"], "C0-", lw=1.5, label="Hoehe -z [m]")
        if ev is not None:
            te = zeit(ulog, ev)
            tf = te[ev.data["fused"].astype(bool)]
            ax.plot(tf, np.full(len(tf), -0.15), "g|", ms=10, label="EV fused")
        _schattiere_aussetzer(ax, [a for a in aussetzer
                                   if a[1] >= tmin and a[0] <= tmax])
        erst = True
        for r in resets.get("Pos NE", []):
            if tmin <= r <= tmax:
                ax.axvline(r, color="magenta", lw=1.3,
                           label="Pos-Reset" if erst else None).set_gid("_marker")
                erst = False
        ax.set_ylabel("Hoehe -z [m]")
        ax.set_title(titel)
        _finalisiere_zeit(ax, (tmin, tmax), ereignisse)

    _zeichne(ax1, fenster[0], fenster[1], "Uebersicht")
    rne = resets.get("Pos NE", np.array([]))
    if len(rne) > 1:
        zmin = max(fenster[0], rne.min() - 5)
        zmax = min(fenster[1], rne.max() + 5)
    else:
        zmin, zmax = fenster
    _zeichne(ax2, zmin, zmax, f"Zoom Reset-Bereich ({zmin:.0f}-{zmax:.0f} s)")
    fig.suptitle("EV-Aussetzer und EKF-Resets", fontsize=13, fontweight="bold")
    return fig


def plot_kovarianz(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet die EKF-Unsicherheit (1-Sigma) aus der Kovarianz-Diagonale.

    Kovarianz-Index (PX4 EKF2, 24 Zustaende):
        0-3 Quaternion, 4-6 Geschw. NED, 7-9 Pos NED,
        10-12 Gyro-Bias, 13-15 Accel-Bias, 16-21 Mag, 22-23 Wind.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (fuer Schattierung).

    Returns:
        matplotlib-Figure.
    """
    st = hole(ulog, "estimator_states", inst)
    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, st)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)

    def sig(i):
        """1-Sigma aus Kovarianz-Index i."""
        return np.sqrt(np.abs(st.data[f"covariances[{i}]"]))

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True,
                             constrained_layout=True)

    for i, lab in zip([7, 8, 9], ["Nord", "Ost", "Unten"]):
        axes[0].plot(t, sig(i), lw=1.1, label=f"sigma Pos {lab}")
    if vlp is not None:
        tv = zeit(ulog, vlp)
        axes[0].plot(tv, vlp.data["eph"], "k:", lw=1.0, alpha=0.7, label="eph")
        axes[0].plot(tv, vlp.data["epv"], "k--", lw=1.0, alpha=0.7, label="epv")
    _schattiere_aussetzer(axes[0], aussetzer)
    axes[0].set_ylabel("Position 1-sigma [m]")
    # robuste Obergrenze (Ausreisser nach dem Aufsetzen ausblenden), fensterbezogen
    m = maske(t, fenster)
    werte = np.concatenate([sig(7)[m], sig(8)[m], sig(9)[m]]) if m.any() else sig(7)
    og = np.nanpercentile(werte, 99) * 1.4
    if np.isfinite(og) and og > 0:
        axes[0].set_ylim(0, max(og, 0.1))
    _finalisiere_zeit(axes[0], fenster, ereignisse, autoscale_y=False)

    for i, lab in zip([4, 5, 6], ["Nord", "Ost", "Unten"]):
        axes[1].plot(t, sig(i), lw=1.1, label=f"sigma Vel {lab}")
    _schattiere_aussetzer(axes[1], aussetzer, label=False)
    axes[1].set_ylabel("Geschw. 1-sigma [m/s]")
    _finalisiere_zeit(axes[1], fenster, ereignisse)

    for i, lab in zip([10, 11, 12], ["x", "y", "z"]):
        axes[2].plot(t, np.degrees(sig(i)), lw=1.0, label=f"Gyro {lab} [deg/s]")
    for i, lab in zip([13, 14, 15], ["x", "y", "z"]):
        axes[2].plot(t, sig(i), lw=1.0, ls="--", label=f"Accel {lab} [m/s^2]")
    axes[2].set_ylabel("Bias 1-sigma")
    _finalisiere_zeit(axes[2], fenster, ereignisse)
    fig.suptitle("EKF-Unsicherheit (Kovarianz-Diagonale, 1-sigma)",
                 fontsize=13, fontweight="bold")
    return fig


def plot_vertrauen(ulog, inst, fenster, ereignisse, schwelle):
    """Plottet das Kalman-Gewicht: wie stark der EKF neue EV-Messungen korrigiert.

    K = P / (P + R) = 1 - R / S  (R = observation_variance, S = innovation_variance).
    K nahe 0: EKF selbst sehr sicher, Messung kaum gewichtet.
    K nahe 1: EKF unsicher, Messung wird fast vollstaendig uebernommen.

    Args:
        ulog: ULog-Objekt.
        inst: EKF-Instanz.
        fenster: (t0, t1).
        ereignisse: Dict {Name: Zeit}.
        schwelle: Aussetzer-Schwelle (fuer Schattierung).

    Returns:
        matplotlib-Figure.
    """
    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    evh = hole(ulog, "estimator_aid_src_ev_hgt", inst)
    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle, fenster)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 9), sharex=True,
                                   constrained_layout=True)
    t = zeit(ulog, ev)
    for k, lab in [(0, "EV Pos x"), (1, "EV Pos y")]:
        K = np.clip(1.0 - ev.data[f"observation_variance[{k}]"]
                    / ev.data[f"innovation_variance[{k}]"], 0.0, 1.0)
        ax1.plot(t, K, lw=1.0, label=lab)
    if evh is not None:
        Kz = np.clip(1.0 - evh.data["observation_variance"]
                     / evh.data["innovation_variance"], 0.0, 1.0)
        ax1.plot(zeit(ulog, evh), Kz, lw=1.0, label="EV Hoehe")
    _schattiere_aussetzer(ax1, aussetzer)
    ax1.set_ylabel("Kalman-Gewicht K [-]")
    ax1.set_ylim(-0.05, 1.05)
    ax1.set_title("Korrekturanteil pro Update (0 = ignoriert, 1 = uebernommen)")
    _finalisiere_zeit(ax1, fenster, ereignisse, autoscale_y=False)

    ax2.plot(t, np.sqrt(ev.data["observation_variance[0]"]), lw=1.1,
             label="sigma Messung (R)")
    ax2.plot(t, np.sqrt(ev.data["innovation_variance[0]"]), lw=1.1,
             label="sigma Innovation (S = P + R)")
    _schattiere_aussetzer(ax2, aussetzer, label=False)
    ax2.set_ylabel("1-sigma [m] (x-Kanal)")
    _finalisiere_zeit(ax2, fenster, ereignisse)
    fig.suptitle("EKF-Vertrauen in die Vision-Vorgabe", fontsize=13, fontweight="bold")
    return fig


# Registry: Name -> Funktion (Reihenfolge wie im Report)
PLOT_FUNCS = {
    "01_position_vision_ekf_sollwert": plot_position_vision_ekf_sollwert,
    "02_bahn_2d_3d": plot_bahn_2d_3d,
    "03_geschwindigkeit": plot_geschwindigkeit,
    "04_lage": plot_lage,
    "05_test_ratios": plot_test_ratios,
    "06_ev_innovationen": plot_ev_innovationen,
    "07_ekf_biases": plot_biases,
    "08_fusion_status": plot_fusion_status,
    "09_flugmodi": plot_flugmodi,
    "10_aussetzer_resets": plot_aussetzer_resets,
    "11_kovarianz": plot_kovarianz,
    "12_vertrauen": plot_vertrauen,
}


# ---------------------------------------------------------------------------
# Text-Zusammenfassung
# ---------------------------------------------------------------------------
def textzusammenfassung(ulog, inst, fenster, schwelle_s):
    """Erstellt eine textuelle Kennzahlen-Zusammenfassung fuer das Zeitfenster.

    Args:
        ulog: ULog-Objekt.
        inst: Primaere EKF-Instanz-ID.
        fenster: (t0, t1) ausgewerteter Zeitraum.
        schwelle_s: Schwelle fuer die Aussetzer-Erkennung in Sekunden.

    Returns:
        String mit der Zusammenfassung.
    """
    L = []
    def p(s=""):
        """Haengt eine Zeile an."""
        L.append(s)

    vlp = hole(ulog, "vehicle_local_position")
    t = zeit(ulog, vlp)
    m = maske(t, fenster)
    voll = voller_zeitraum(ulog)
    p("=" * 64)
    p("  FLUG-ZUSAMMENFASSUNG")
    p("=" * 64)
    p(f"Logdauer gesamt         : {voll[1] - voll[0]:.1f} s")
    p(f"Ausgewerteter Zeitraum  : {fenster[0]:.1f} .. {fenster[1]:.1f} s")
    ev_ereig = ereigniszeiten(ulog)
    p(f"Arming / Landung        : "
      f"{ev_ereig['Armed']:.1f} s / {ev_ereig['Landung']:.1f} s"
      if ev_ereig["Armed"] is not None and ev_ereig["Landung"] is not None
      else "Arming/Landung: n/a")

    x, y, z = vlp.data["x"][m], vlp.data["y"][m], vlp.data["z"][m]
    if len(x) > 1:
        strecke = np.nansum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2))
        p(f"Max. Hoehe (-z)         : {-np.nanmin(z):.2f} m")
        p(f"Bahn-Ausdehnung x/y/z   : {np.nanmax(x)-np.nanmin(x):.2f} / "
          f"{np.nanmax(y)-np.nanmin(y):.2f} / {np.nanmax(z)-np.nanmin(z):.2f} m")
        p(f"Zurueckgelegte Strecke  : {strecke:.1f} m")

    vs = hole(ulog, "vehicle_status")
    nav = vs.data["nav_state"][maske(zeit(ulog, vs), fenster)]
    if len(nav):
        p("")
        p("Flugmodus-Anteile (im Zeitraum):")
        for v in sorted(set(nav)):
            p(f"   {NAV_STATE_NAMES.get(int(v), str(v)):16s}: {100*np.mean(nav==v):5.1f} %")

    ev = hole(ulog, "estimator_aid_src_ev_pos", inst)
    if ev is not None:
        me = maske(zeit(ulog, ev), fenster)
        if me.any():
            p("")
            p("External-Vision-Fusion (Position):")
            p(f"   fused-Anteil         : {100*np.mean(ev.data['fused'][me]):.1f} %")
            p(f"   rejected-Anteil      : {100*np.mean(ev.data['innovation_rejected'][me]):.2f} %")
            tr = np.maximum(ev.data["test_ratio[0]"][me], ev.data["test_ratio[1]"][me])
            p(f"   Test-Ratio mittel/max: {np.nanmean(tr):.4f} / {np.nanmax(tr):.4f}")

    aussetzer = finde_ev_aussetzer(ulog, inst, schwelle_s, fenster)
    p("")
    p(f"EV-Aussetzer (Luecke > {schwelle_s:.1f} s):")
    if aussetzer:
        p(f"   Anzahl               : {len(aussetzer)} "
          f"(gesamt {sum(a[2] for a in aussetzer):.1f} s ohne Vision)")
        for ta, te, d in aussetzer:
            p(f"   {ta:7.1f} .. {te:7.1f} s   (Dauer {d:.2f} s)")
    else:
        p("   keine")

    resets = finde_resets(ulog, inst, fenster)
    p("")
    p("EKF-Resets (Zeitpunkte im Zeitraum):")
    for name in ["Pos NE", "Pos D", "Vel NE", "Quat"]:
        rt = resets.get(name, np.array([]))
        zeiten = ", ".join(f"{x:.1f}" for x in rt) if len(rt) else "-"
        p(f"   {name:7s}: {len(rt):2d}  @ [{zeiten}] s")

    st = hole(ulog, "estimator_states", inst)
    if st is not None:
        ms = maske(zeit(ulog, st), fenster)
        if ms.any():
            sp = np.sqrt(np.abs(st.data["covariances[7]"]))[ms]
            sv = np.sqrt(np.abs(st.data["covariances[4]"]))[ms]
            p("")
            p("EKF-Unsicherheit (1-sigma):")
            p(f"   Position Nord        : mittel {np.nanmean(sp):.3f} m, max {np.nanmax(sp):.3f} m")
            p(f"   Geschwindigkeit Nord : mittel {np.nanmean(sv):.3f} m/s, max {np.nanmax(sv):.3f} m/s")
    if ev is not None and me.any():
        K = np.clip(1.0 - ev.data["observation_variance[0]"][me]
                    / ev.data["innovation_variance[0]"][me], 0.0, 1.0)
        p(f"   Kalman-Gewicht EV-Pos: median {np.nanmedian(K):.3f}, max {np.nanmax(K):.3f}")

    ts = hole(ulog, "trajectory_setpoint")
    if ts is not None and m.any():
        p("")
        p("Tracking-Fehler (Sollwert - EKF), RMS im Zeitraum:")
        t_sp = zeit(ulog, ts)
        for i, (ax, key) in enumerate(zip(["x", "y", "z"],
                                          ["position[0]", "position[1]", "position[2]"])):
            sp_i = np.interp(t[m], t_sp, ts.data[key])
            est_i = [x, y, z][i]
            ok = np.isfinite(sp_i) & np.isfinite(est_i)
            rms = np.sqrt(np.nanmean((sp_i[ok] - est_i[ok]) ** 2))
            p(f"   {ax}: {rms:.3f} m")
    p("=" * 64)
    return "\n".join(L)


# ---------------------------------------------------------------------------
# Hauptprogramm
# ---------------------------------------------------------------------------
def main():
    """Liest Argumente, erzeugt aktivierte Plots, PDF-Report und Textzusammenfassung."""
    parser = argparse.ArgumentParser(
        description="PX4-ULog-Auswertung fuer Vision-gestuetzten EKF-Flug.")
    parser.add_argument("ulog", help="Pfad zur .ulg-Datei")
    parser.add_argument("--out", default=None,
                        help="Ausgabeordner (Standard: <logname>_auswertung/)")
    parser.add_argument("--instance", type=int, default=None,
                        help="EKF-Instanz (Standard: primaere aus selector_status)")
    parser.add_argument("--tmin", type=float, default=None,
                        help="Startzeit des Auswertungsfensters [s] (Standard: Logstart)")
    parser.add_argument("--tmax", type=float, default=None,
                        help="Endzeit des Auswertungsfensters [s] (Standard: Logende)")
    parser.add_argument("--dropout-schwelle", type=float, default=1.0,
                        help="Luecke in s ab der ein EV-Aussetzer zaehlt (Standard 1.0)")
    args = parser.parse_args()

    ulog = lade_ulog(args.ulog)

    if args.instance is not None:
        inst = args.instance
    else:
        sel = hole(ulog, "estimator_selector_status")
        inst = int(sel.data["primary_instance"][-1]) if sel is not None else 0

    # Zeitfenster aufloesen: (a) ganzer Flug oder (b) --tmin/--tmax
    voll = voller_zeitraum(ulog)
    t0 = args.tmin if args.tmin is not None else voll[0]
    t1 = args.tmax if args.tmax is not None else voll[1]
    fenster = (t0, t1)
    gefenstert = (args.tmin is not None) or (args.tmax is not None)
    ereignisse = ereigniszeiten(ulog)

    if args.out is None:
        basis = os.path.splitext(os.path.basename(args.ulog))[0]
        args.out = f"{basis}_auswertung"
    os.makedirs(args.out, exist_ok=True)

    print(f"Auswertung laeuft ... (EKF-Instanz {inst})")
    print(f"Zeitraum: {t0:.1f} .. {t1:.1f} s"
          + ("  [Fenster aktiv]" if gefenstert else "  [ganzer Flug]"))
    print(f"Ausgabeordner: {os.path.abspath(args.out)}\n")

    aktive = [(n, f) for n, f in PLOT_FUNCS.items() if PLOTS_AKTIV.get(n, True)]
    pdf_pfad = os.path.join(args.out, "report.pdf")
    with PdfPages(pdf_pfad) as pdf:
        for name, fn in aktive:
            try:
                fig = fn(ulog, inst, fenster, ereignisse, args.dropout_schwelle)
                if gefenstert:
                    fig.text(0.99, 0.005, f"Zeitfenster {t0:.1f}-{t1:.1f} s",
                             ha="right", va="bottom", fontsize=7, alpha=0.5)
                fig.savefig(os.path.join(args.out, name + ".png"), dpi=130)
                pdf.savefig(fig)
                plt.close(fig)
                print(f"  [ok] {name}.png")
            except Exception as e:
                print(f"  [uebersprungen] {name}: {e}")

    txt = textzusammenfassung(ulog, inst, fenster, args.dropout_schwelle)
    print("\n" + txt)
    with open(os.path.join(args.out, "zusammenfassung.txt"), "w") as f:
        f.write(txt + "\n")
    print(f"\nFertig. PDF-Report: {pdf_pfad}")


if __name__ == "__main__":
    main()