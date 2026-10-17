#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compara execuções gravadas em rosbag (ex.: LIDAR vs sonar).

Uso:
    rosrun pioneer_reactive_nav analyze_runs.py \
        data/lidar_run1.bag data/sonar_run1.bag \
        --labels LIDAR SONAR --out results/

Gera: summary.csv, trajetorias.png, folga.png, comandos.png
Observação: como o bag grava LIDAR e sonar ao mesmo tempo, a "folga"
medida pelo LIDAR serve de referência para avaliar quase-colisões
também nas execuções feitas somente com sonar.
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def load_bag(path, topics, front_deg, lidar_offset):
    od = {"t": [], "x": [], "y": []}
    cm = {"t": [], "v": [], "w": []}
    sc = {"t": [], "front": [], "h90": []}
    so = {"t": [], "front": [], "h90": []}
    fr = math.radians(front_deg)

    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=list(topics.values())):
            ts = t.to_sec()
            if topic == topics["odom"]:
                od["t"].append(ts)
                od["x"].append(msg.pose.pose.position.x)
                od["y"].append(msg.pose.pose.position.y)
            elif topic == topics["cmd"]:
                cm["t"].append(ts)
                cm["v"].append(msg.linear.x)
                cm["w"].append(msg.angular.z)
            elif topic == topics["scan"]:
                r = np.asarray(msg.ranges, dtype=float)
                ang = msg.angle_min + np.arange(r.size) * msg.angle_increment + lidar_offset
                ang = np.arctan2(np.sin(ang), np.cos(ang))
                ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= msg.range_max)
                f = ok & (np.abs(ang) <= fr)
                h = ok & (np.abs(ang) <= math.pi / 2)
                sc["t"].append(ts)
                sc["front"].append(r[f].min() if f.any() else np.nan)
                sc["h90"].append(r[h].min() if h.any() else np.nan)
            elif topic == topics["sonar"]:
                so["t"].append(ts)
                if msg.points:
                    xy = np.array([[p.x, p.y] for p in msg.points])
                    d = np.hypot(xy[:, 0], xy[:, 1])
                    a = np.arctan2(xy[:, 1], xy[:, 0])
                    f = np.abs(a) <= fr
                    h = np.abs(a) <= math.pi / 2
                    so["front"].append(d[f].min() if f.any() else np.nan)
                    so["h90"].append(d[h].min() if h.any() else np.nan)
                else:
                    so["front"].append(np.nan)
                    so["h90"].append(np.nan)

    for d in (od, cm, sc, so):
        for k in d:
            d[k] = np.asarray(d[k], dtype=float)
    return od, cm, sc, so


def rate(t):
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float("nan")


def episodes(below):
    if below.size == 0:
        return 0
    return int(below[0]) + int(np.sum(below[1:] & ~below[:-1]))


def compute_metrics(label, od, cm, sc, so, near_thr, cell, arena):
    if od["t"].size < 2:
        raise SystemExit("[%s] sem mensagens de odometria no bag." % label)

    dur = od["t"][-1] - od["t"][0]
    dx, dy = np.diff(od["x"]), np.diff(od["y"])
    path = float(np.sum(np.hypot(dx, dy)))

    cx = np.floor((od["x"] - od["x"].min()) / cell).astype(int)
    cy = np.floor((od["y"] - od["y"].min()) / cell).astype(int)
    cells = len(set(zip(cx, cy)))
    total_cells = math.ceil(arena[0] / cell) * math.ceil(arena[1] / cell)

    m = {
        "run": label,
        "duracao_s": round(dur, 1),
        "distancia_percorrida_m": round(path, 2),
        "velocidade_media_m_s": round(path / dur, 3) if dur > 0 else float("nan"),
        "celulas_visitadas": cells,
        "cobertura_estimada_pct": round(100.0 * cells / total_cells, 1),
        "taxa_odom_hz": round(rate(od["t"]), 1),
        "taxa_scan_hz": round(rate(sc["t"]), 1),
        "taxa_sonar_hz": round(rate(so["t"]), 1),
    }
    if cm["t"].size:
        m["pct_tempo_parado_ou_girando"] = round(100.0 * float(np.mean(cm["v"] < 0.02)), 1)

    if sc["h90"].size:
        h = sc["h90"]
        below = np.nan_to_num(h, nan=np.inf) < near_thr
        dt = np.mean(np.diff(sc["t"])) if sc["t"].size > 1 else 0.0
        m["folga_min_lidar_m"] = round(float(np.nanmin(h)), 3)
        m["quase_colisoes_lidar"] = episodes(below)
        m["tempo_abaixo_%.2fm_s" % near_thr] = round(float(below.sum() * dt), 1)
    if so["h90"].size and not np.all(np.isnan(so["h90"])):
        m["folga_min_sonar_m"] = round(float(np.nanmin(so["h90"])), 3)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--labels", nargs="+", help="Rótulos (um por bag)")
    ap.add_argument("--out", default="results")
    ap.add_argument("--odom", default="/RosAria/pose")
    ap.add_argument("--cmd", default="/RosAria/cmd_vel")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--sonar", default="/RosAria/sonar")
    ap.add_argument("--front-deg", type=float, default=30.0)
    ap.add_argument("--lidar-offset", type=float, default=0.0)
    ap.add_argument("--near", type=float, default=0.30,
                    help="Limiar de quase-colisão [m] (padrão 0.30)")
    ap.add_argument("--cell", type=float, default=0.5, help="Tamanho da célula [m]")
    ap.add_argument("--arena", type=float, nargs=2, default=[7.0, 3.5])
    a = ap.parse_args()

    labels = a.labels or [os.path.splitext(os.path.basename(b))[0] for b in a.bags]
    if len(labels) != len(a.bags):
        raise SystemExit("--labels deve ter o mesmo número de bags.")
    os.makedirs(a.out, exist_ok=True)
    topics = {"odom": a.odom, "cmd": a.cmd, "scan": a.scan, "sonar": a.sonar}

    runs, rows = [], []
    for path, lab in zip(a.bags, labels):
        print("Lendo %s ..." % path)
        od, cm, sc, so = load_bag(path, topics, a.front_deg, a.lidar_offset)
        runs.append((lab, od, cm, sc, so))
        rows.append(compute_metrics(lab, od, cm, sc, so, a.near, a.cell, a.arena))

    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(a.out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print("\n== %s ==" % r["run"])
        for k, v in r.items():
            if k != "run":
                print("  %-32s %s" % (k, v))

    # Trajetórias
    plt.figure(figsize=(8, 5))
    for lab, od, *_ in runs:
        plt.plot(od["x"] - od["x"][0], od["y"] - od["y"][0], label=lab)
    plt.plot(0, 0, "ko", ms=4)
    plt.axis("equal"); plt.grid(True, alpha=.3); plt.legend()
    plt.xlabel("x [m] (relativo ao início)"); plt.ylabel("y [m]")
    plt.title("Trajetórias (odometria)")
    plt.tight_layout(); plt.savefig(os.path.join(a.out, "trajetorias.png"), dpi=150)
    plt.close()

    # Folga ao longo do tempo
    fig, axs = plt.subplots(len(runs), 1, figsize=(9, 3 * len(runs)), squeeze=False)
    for ax, (lab, od, cm, sc, so) in zip(axs[:, 0], runs):
        t0 = od["t"][0]
        if sc["t"].size:
            ax.plot(sc["t"] - t0, sc["h90"], lw=.8, label="LIDAR (mín. ±90°)")
        if so["t"].size:
            ax.plot(so["t"] - t0, so["h90"], lw=.8, label="Sonar (mín. ±90°)")
        ax.axhline(a.near, color="r", ls="--", lw=.8, label="limiar")
        ax.set_title("Execução: %s" % lab); ax.set_ylabel("Distância mín. [m]")
        ax.grid(True, alpha=.3); ax.legend(loc="upper right", fontsize=8)
    axs[-1, 0].set_xlabel("tempo [s]")
    plt.tight_layout(); plt.savefig(os.path.join(a.out, "folga.png"), dpi=150)
    plt.close()

    # Comandos
    fig, axs = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    for lab, od, cm, *_ in runs:
        if cm["t"].size:
            axs[0].plot(cm["t"] - od["t"][0], cm["v"], lw=.8, label=lab)
            axs[1].plot(cm["t"] - od["t"][0], cm["w"], lw=.8, label=lab)
    axs[0].set_ylabel("v [m/s]"); axs[1].set_ylabel("w [rad/s]")
    axs[1].set_xlabel("tempo [s]")
    for ax in axs:
        ax.grid(True, alpha=.3); ax.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(a.out, "comandos.png"), dpi=150)
    plt.close()
    print("\nArquivos salvos em %s/" % a.out)


if __name__ == "__main__":
    main()
