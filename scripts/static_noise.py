#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mede ruído e confiabilidade dos sensores com o robô PARADO.

Procedimento: coloque um obstáculo fixo (ex.: caixa/parede) à frente do
robô, grave ~30 s com tools/record_run.sh e rode:

    rosrun pioneer_reactive_nav static_noise.py data/parado.bag --out results/

Saídas: impressão no terminal e results/ruido_estatico.png
"""
import argparse
import math
import os
import warnings

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

warnings.filterwarnings("ignore")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--sonar", default="/RosAria/sonar")
    ap.add_argument("--front-deg", type=float, default=30.0)
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fr = math.radians(a.front_deg)

    R, ts_scan, ang = [], [], None
    son_t, son_front, son_n = [], [], []

    with rosbag.Bag(a.bag) as bag:
        for topic, msg, t in bag.read_messages(topics=[a.scan, a.sonar]):
            if topic == a.scan:
                r = np.asarray(msg.ranges, dtype=float)
                if ang is None:
                    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
                    ang = np.arctan2(np.sin(ang), np.cos(ang))
                if r.size != ang.size:
                    continue
                bad = ~np.isfinite(r) | (r < msg.range_min) | (r > msg.range_max)
                r[bad] = np.nan
                R.append(r)
                ts_scan.append(t.to_sec())
            else:
                son_t.append(t.to_sec())
                son_n.append(len(msg.points))
                ds = [math.hypot(p.x, p.y) for p in msg.points
                      if abs(math.atan2(p.y, p.x)) <= fr]
                son_front.append(min(ds) if ds else np.nan)

    fig, axs = plt.subplots(1, 2, figsize=(11, 4))

    if R:
        R = np.array(R)
        front = np.abs(ang) <= fr
        inval = np.mean(np.isnan(R), axis=0)
        std = np.nanstd(R, axis=0)
        dur = ts_scan[-1] - ts_scan[0]
        print("== LIDAR ==")
        print("  mensagens: %d  (taxa ~ %.1f Hz)" % (len(R), (len(R) - 1) / dur if dur > 0 else 0))
        print("  feixes frontais (±%.0f°): %d" % (a.front_deg, front.sum()))
        print("  leituras inválidas (frente): %.1f %%" % (100 * np.mean(inval[front])))
        print("  desvio-padrão mediano (frente): %.4f m" % np.nanmedian(std[front]))
        print("  desvio-padrão máximo  (frente): %.4f m" % np.nanmax(std[front]))
        order = np.argsort(ang)
        axs[0].plot(np.degrees(ang[order]), std[order], lw=.8)
        axs[0].set_xlabel("ângulo [°]"); axs[0].set_ylabel("desvio-padrão [m]")
        axs[0].set_title("LIDAR: ruído por feixe (robô parado)")
        axs[0].grid(True, alpha=.3)
    else:
        print("== LIDAR == sem dados em %s" % a.scan)

    if son_t:
        sf = np.array(son_front)
        dur = son_t[-1] - son_t[0]
        print("== SONAR ==")
        print("  mensagens: %d  (taxa ~ %.1f Hz)" % (len(son_t), (len(son_t) - 1) / dur if dur > 0 else 0))
        print("  nº médio de ecos por mensagem: %.1f" % np.mean(son_n))
        print("  mensagens sem eco frontal: %.1f %%" % (100 * np.mean(np.isnan(sf))))
        print("  média da distância frontal: %.3f m" % np.nanmean(sf))
        print("  desvio-padrão da distância frontal: %.4f m" % np.nanstd(sf))
        axs[1].plot(np.array(son_t) - son_t[0], sf, ".-", ms=3, lw=.6)
        axs[1].set_xlabel("tempo [s]"); axs[1].set_ylabel("distância frontal [m]")
        axs[1].set_title("Sonar: eco frontal (robô parado)")
        axs[1].grid(True, alpha=.3)
    else:
        print("== SONAR == sem dados em %s" % a.sonar)

    plt.tight_layout()
    out = os.path.join(a.out, "ruido_estatico.png")
    plt.savefig(out, dpi=150)
    print("\nFigura salva em %s" % out)


if __name__ == "__main__":
    main()
