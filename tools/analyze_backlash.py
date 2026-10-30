#!/usr/bin/env python3
"""
backlash_test 가 만든 CSV로 백래시를 계산한다.

사용법:
    python3 analyze_backlash.py backlash_0cycle.csv
    python3 analyze_backlash.py backlash_0cycle.csv --counts-per-rev 16384

출력:
    - 화면: 반복마다 백래시 값과 평균/표준편차
    - <csv이름>.png: 토크-위치 곡선 그래프

계산 방법 두 가지를 같이 보여준다.
  (A) 0점 교차법 (Volbeda 2024 방식)
      +쪽에서 토크를 0으로 내렸을 때의 위치와, -쪽에서 0으로 올렸을 때의 위치 차이
  (B) 직선 외삽법 (GB/T 35089 계열)
      토크가 큰 구간(최대의 50~100%)을 직선으로 맞추고, 토크 0에서의 두 직선 간격
  두 값이 비슷하면 측정이 잘 된 것. 크게 다르면 마찰이 크거나 토크가 너무 작은 것.
"""
import argparse
import csv
import math
import sys

import numpy as np


def read_csv(path):
    meta = {}
    rows = []
    with open(path) as f:
        lines = []
        for line in f:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition("=")
                meta[k.strip()] = v.strip()
            else:
                lines.append(line)
    reader = csv.DictReader(lines)
    for r in reader:
        rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        sys.exit("CSV에 데이터가 없음")
    cols = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    return meta, cols


def backlash_zero_crossing(cmd, pos):
    """+A -> 0 으로 내려오는 끝 점과 -A -> 0 으로 올라오는 끝 점의 위치 차이."""
    amp = np.max(np.abs(cmd))
    # 하강 구간 끝(+에서 0 도달): cmd가 양수였다가 0이 되는 첫 지점
    i_plus = None
    i_minus = None
    for i in range(1, len(cmd)):
        if i_plus is None and cmd[i - 1] > 0 and cmd[i] <= 0 and np.max(cmd[:i]) >= 0.9 * amp:
            i_plus = i
        if i_minus is None and cmd[i - 1] < 0 and cmd[i] >= 0 and np.min(cmd[:i]) <= -0.9 * amp:
            i_minus = i
    if i_plus is None or i_minus is None:
        return math.nan
    return pos[i_plus] - pos[i_minus]


def backlash_extrapolate(tor, cmd, pos):
    """토크가 증가 중인(하중을 거는) 구간에서 |cmd|가 최대의 50% 이상인 점들로 직선을 맞춘다."""
    amp = np.max(np.abs(cmd))
    dcmd = np.gradient(cmd)
    up = (cmd >= 0.5 * amp) & (dcmd > 0)      # +쪽으로 하중 증가
    dn = (cmd <= -0.5 * amp) & (dcmd < 0)     # -쪽으로 하중 증가
    if up.sum() < 5 or dn.sum() < 5:
        return math.nan, math.nan
    # pos = a * tor + b. 토크 0에서의 위치 = b
    a1, b1 = np.polyfit(tor[up], pos[up], 1)
    a2, b2 = np.polyfit(tor[dn], pos[dn], 1)
    stiffness_counts_per_unit = (a1 + a2) / 2
    return b1 - b2, stiffness_counts_per_unit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--counts-per-rev", type=int, default=None,
                    help="모터 엔코더 1회전당 카운트 (CSV 메타값보다 우선)")
    ap.add_argument("--gear-ratio", type=float, default=None)
    args = ap.parse_args()

    meta, c = read_csv(args.csv)
    cpr = args.counts_per_rev if args.counts_per_rev is not None else int(meta.get("counts_per_rev", 0))
    gr = args.gear_ratio if args.gear_ratio is not None else float(meta.get("gear_ratio", 28.35))

    def to_out_deg(counts):
        # pos가 모터 쪽 엔코더라고 가정: 모터 각도를 기어비로 나눠 출력 각도로 환산
        if not cpr:
            return math.nan
        return counts / cpr * 360.0 / gr

    reps = sorted(set(c["rep"].astype(int)))
    res_a, res_b = [], []
    print(f"파일: {args.csv}")
    print(f"기어비 {gr}, 엔코더 {cpr if cpr else '미입력(카운트로만 표시)'}")
    print(f"{'rep':>4} {'A: 0점교차[cnt]':>16} {'B: 외삽[cnt]':>14} {'A[deg 출력]':>12} {'B[deg 출력]':>12}")
    for r in reps:
        m = c["rep"].astype(int) == r
        a = backlash_zero_crossing(c["cmd_tor"][m], c["pos"][m])
        b, _ = backlash_extrapolate(c["tor"][m], c["cmd_tor"][m], c["pos"][m])
        res_a.append(a)
        res_b.append(b)
        print(f"{r:>4} {a:>16.1f} {b:>14.1f} {to_out_deg(a):>12.4f} {to_out_deg(b):>12.4f}")

    ra, rb = np.array(res_a), np.array(res_b)
    print("-" * 64)
    print(f"평균 A: {np.nanmean(ra):.1f} cnt ({to_out_deg(np.nanmean(ra)):.4f} deg)  표준편차 {np.nanstd(ra):.1f} cnt")
    print(f"평균 B: {np.nanmean(rb):.1f} cnt ({to_out_deg(np.nanmean(rb)):.4f} deg)  표준편차 {np.nanstd(rb):.1f} cnt")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 5))
        for r in reps:
            m = c["rep"].astype(int) == r
            ax.plot(c["tor"][m], c["pos"][m] - c["pos"][0], lw=1, label=f"rep {r}")
        ax.axvline(0, color="gray", lw=0.5)
        ax.set_xlabel("actual torque (1000 = rated)")
        ax.set_ylabel("motor position change [count]")
        ax.set_title("torque vs position (output locked)")
        ax.legend()
        out = args.csv.rsplit(".", 1)[0] + ".png"
        fig.tight_layout()
        fig.savefig(out, dpi=150)
        print(f"그래프 저장: {out}")
    except ImportError:
        print("matplotlib 이 없어서 그래프는 생략 (pip install matplotlib)")


if __name__ == "__main__":
    main()
