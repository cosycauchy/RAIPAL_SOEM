#!/usr/bin/env python3
"""
endurance_test 가 만든 rev_trace.csv (처음 사이클의 모터 움직임 기록)를 그래프로 그린다.

사용법:
    python3 plot_trace.py rev_trace.csv        # → rev_trace.png

위: 위치 (출력축 각도, 시작 위치 기준)와 목표   가운데: 속도 [rpm]   아래: 토크 명령과 드라이버가 알려주는 실제 토크
세로 점선 = 구간이 바뀐 곳. 숫자는 (바퀴*10 + 0 가속 / 1 일정속도 / 2 감속 / 3 위치로 마무리)
"""
import csv
import sys

CPR, GEAR = 65536, 28.35

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "rev_trace.csv"
    rows = [r for r in csv.DictReader(open(path))]
    if not rows:
        sys.exit("기록이 비어 있음")
    t = [float(r["t_s"]) for r in rows]
    p0 = int(rows[0]["pos"])
    deg = lambda c: (c - p0) / (CPR * GEAR) * 360.0
    pos = [deg(int(r["pos"])) for r in rows]
    tgt = [deg(int(r["target_pos"])) for r in rows]
    rpm = [float(r["rpm"]) for r in rows]
    tor = [int(r["cmd_tor"]) for r in rows]
    act = [int(r["tor"]) for r in rows]          # 드라이버가 알려주는 실제 토크 (1000 = 정격)
    tag = [int(r["tag"]) for r in rows]
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    ax[0].plot(t, pos, label="actual"); ax[0].plot(t, tgt, lw=0.8, ls="--", label="target (only in position mode)")
    ax[0].set_ylabel("output angle [deg]"); ax[0].legend(loc="best")
    ax[1].plot(t, rpm); ax[1].set_ylabel("speed [rpm, output]"); ax[1].axhline(0, color="gray", lw=0.5)
    ax[2].plot(t, tor, label="command"); ax[2].plot(t, act, lw=0.8, label="driver feedback")
    ax[2].set_ylabel("torque (1000 = rated)"); ax[2].set_xlabel("time [s]"); ax[2].legend(loc="best")
    for i in range(1, len(t)):
        if tag[i] != tag[i - 1]:
            for a in ax:
                a.axvline(t[i], color="gray", ls=":", lw=0.6)
            ax[0].text(t[i], ax[0].get_ylim()[1], str(tag[i]), fontsize=7, va="top")
    ax[0].grid(alpha=0.3); ax[1].grid(alpha=0.3); ax[2].grid(alpha=0.3)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=120)
    print("그래프 저장:", out)

if __name__ == "__main__":
    main()
