#!/usr/bin/env python3
"""
analyze_backlash.py --cycle 로 쌓은 backlash_log.csv 를 사이클-백래시 그래프로 그린다.

사용법:
    python3 plot_trend.py                      # backlash_log.csv 를 읽음
    python3 plot_trend.py my_log.csv

출력:
    - 화면: 사이클별 백래시 표
    - backlash_trend.png: 사이클(가로) - 백래시 출력 각도(세로), 종료 기준 0.2도 점선
"""
import csv
import sys

LIMIT_DEG = 0.2


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "backlash_log.csv"
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path} 에 기록이 없음")
    rows.sort(key=lambda r: int(r["cycle"]))

    print(f"{'사이클':>6} {'백래시[deg]':>11} {'백래시[cnt]':>11} {'최고온도':>8}  날짜")
    for r in rows:
        print(f"{r['cycle']:>6} {float(r['backlash_deg']):>11.4f} {r['backlash_cnt']:>11} "
              f"{r['temp_max_c']:>8}  {r['datetime']}")
    last = rows[-1]
    print(f"최근 값: {float(last['backlash_deg']):.4f} 도 (종료 기준 {LIMIT_DEG} 도)")
    if float(last["backlash_deg"]) >= LIMIT_DEG:
        print(">>> 종료 기준 도달. 실험을 멈출 것.")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x = [int(r["cycle"]) for r in rows]
    y = [float(r["backlash_deg"]) for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(x, y, marker="o")
    ax.axhline(LIMIT_DEG, ls="--", color="gray")
    ax.text(x[0], LIMIT_DEG, " stop at 0.2 deg", va="bottom", color="gray")
    ax.set_xlabel("cycle")
    ax.set_ylabel("backlash at output [deg]")
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("backlash_trend.png", dpi=130)
    print("그래프 저장: backlash_trend.png")


if __name__ == "__main__":
    main()
