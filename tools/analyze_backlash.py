#!/usr/bin/env python3
"""
backlash_test 가 만든 CSV로 백래시를 계산한다. (10 kg 원판을 단 상태, 진동 방식)

사용법:
    python3 analyze_backlash.py backlash_0cycle.csv

계산:
    토크를 + 로 걸면 모터는 틈의 한쪽 끝에, - 로 걸면 반대쪽 끝에 붙는다.
    두 끝 위치의 차이가 기어 사이 틈, 즉 백래시다.
    - 처음 4구간은 버린다 (막 토크를 걸기 시작해 흔들리는 구간)
    - 구간 뒤쪽 1/3에서 자리 잡은 위치를 구하고, 이웃한 +/- 구간의 차이를 백래시로 본다
    - 위치마다 중앙값을 대표값으로 쓴다 (튀는 값에 덜 흔들림)
    - 선배 코드 방식(구간 안 최대-최소의 최댓값)도 비교용으로 같이 보여준다

출력:
    - 화면: 측정 위치별 백래시 (모터 카운트, 출력 각도)
    - <csv이름>.png: 위치별 모터 위치 파형
"""
import argparse
import csv
import statistics
import sys
from collections import defaultdict

SKIP_HALVES = 4


def read_csv(path):
    meta, lines = {}, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition("=")
                meta[k.strip()] = v.strip()
            else:
                lines.append(line)
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(lines)]
    if not rows:
        sys.exit("CSV에 데이터가 없음")
    return meta, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--counts-per-rev", type=int, default=None)
    ap.add_argument("--gear-ratio", type=float, default=None)
    ap.add_argument("--cycle", type=int, default=None,
                    help="이 측정이 몇 사이클째인지. 주면 결과를 --log 파일에 한 줄 추가한다")
    ap.add_argument("--log", default="backlash_log.csv",
                    help="결과를 누적할 파일 (기본 backlash_log.csv)")
    ap.add_argument("--note", default="", help="메모 (예: '선풍기 켬')")
    args = ap.parse_args()

    meta, rows = read_csv(args.csv)
    cpr = args.counts_per_rev or int(meta.get("counts_per_rev", 65536))
    gr = args.gear_ratio or float(meta.get("gear_ratio", 28.35))

    def deg(counts):  # 모터 카운트 -> 출력 각도
        return counts / cpr * 360.0 / gr

    # (위치, 구간) 별로 pos 모으기
    segs = defaultdict(list)
    for r in rows:
        segs[(int(r["pos_idx"]), int(r["half"]))].append(r["pos"])

    # 구간마다 "자리 잡은 위치" = 구간 뒤쪽 1/3의 중앙값 (토크 방향 쪽 이빨에 붙은 위치)
    # 이웃한 + 구간과 - 구간의 자리 잡은 위치 차이 = 백래시
    # (선배 방식: 구간 안에서 최대-최소. 비교용으로 같이 계산)
    levels = defaultdict(dict)
    senior = defaultdict(list)
    for (pi, half), ps in sorted(segs.items()):
        if half < SKIP_HALVES or len(ps) < 6:
            continue
        levels[pi][half] = statistics.median(ps[-max(2, len(ps) // 3):])
        senior[pi].append(max(ps) - min(ps))
    per_pos = defaultdict(list)
    for pi, lv in levels.items():
        for h in sorted(lv):
            if h - 1 in lv:
                per_pos[pi].append(abs(lv[h] - lv[h - 1]))

    print(f"파일: {args.csv}   (기어비 {gr}, 엔코더 {cpr} 카운트/회전)")
    print(f"{'위치':>4} {'횟수':>6} {'백래시[cnt]':>12} {'출력[deg]':>10} {'선배식 최댓값[cnt]':>18}")
    medians = []
    for pi in sorted(per_pos):
        d = per_pos[pi]
        med = statistics.median(d)
        medians.append(med)
        print(f"{pi:>4} {len(d):>6} {med:>12.0f} {deg(med):>10.4f} {max(senior[pi]):>18.0f}")
    if not medians:
        sys.exit("계산할 구간이 없음 (measure_time_s 가 너무 짧은지 확인)")
    avg = sum(medians) / len(medians)
    print("-" * 52)
    print(f"백래시 (위치별 중앙값의 평균): {avg:.0f} 카운트 = 출력 {deg(avg):.4f} 도")
    print(f"종료 기준 0.2도 = {0.2 / 360 * gr * cpr:.0f} 카운트")

    if args.cycle is not None:
        import datetime
        import os
        new = not os.path.exists(args.log)
        with open(args.log, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["datetime", "cycle", "backlash_cnt", "backlash_deg", "senior_max_cnt",
                            "temp_max_c", "torque_mag", "csv_file", "note"])
            w.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), args.cycle,
                        round(avg), round(deg(avg), 4),
                        round(max(max(v) for v in senior.values())),
                        int(max(r["temp"] for r in rows)), meta.get("torque_mag", ""),
                        args.csv, args.note])
        print(f"기록 추가: {args.log} (사이클 {args.cycle})")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(per_pos)
        fig, axes = plt.subplots(n, 1, figsize=(8, 2.2 * n), squeeze=False)
        for ax, pi in zip(axes[:, 0], sorted(per_pos)):
            sel = [r for r in rows if int(r["pos_idx"]) == pi]
            t0 = sel[0]["t_s"]
            ax.plot([r["t_s"] - t0 for r in sel], [r["pos"] - sel[0]["pos"] for r in sel], lw=0.8)
            ax.set_ylabel(f"pos {pi} [cnt]")
        axes[-1, 0].set_xlabel("time [s]")
        fig.suptitle("motor position during vibration (disc mounted)")
        fig.tight_layout()
        out = args.csv.rsplit(".", 1)[0] + ".png"
        fig.savefig(out, dpi=130)
        print(f"그래프 저장: {out}")
    except ImportError:
        print("matplotlib 이 없어서 그래프는 생략 (pip install matplotlib)")


if __name__ == "__main__":
    main()
