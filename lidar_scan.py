#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
import serial
import yaml
import matplotlib.pyplot as plt


CPR = 1320.0


def connect(p):
    sp, auto = p["serial"], p["autostop"]
    try:
        ser = serial.Serial(sp["port"], int(sp["baud"]), timeout=0.1)
    except Exception as e:
        sys.exit(f"[에러] 시리얼 연결 실패: {e}")
    print(f"[연결] {sp['port']} @ {sp['baud']} — 부팅 대기(2s)...")
    time.sleep(2.0)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(b"z\n")
    time.sleep(0.2)
    ser.write(b"s\n")
    print("시작 명령 전송 — 스캔 중")

    raw_list = []
    last_print, last_enc = 0.0, 0.0
    last_line = "(대기)"

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                if line.startswith("#"):
                    sys.stdout.write("\r" + " " * 88 + "\r")
                    print(f"[EVENT] {line.lstrip('# ').strip()}")
                else:
                    parts = line.split(",")
                    if len(parts) == 4:
                        try:
                            rec = tuple(float(x) for x in parts)
                            raw_list.append(rec)
                            last_enc = rec[1]
                            last_line = (f"tilt {rec[0]:5.1f}°  enc {rec[1]:7.0f}  "
                                         f"d {rec[2]:4.0f}cm  s {rec[3]:4.0f}")
                        except ValueError:
                            pass

            cur = abs(last_enc) / CPR
            now = time.time()
            if now - last_print >= 0.3:
                tgt = f"/{auto['target']:.0f}" if auto["enabled"] else ""
                sys.stdout.write(f"\r[scan] 점 {len(raw_list):6d}  "
                                 f"{cur:6.1f}{tgt}rev  | {last_line}   ")
                sys.stdout.flush()
                last_print = now

            if auto["enabled"] and cur >= auto["target"]:
                print(f"\n목표 {auto['target']:.0f}rev 도달")
                break
    except KeyboardInterrupt:
        print("\n스캔 종료")

    try:
        ser.write(b"h\n")
        time.sleep(0.1)
        ser.close()
    except Exception:
        pass

    raw = np.asarray(raw_list, dtype=float) if raw_list else np.empty((0, 4))
    print(f"[완료] 총 {raw.shape[0]} 점 수집")
    return raw


if __name__ == "__main__":
    params_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "params.yaml")
    try:
        with open(params_path) as f:
            p = yaml.safe_load(f) or {}
    except FileNotFoundError:
        sys.exit(f"[에러] params.yaml 이 같은 폴더에 없음")

    g, flt = p["geometry"], p["filter"]

    raw = connect(p)
    dist, strg = raw[:, 2], raw[:, 3]
    mask = (dist >= flt["dist_min"]) & (dist <= flt["dist_max"]) \
           & (strg >= flt["strength_min"])
    sub = raw[mask]
    phi = np.radians(sub[:, 0])
    theta = np.radians(sub[:, 1] * 360.0 / CPR)  # enc 카운트 -> 턴테이블 각도로 변환
    radial = g["D"] - sub[:, 2] * np.cos(phi)  # radial: x/y 계산에 필요(필터 아님)
    xyz = np.column_stack([radial * np.cos(theta),
                           -radial * np.sin(theta),
                           g["H"] + sub[:, 2] * np.sin(phi)]) #실습 예제에서 phi를 이용해 Z축 추가

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c="#ff9f43", s=3, depthshade=False)

    ax.set_xlabel("X [cm]")
    ax.set_ylabel("Y [cm]")
    ax.set_zlabel("Z [cm]")
    ax.set_title(f"LiDAR Scan Result — {xyz.shape[0]} pts")
    try:
        ax.set_box_aspect((np.ptp(xyz[:, 0]) + 1e-6,
                           np.ptp(xyz[:, 1]) + 1e-6,
                           np.ptp(xyz[:, 2]) + 1e-6))
    except Exception:
        pass

    fig.patch.set_facecolor("#15151a")
    ax.set_facecolor("#15151a")

    ax.tick_params(colors="#aaaaaa")
    for lab in (ax.xaxis.label, ax.yaxis.label, ax.zaxis.label, ax.title):
        lab.set_color("#dddddd")
    ax.legend(loc="upper right", facecolor="#1d1d24", labelcolor="#dddddd")

    print("Result")
    plt.show()