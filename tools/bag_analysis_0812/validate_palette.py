#!/usr/bin/env python3
"""validate_palette.js 의 계산 가능한 검사를 파이썬으로 포팅 (node 없음).
임계값·행렬은 원본과 동일하게 유지."""
import itertools
import math
import sys

BAND = {"light": (0.43, 0.77), "dark": (0.48, 0.67)}
CHROMA_FLOOR = 0.10
CVD_TARGET, CVD_FLOOR = 8.0, 6.0
NORMAL_FLOOR = 15.0
CONTRAST_MIN = 3.0
DEFAULT_SURFACE = {"light": "#fcfcfb", "dark": "#1a1a19"}
MACHADO = {
    "protan": [[0.152286, 1.052583, -0.204868],
               [0.114503, 0.786281, 0.099216],
               [-0.003882, -0.048116, 1.051998]],
    "deutan": [[0.367322, 0.860646, -0.227968],
               [0.280085, 0.672501, 0.047413],
               [-0.011820, 0.042940, 0.968881]],
    "tritan": [[1.255528, -0.076749, -0.178779],
               [-0.078411, 0.930809, 0.147602],
               [0.004733, 0.691367, 0.303900]],
}


def hex2srgb(h):
    h = h.strip().lstrip("#")
    return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]


def s2lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def lin(h):
    return [s2lin(c) for c in hex2srgb(h)]


def rel_lum(h):
    r, g, b = lin(h)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast(a, b):
    hi, lo = sorted([rel_lum(a), rel_lum(b)], reverse=True)
    return (hi + 0.05) / (lo + 0.05)


def oklab_from_lin(rgb):
    r, g, b = rgb
    l = (0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b) ** (1 / 3)
    m = (0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b) ** (1 / 3)
    s = (0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b) ** (1 / 3)
    return [0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s,
            1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s,
            0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s]


def oklch(h):
    L, a, b = oklab_from_lin(lin(h))
    return L, math.hypot(a, b)


def simulate(h, kind):
    r, g, b = lin(h)
    M = MACHADO[kind]
    return [min(1.0, max(0.0, M[i][0] * r + M[i][1] * g + M[i][2] * b))
            for i in range(3)]


def delta_e(h1, h2, kind=None):
    a = oklab_from_lin(simulate(h1, kind) if kind else lin(h1))
    b = oklab_from_lin(simulate(h2, kind) if kind else lin(h2))
    return 100 * math.dist(a, b)


def validate(palette, mode="light", surface=None, pairs="adjacent"):
    surface = surface or DEFAULT_SURFACE[mode]
    lo, hi = BAND[mode]
    ok = True
    print(f"\n--- mode={mode}  surface={surface}  pairs={pairs} ---")

    off = [(c, round(oklch(c)[0], 3)) for c in palette
           if not (lo <= oklch(c)[0] <= hi)]
    ok &= not off
    print(f"  [{'PASS' if not off else 'FAIL'}] 밝기 밴드 L {lo}-{hi}: "
          f"{off if off else f'{len(palette)}개 모두 통과'}")

    lowc = [(c, round(oklch(c)[1], 3)) for c in palette
            if oklch(c)[1] < CHROMA_FLOOR]
    ok &= not lowc
    print(f"  [{'PASS' if not lowc else 'FAIL'}] 채도 하한 {CHROMA_FLOOR}: "
          f"{lowc if lowc else f'{len(palette)}개 모두 통과'}")

    n = len(palette)
    pl = (list(itertools.combinations(range(n), 2)) if pairs == "all"
          else [(i, i + 1) for i in range(n - 1)])
    worst = None
    for kind in ("protan", "deutan"):
        for i, j in pl:
            d = delta_e(palette[i], palette[j], kind)
            if worst is None or d < worst[0]:
                worst = (d, kind, palette[i], palette[j])
    st = "PASS" if worst[0] >= CVD_TARGET else ("WARN" if worst[0] >= CVD_FLOOR else "FAIL")
    ok &= st != "FAIL"
    print(f"  [{st}] CVD 분리 (목표 >={CVD_TARGET}): 최악 ΔE {worst[0]:.1f} "
          f"({worst[1]}, {worst[2]} vs {worst[3]})")

    wn = min(((delta_e(palette[i], palette[j]), palette[i], palette[j])
              for i, j in pl), key=lambda x: x[0])
    okn = wn[0] >= NORMAL_FLOOR
    ok &= okn
    print(f"  [{'PASS' if okn else 'FAIL'}] 정상시야 하한 {NORMAL_FLOOR}: "
          f"최악 ΔE {wn[0]:.1f} ({wn[1]} vs {wn[2]})")

    low = [(c, round(contrast(c, surface), 2)) for c in palette
           if contrast(c, surface) < CONTRAST_MIN]
    print(f"  [{'PASS' if not low else 'WARN'}] 표면 대비 >={CONTRAST_MIN}: "
          f"{low if low else '모두 통과'}"
          + ("  -> 직접 라벨/테이블 필수" if low else ""))
    print(f"  => {'통과' if ok else '실패'}")
    return ok


if __name__ == "__main__":
    print("차트 시리즈 2색 (reference palette slot 1 blue / slot 2 orange)")
    a = validate(["#2a78d6", "#eb6834"], "light", "#ffffff", "all")
    b = validate(["#3987e5", "#d95926"], "dark", "#17202a", "all")
    sys.exit(0 if (a and b) else 1)
