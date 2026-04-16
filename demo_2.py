from placer import nest, ga_nest
from demo import visualise   # ← reuse your SVG renderer

# ── 3 rectangle types ─────────────────────────────────────────────

SHAPES = [
    # Large
    [(0,0),(80,0),(80,40),(0,40)],
    [(0,0),(80,0),(80,40),(0,40)],

    # Medium
    [(0,0),(50,0),(50,30),(0,30)],
    [(0,0),(50,0),(50,30),(0,30)],
    [(0,0),(50,0),(50,30),(0,30)],

    # Small
    [(0,0),(30,0),(30,20),(0,20)],
    [(0,0),(30,0),(30,20),(0,20)],
    [(0,0),(30,0),(30,20),(0,20)],
    [(0,0),(30,0),(30,20),(0,20)],
]

NAMES = [
    "L1","L2",
    "M1","M2","M3",
    "S1","S2","S3","S4"
]

# ── Bin ───────────────────────────────────────────────────────────

BIN_W = 200
BIN_H = 120

COMMON = dict(
    bin_w=BIN_W,
    bin_h=BIN_H,
    clearance=2.0,
    rotations=(0, 45, 90, 135, 180, 225, 270, 315),
    bin_margin=0.0,
)

# ── Attach names helper ───────────────────────────────────────────

def attach_names(result):
    for p in result['placements']:
        idx = p['shape_idx']
        p['name'] = NAMES[idx]

# ── Run ───────────────────────────────────────────────────────────

if __name__ == "__main__":

    print("=== GREEDY ===")
    res = nest(SHAPES, **COMMON)
    attach_names(res)

    print(f"[result] density: {res['density']}%")
    print(f"[result] placement: {res['placement_rate']}%")
    print(f"[debug] internal fitness: {res['utilisation']}%")
    print(f"Placed: {res['placed']}/{res['total']}")

    visualise(res, "rect_greedy.svg")   # ← SVG output

    print("\n=== GA ===")
    res_ga = ga_nest(
        SHAPES,
        pop_size=20,
        generations=20,
        **COMMON
    )
    attach_names(res_ga)

    print(f"Utilisation: {res_ga['utilisation']}%")
    print(f"Placed: {res_ga['placed']}/{res_ga['total']}")

    visualise(res_ga, "rect_ga.svg")   # ← SVG output

    print("\n[done] open rect_greedy.svg and rect_ga.svg")