"""
demo.py  —  nesting engine demo

Usage:
    python demo.py                      # greedy, built-in shapes
    python demo.py --ga                 # GA optimisation (slower, better)
    python demo.py --rotations 8        # 8 rotation angles (0..315 step 45)
    python demo.py --grain              # grain direction (0/180 only)
    python demo.py --margin 5           # 5mm bin boundary margin
    python demo.py --clearance 1.5      # 1.5mm kerf
    python demo.py shapes.json          # load your own shapes
    python demo.py --ga --gen 50        # GA with 50 generations

shapes.json format:
    [{"name":"bracket","vertices":[[0,0],[40,0],[40,30],[0,30]]}, ...]
"""

import sys
import json
import math
import argparse

# ── Test shapes ───────────────────────────────────────────────────────────────

TEST_SHAPES = [
    ("rect_large",    [(0,0),(60,0),(60,40),(0,40)]),
    ("rect_med",      [(0,0),(45,0),(45,30),(0,30)]),
    ("rect_small",    [(0,0),(25,0),(25,20),(0,20)]),
    ("square",        [(0,0),(35,0),(35,35),(0,35)]),
    ("triangle",      [(0,0),(50,0),(25,43)]),
    ("l_shape",       [(0,0),(40,0),(40,15),(15,15),(15,40),(0,40)]),
    ("t_shape",       [(0,30),(50,30),(50,45),(30,45),(30,70),(20,70),(20,45),(0,45)]),
    ("trapezoid",     [(10,0),(60,0),(50,30),(0,30)]),
    ("pentagon",      [(25,0),(50,18),(40,48),(10,48),(0,18)]),
    ("parallelogram", [(10,0),(60,0),(50,35),(0,35)]),
    ("rect_thin",     [(0,0),(80,0),(80,12),(0,12)]),
    ("small_sq",      [(0,0),(18,0),(18,18),(0,18)]),
]

COLOURS = [
    "#AFA9EC","#9FE1CB","#F5C4B3","#FAC775","#B5D4F4",
    "#C0DD97","#F4C0D1","#D3D1C7","#F09595","#5DCAA5",
    "#EF9F27","#85B7EB","#ED93B1","#97C459","#F0997B",
]


# ── Visualiser ────────────────────────────────────────────────────────────────

def visualise(result, output_file="nesting_result.svg", show_stats=True):
    """
    Render nesting result as SVG.
    Hole-filled shapes shown with dashed outline.
    GA convergence bar shown if ga_stats present.
    """
    placements          = result['placements']
    bin_w, bin_h        = result['bin']
    util                = result['utilisation']
    ga_stats            = result.get('ga_stats')

    scale = min(760/bin_w, 520/bin_h)
    svg_w = bin_w*scale + 80
    svg_h = bin_h*scale + (180 if ga_stats else 120)

    def tx(x): return 40 + x*scale
    def ty(y): return 40 + (bin_h-y)*scale

    def pts(verts):
        return " ".join(f"{tx(v[0]):.1f},{ty(v[1]):.1f}" for v in verts)

    L = []
    L.append('<?xml version="1.0" encoding="UTF-8"?>')
    L.append(f'<svg xmlns="http://www.w3.org/2000/svg" '
             f'width="{svg_w:.0f}" height="{svg_h:.0f}" '
             f'style="background:#f8f7f4;font-family:sans-serif">')

    # ── Title ─────────────────────────────────────────────────────────────────
    mode = "GA" if ga_stats else "Greedy"
    L.append(f'<text x="{svg_w/2:.0f}" y="24" text-anchor="middle" '
             f'font-size="14" font-weight="500" fill="#3d3d3a">'
             f'{mode} nesting - {util}% utilisation '
             f'({result["placed"]}/{result["total"]} parts)</text>')

    # ── Bin outline ───────────────────────────────────────────────────────────
    bx0,by0 = tx(0),ty(bin_h)
    bw,bh   = bin_w*scale, bin_h*scale
    L.append(f'<rect x="{bx0:.1f}" y="{by0:.1f}" width="{bw:.1f}" '
             f'height="{bh:.1f}" fill="#fff" stroke="#999" '
             f'stroke-width="1.5" stroke-dasharray="6 3"/>')

    # ── Placed parts ──────────────────────────────────────────────────────────
    for i, p in enumerate(placements):
        col   = COLOURS[i % len(COLOURS)]
        verts = p['placed']
        label = p.get('name', f'#{i+1}')
        rot   = p['rotation']
        filled = p.get('hole_filled', False)

        stroke_dash = 'stroke-dasharray="4 2"' if filled else ''
        stroke_w    = '0.6' if filled else '0.9'

        L.append(f'<polygon points="{pts(verts)}" '
                 f'fill="{col}" fill-opacity="0.72" '
                 f'stroke="#444" stroke-width="{stroke_w}" {stroke_dash}/>')

        cx = sum(v[0] for v in verts)/len(verts)
        cy = sum(v[1] for v in verts)/len(verts)
        rot_label = f" {int(rot)}°" if rot != 0 else ""
        L.append(f'<text x="{tx(cx):.1f}" y="{ty(cy):.1f}" '
                 f'text-anchor="middle" dominant-baseline="central" '
                 f'font-size="9" fill="#222">{label}{rot_label}</text>')

    # ── Utilisation bar ───────────────────────────────────────────────────────
    bar_y  = by0+bh+16
    fill_w = bw*util/100
    L.append(f'<rect x="{bx0:.1f}" y="{bar_y:.1f}" width="{bw:.1f}" '
             f'height="8" fill="#e0ddd5" rx="4"/>')
    L.append(f'<rect x="{bx0:.1f}" y="{bar_y:.1f}" width="{fill_w:.1f}" '
             f'height="8" fill="#1D9E75" rx="4"/>')
    L.append(f'<text x="{bx0+bw+8:.1f}" y="{bar_y+7:.1f}" '
             f'font-size="11" fill="#555">{util}%</text>')

    # ── GA convergence strip (if available) ───────────────────────────────────
    if ga_stats:
        history = ga_stats['history']
        gens    = len(history)
        sx      = bx0; sy = bar_y+24; sw=bw; sh=50
        L.append(f'<text x="{sx:.1f}" y="{sy-4:.1f}" '
                 f'font-size="10" fill="#888">GA convergence</text>')
        L.append(f'<rect x="{sx:.1f}" y="{sy:.1f}" width="{sw:.1f}" '
                 f'height="{sh:.1f}" fill="#f0ede6" rx="3"/>')

        if gens > 1:
            lo = min(history); hi = max(history)
            span = max(hi-lo, 1.0)
            def gx(g): return sx + g/(gens-1)*sw
            def gy(v): return sy+sh - (v-lo)/span*sh
            path = " ".join(
                f"{'M' if i==0 else 'L'}{gx(i):.1f},{gy(v):.1f}"
                for i,v in enumerate(history))
            L.append(f'<path d="{path}" fill="none" '
                     f'stroke="#185FA5" stroke-width="1.5"/>')

        elapsed = ga_stats['elapsed_s']
        ngen    = ga_stats['generations']
        L.append(f'<text x="{sx+sw:.1f}" y="{sy+sh+12:.1f}" '
                 f'text-anchor="end" font-size="10" fill="#666">'
                 f'{ngen} gens · {elapsed}s</text>')

    # ── Legend: dashed = hole-filled ─────────────────────────────────────────
    any_hole = any(p.get('hole_filled') for p in placements)
    if any_hole:
        lx = bx0; ly = svg_h-14
        L.append(f'<line x1="{lx:.0f}" y1="{ly:.0f}" '
                 f'x2="{lx+20:.0f}" y2="{ly:.0f}" '
                 f'stroke="#444" stroke-width="1" stroke-dasharray="4 2"/>')
        L.append(f'<text x="{lx+24:.0f}" y="{ly+4:.0f}" '
                 f'font-size="10" fill="#666">hole-filled</text>')

    L.append('</svg>')
    svg = "\n".join(L)
    with open(output_file, 'w', encoding='utf-8') as f: f.write(svg)
    print(f"[viz] saved -> {output_file}")
    return svg


# ── CLI ───────────────────────────────────────────────────────────────────────

def build_rotation_tuple(n):
    """Build n evenly-spaced rotations from 0 to 360."""
    return tuple(round(360*i/n, 1) for i in range(n))

def load_shapes(path):
    with open(path) as f: data=json.load(f)
    named=[(item.get('name',f'shape{i}'),[tuple(v) for v in item['vertices']])
           for i,item in enumerate(data)]
    return named

def print_placements(result, names):
    for p in result['placements']:
        idx  = p.get('shape_idx', '?')
        name = names[idx] if isinstance(idx,int) and idx<len(names) else p.get('name','?')
        pos  = p['position']
        hf   = ' [hole]' if p.get('hole_filled') else ''
        print(f"  {name:20s}  pos=({pos[0]:6.1f},{pos[1]:6.1f})  "
              f"rot={p['rotation']:.0f}°{hf}")

def main():
    ap = argparse.ArgumentParser(description="2D nesting engine demo")
    ap.add_argument('file',         nargs='?',   help='shapes JSON file')
    ap.add_argument('--ga',         action='store_true', help='use GA optimisation')
    ap.add_argument('--rotations',  type=int, default=8,
                    help='number of rotation angles (4=0/90/180/270, 8=every 45°)')
    ap.add_argument('--clearance',  type=float, default=2.0,  help='kerf mm')
    ap.add_argument('--margin',     type=float, default=0.0,  help='bin margin mm')
    ap.add_argument('--grain',      action='store_true', help='grain direction (0/180 only)')
    ap.add_argument('--bin',        type=str,   default='300x220', help='bin WxH mm')
    ap.add_argument('--pop',        type=int,   default=20,   help='GA population size')
    ap.add_argument('--gen',        type=int,   default=30,   help='GA generations')
    ap.add_argument('--no-holes',   action='store_true', help='disable hole filling')
    ap.add_argument('--out',        type=str,   default='nesting_result.svg')
    args = ap.parse_args()

    # Bin
    try:
        bw, bh = map(float, args.bin.lower().split('x'))
    except Exception:
        print(f"[error] --bin format should be WxH e.g. 300x220"); return

    # Shapes
    if args.file:
        named = load_shapes(args.file)
        print(f"[demo] loaded {len(named)} shapes from {args.file}")
    else:
        named = TEST_SHAPES
        print(f"[demo] using {len(named)} built-in test shapes")

    shapes = [v for _,v in named]
    names  = [n for n,_ in named]

    # Rotations
    rot_set = build_rotation_tuple(args.rotations)
    print(f"[demo] bin: {bw:.0f}×{bh:.0f}mm  clearance: {args.clearance}mm  "
          f"margin: {args.margin}mm")
    print(f"[demo] rotations: {list(rot_set)}")
    if args.grain: print("[demo] grain direction ON (0°/180° only)")

    common = dict(
        bin_w=bw, bin_h=bh,
        clearance=args.clearance,
        rotations=rot_set,
        grain_direction=args.grain,
        bin_margin=args.margin,
        enable_hole_fill=not args.no_holes,
    )

    from placer import nest, ga_nest

    if args.ga:
        print(f"[demo] running GA (pop={args.pop}, gen={args.gen})...")
        result = ga_nest(shapes, pop_size=args.pop, generations=args.gen,
                         verbose=True, **common)
    else:
        print("[demo] running greedy nesting...")
        result = nest(shapes, **common)

    # Attach names
    for p in result['placements']:
        idx = p.get('shape_idx')
        p['name'] = names[idx] if isinstance(idx,int) and idx<len(names) else '?'

    print(f"\n[result] placed {result['placed']}/{result['total']} shapes")
    print(f"[result] utilisation: {result['utilisation']}%")
    if 'ga_stats' in result:
        gs = result['ga_stats']
        print(f"[GA]     {gs['generations']} generations · {gs['elapsed_s']}s")

    print(f"\n[placements]")
    print_placements(result, names)

    visualise(result, args.out)
    print(f"\n[done] open {args.out} in a browser")

if __name__ == "__main__":
    main()