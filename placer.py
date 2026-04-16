"""
placer.py  —  hybrid nesting engine

Architecture:
  - C++ core (nfp_engine) for NFP/IFP/offset geometry
  - Pure Python fallback when C++ is not compiled
  - nest()    — greedy bottom-left placer, accepts GA-driven sequence
  - ga_nest() — GA wrapper that evolves placement order

═══════════════════════════════════════════════════════════════════════════════
WHAT GETS STORED AND DISPLAYED — critical distinction
═══════════════════════════════════════════════════════════════════════════════

  norm_raw_aligned : raw polygon, aligned to same origin as norm_off
  norm_off         : offset (clearance-expanded) polygon, bbox-min = (0,0)

  placed_vis  ← norm_off + pos   : OFFSET shape in bin coords
                                    Used as A in NFP for subsequent shapes.
                                    NOT drawn in SVG.

  placed_raw  ← norm_raw_aligned + pos : RAW shape in bin coords
                                          Used for area calculation.
                                          NOT drawn in SVG.

  result['placed']  ← norm_raw_aligned + pos  : RAW shape — DRAWN in SVG
                       This is what the operator sees and cuts.
                       The clearance gap between adjacent raw shapes is the
                       visible kerf/spacing on the sheet.

  The internal offset shapes (placed_vis) enforce the gap mechanically but
  are never shown — only the clean raw outlines appear in the output.

═══════════════════════════════════════════════════════════════════════════════
COORDINATE FRAME CONTRACT
═══════════════════════════════════════════════════════════════════════════════

  1. rotate_poly(shape, deg)          → rotated  (input coords)
  2. _to_origin(rotated)              → norm_raw, ox, oy  (bbox-min = 0,0)
  3. offset_poly(norm_raw, clearance) → off_b    (built from norm_raw frame)
  4. _to_origin(off_b)                → norm_off, fx, fy  (bbox-min = 0,0)
  5. norm_raw_aligned = norm_raw shifted by (-fx, -fy)
     → norm_raw and norm_off now share the same spatial origin
  6. compute_bin_ifp(bin_rect, norm_off) → ifp  (bin coords for bbox-min)
  7. compute_nfp(placed_vis[i], norm_off) → nfp_i  (bin coords)
  8. pos = accepted candidate (bin coords, = bbox-min of norm_off)
  9. placed_vis  ← _translate(norm_off,         pos[0], pos[1])
     placed_raw  ← _translate(norm_raw_aligned, pos[0], pos[1])
     display     ← placed_raw  (raw outline only)

═══════════════════════════════════════════════════════════════════════════════
"""

import math
import random
import time
from typing import List, Tuple, Optional, Dict, Any

Poly = List[Tuple[float, float]]

EDGE_SAMPLES   = 16
HOLE_TOP_K     = 4
HOLE_SHAPE_K   = 4
HOLE_MIN_AREA  = 200.0
HOLE_GRID_STEP = 15.0

# ── Engine import ─────────────────────────────────────────────────────────────

try:
    import nfp_engine as _nfp
    def _compute_nfp_raw(A, B):    return list(map(tuple, _nfp.compute_nfp(A, B)))
    def _compute_ifp_raw(bin_, B): return list(map(tuple, _nfp.compute_bin_ifp(bin_, B)))
    def offset_poly(poly, d):      return list(map(tuple, _nfp.offset(poly, d)))
    print("[nfp] using compiled C++ engine")
except ImportError:
    print("[nfp] C++ engine not found — using pure Python fallback")
    print("[nfp] Build: pip install pybind11 && mkdir build && cd build && cmake .. && cmake --build . --config Release")

    def _cross(a, b): return a[0]*b[1] - a[1]*b[0]
    def _sub(a, b):   return (a[0]-b[0], a[1]-b[1])
    def _add(a, b):   return (a[0]+b[0], a[1]+b[1])
    def _len(a):      return math.sqrt(a[0]**2 + a[1]**2)

    def _sarea(p):
        n = len(p)
        return sum(_cross(p[i], p[(i+1)%n]) for i in range(n)) * 0.5

    def _ccw(p):
        return list(p) if _sarea(p) > 0 else list(reversed(p))

    def _hull(pts):
        pts = sorted(set(pts))
        if len(pts) < 3: return pts
        def cr(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
        lo = []
        for p in pts:
            while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0: lo.pop()
            lo.append(p)
        hi = []
        for p in reversed(pts):
            while len(hi) >= 2 and cr(hi[-2], hi[-1], p) <= 0: hi.pop()
            hi.append(p)
        return lo[:-1] + hi[:-1]

    def _mink(A, Bn):
        def edges(poly):
            n = len(poly); ev = []
            for i in range(n):
                v = _sub(poly[(i+1)%n], poly[i])
                if _len(v) < 1e-9: continue
                ev.append((v, math.atan2(v[1], v[0]) % (2*math.pi)))
            return ev
        eA, eB = edges(A), edges(Bn)
        if not eA or not eB: return []
        def bot(p):
            return min(range(len(p)), key=lambda i: (p[i][1], p[i][0]))
        bA, bB = bot(A), bot(Bn)
        eA = eA[bA:] + eA[:bA]
        eB = eB[bB:] + eB[:bB]
        cur = _add(A[bA], Bn[bB]); nfp = [cur]
        ia, ib = 0, 0
        while ia < len(eA) or ib < len(eB):
            aA = eA[ia][1] if ia < len(eA) else 1e18
            aB = eB[ib][1] if ib < len(eB) else 1e18
            step = (0, 0)
            if aA <= aB + 1e-9 and ia < len(eA):
                step = _add(step, eA[ia][0]); ia += 1
                if ib < len(eB) and abs(aA-aB) < 1e-9:
                    step = _add(step, eB[ib][0]); ib += 1
            else:
                step = _add(step, eB[ib][0]); ib += 1
            cur = _add(cur, step)
            if _len(_sub(nfp[-1], cur)) > 1e-9: nfp.append(cur)
        if len(nfp) >= 2 and _len(_sub(nfp[0], nfp[-1])) < 1e-9:
            nfp.pop()
        return _ccw(nfp)

    def _compute_nfp_raw(A, B):
        A = _ccw(list(A)); B = _ccw(list(B))
        bx, by = B[0]
        Bn = _ccw([(2*bx-v[0], 2*by-v[1]) for v in B])
        r = _mink(A, Bn)
        return _ccw(r) if r else []

    def _compute_ifp_raw(bin_, B):
        B = _ccw(list(B))
        bx0 = min(v[0] for v in bin_); bx1 = max(v[0] for v in bin_)
        by0 = min(v[1] for v in bin_); by1 = max(v[1] for v in bin_)
        ref = B[0]
        mnx = min(v[0]-ref[0] for v in B); mxx = max(v[0]-ref[0] for v in B)
        mny = min(v[1]-ref[1] for v in B); mxy = max(v[1]-ref[1] for v in B)
        x0, y0 = bx0-mnx, by0-mny
        x1, y1 = bx1-mxx, by1-mxy
        if x0 >= x1 or y0 >= y1: return []
        return _ccw([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])

    def offset_poly(poly, d):
        if d < 1e-9: return list(poly)
        all_pts = list(poly); n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i+1)%n]
            e = _sub(b, a); le = _len(e)
            if le < 1e-9: continue
            nx, ny = -e[1]/le * d, e[0]/le * d
            all_pts += [(a[0]+nx, a[1]+ny), (b[0]+nx, b[1]+ny)]
        for v in poly:
            for j in range(16):
                ang = 2*math.pi * j / 16
                all_pts.append((v[0]+d*math.cos(ang), v[1]+d*math.sin(ang)))
        return _ccw(_hull(all_pts))


# ── Anchored NFP / IFP ────────────────────────────────────────────────────────
# The raw library uses B[0] as the reference vertex.  After a convex hull
# B[0] is unpredictable.  We shift B so B[0]=(0,0), call the library, then
# un-shift the result.  This makes the reference = B's bbox-min corner = (0,0).

def _anchor(poly):
    """Return (shifted_poly, (dx,dy)) where shifted_poly[0] == (0,0)."""
    dx, dy = poly[0]
    return [(v[0]-dx, v[1]-dy) for v in poly], (dx, dy)

def compute_nfp(A, B):
    """NFP(A offset, B offset-normalised). Result in bin coords."""
    B_anch, (bax, bay) = _anchor(B)
    nfp = _compute_nfp_raw(A, B_anch)
    if not nfp: return []
    return [(v[0]-bax, v[1]-bay) for v in nfp]

def compute_bin_ifp(bin_rect, B):
    """IFP of B inside bin_rect. B bbox-normalised to (0,0)."""
    B_anch, (bax, bay) = _anchor(B)
    ifp = _compute_ifp_raw(bin_rect, B_anch)
    if not ifp: return []
    return [(v[0]-bax, v[1]-bay) for v in ifp]


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _sarea_py(poly):
    n = len(poly)
    return sum((poly[i][0]*poly[(i+1)%n][1] - poly[(i+1)%n][0]*poly[i][1])
               for i in range(n)) * 0.5

def _poly_area(poly):  return abs(_sarea_py(poly))

def _bbox(poly):
    return (min(v[0] for v in poly), min(v[1] for v in poly),
            max(v[0] for v in poly), max(v[1] for v in poly))

def _bboxes_overlap(b1, b2, eps=1e-6):
    return not (b1[2] < b2[0] - eps or b1[0] > b2[2] + eps or
                b1[3] < b2[1] - eps or b1[1] > b2[3] + eps)

def _to_origin(poly):
    """Shift poly so bbox-min = (0,0). Returns (shifted_poly, ox, oy)."""
    x0, y0, _, _ = _bbox(poly)
    return [(v[0]-x0, v[1]-y0) for v in poly], x0, y0

def _translate(poly, dx, dy):
    return [(v[0]+dx, v[1]+dy) for v in poly]

def _pip_py(pt, poly):
    x, y = pt; n, c = len(poly), 0; j = n-1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
            c += 1
        j = i
    return bool(c & 1)

def _on_edge(pt, poly, eps=1e-6):
    x, y = pt; n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i+1)%n]
        cr = (bx-ax)*(y-ay) - (by-ay)*(x-ax)
        if abs(cr) > eps: continue
        if (min(ax,bx)-eps <= x <= max(ax,bx)+eps and
                min(ay,by)-eps <= y <= max(ay,by)+eps):
            return True
    return False

def _in_ifp(pt, ifp): return _pip_py(pt, ifp) or _on_edge(pt, ifp)

def _polys_overlap(P, Q, eps=1e-6):
    """
    Hard overlap guard — exact polygon intersection.
    Offset shapes already enforce the gap; this catches NFP approximation errors.
    Two touching shapes (shared edge only) are NOT considered overlapping.
    """
    # Shrink each poly slightly inward before testing to allow edge-touching
    def centroid(p):
        n = len(p)
        return sum(v[0] for v in p)/n, sum(v[1] for v in p)/n
    def shrink(p, delta=eps*10):
        cx, cy = centroid(p)
        return [(v[0]+(cx-v[0])*delta, v[1]+(cy-v[1])*delta) for v in p]
    Ps, Qs = shrink(P), shrink(Q)
    for v in Ps:
        if _pip_py(v, Qs): return True
    for v in Qs:
        if _pip_py(v, Ps): return True
    def seg_x(a1, a2, b1, b2):
        def side(p, q, r):
            return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
        d1=side(b1,b2,a1); d2=side(b1,b2,a2)
        d3=side(a1,a2,b1); d4=side(a1,a2,b2)
        return (((d1>0 and d2<0) or (d1<0 and d2>0)) and
                ((d3>0 and d4<0) or (d3<0 and d4>0)))
    nP, nQ = len(P), len(Q)
    for i in range(nP):
        for j in range(nQ):
            if seg_x(P[i],P[(i+1)%nP], Q[j],Q[(j+1)%nQ]):
                return True
    return False


# ── Sampling + rotation ───────────────────────────────────────────────────────

def sample_polygon_edges(poly, n_per_edge=EDGE_SAMPLES):
    pts = list(poly); n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i+1)%n]
        for k in range(1, n_per_edge):
            t = k / n_per_edge
            pts.append((ax + t*(bx-ax), ay + t*(by-ay)))
    return pts

def rotate_poly(poly, degrees):
    if degrees == 0: return list(poly)
    rad = math.radians(degrees)
    cx = sum(v[0] for v in poly) / len(poly)
    cy = sum(v[1] for v in poly) / len(poly)
    c, s = math.cos(rad), math.sin(rad)
    return [(cx + (x-cx)*c - (y-cy)*s,
             cy + (x-cx)*s + (y-cy)*c) for x, y in poly]


# ── Preprocessing ─────────────────────────────────────────────────────────────

def sort_by_area(shapes):
    idx = sorted(range(len(shapes)),
                 key=lambda i: _poly_area(shapes[i]), reverse=True)
    return [shapes[i] for i in idx], idx

def build_rotation_set(base, grain_direction=False):
    return (0.0, 180.0) if grain_direction else tuple(float(r) for r in base)

def score_position(pos, shape_bbox, cur_bbox):
    """
    Gravity score: minimise the overall bounded area, then bottom-left placement.

    cur_bbox: (cx0,cy0,cx1,cy1) of everything placed so far.
    shape_bbox: (x0,y0,x1,y1) of the normalised shape at origin.
    pos: (px, py) = bottom-left corner of placed shape in bin coords.
    """
    x0, y0, x1, y1 = shape_bbox
    w = x1 - x0
    h = y1 - y0
    
    clx, cly, crx, cry = cur_bbox
    
    nrx = max(crx, pos[0] + w)
    nry = max(cry, pos[1] + h)
    
    new_area = nrx * nry
    return new_area + (pos[1] + h) + pos[0] * 0.01



# ── Single shape placement ────────────────────────────────────────────────────

def _place_one(shape, placed_vis, bin_rect, clearance, rotations):
    """
    Try all rotations of `shape`. Return best valid placement dict or None.

    placed_vis : list of OFFSET shapes in bin coords.
                 Used as A in NFP — ensures clearance on both sides.
                 Never drawn; only the raw outlines appear in output.

    result['placed']     = raw shape in bin coords  → drawn in SVG
    result['placed_raw'] = raw shape in bin coords  → used for area
    placed_vis entry     = offset shape in bin coords → used for NFP only
    """
    best_score = float('inf')
    best_pos   = None
    best_rot   = 0.0
    best_raw   = None   # raw shape at best_pos — for display AND area
    best_off   = None   # offset shape at best_pos — for placed_vis

    if placed_vis:
        crx = max(_bbox(pv)[2] for pv in placed_vis)
        cry = max(_bbox(pv)[3] for pv in placed_vis)
        cur_bbox = (0, 0, crx, cry)
    else:
        cur_bbox = (0, 0, 0, 0)

    for deg in rotations:
        rotated = rotate_poly(shape, deg)

        # 1. Normalise raw shape to origin
        norm_raw, rx, ry = _to_origin(rotated)

        # 2. Offset from norm_raw (same frame)
        off_b = offset_poly(norm_raw, clearance)
        if not off_b: continue

        # 3. Normalise offset — record the extra shift (fx, fy)
        norm_off, fx, fy = _to_origin(off_b)

        # 4. Align norm_raw to norm_off's origin
        #    off_b bbox-min sits at (-fx,-fy) relative to norm_raw's bbox-min.
        #    Shift norm_raw by (-fx,-fy) so both share the same origin.
        norm_raw_aligned = _translate(norm_raw, -fx, -fy)

        # Score needs the aligned raw shape's bbox
        raw_bbox = _bbox(norm_raw_aligned)

        # 5. IFP — where can norm_off's bbox-min go inside the bin?
        ifp = compute_bin_ifp(bin_rect, norm_off)
        if not ifp: continue

        # 6. NFP against every already-placed offset shape
        nfp_table = []
        for a in placed_vis:
            try:
                nfp = compute_nfp(a, norm_off)
                if nfp: nfp_table.append(nfp)
            except Exception:
                pass

        # 7. Candidate positions: IFP boundary + all NFP boundaries
        candidates = sample_polygon_edges(ifp, EDGE_SAMPLES)
        for nfp in nfp_table:
            candidates.extend(sample_polygon_edges(nfp, EDGE_SAMPLES))

        # 8. Test each candidate
        for pos in candidates:
            if not _in_ifp(pos, ifp): continue
            if any(_pip_py(pos, nfp) for nfp in nfp_table): continue

            # Hard guard: exact overlap check on offset shapes
            candidate_off = _translate(norm_off, pos[0], pos[1])
            cand_bbox = _bbox(candidate_off)
            if any(_bboxes_overlap(cand_bbox, _bbox(pv)) and _polys_overlap(candidate_off, pv) for pv in placed_vis):
                continue

            s = score_position(pos, raw_bbox, cur_bbox)
            if s < best_score:
                best_score = s
                best_pos   = pos
                best_rot   = deg
                best_raw   = _translate(norm_raw_aligned, pos[0], pos[1])
                best_off   = candidate_off

    if best_pos is None:
        return None

    return {
        'shape':      shape,
        'placed':     best_raw,   # ← RAW outline drawn in SVG + used for area
        'placed_raw': best_raw,   # same reference kept for backward compat
        '_placed_off': best_off,  # ← OFFSET shape; added to placed_vis only
        'position':   best_pos,
        'rotation':   best_rot,
    }


# ── Hole detection ────────────────────────────────────────────────────────────

def detect_holes(placed_vis, bin_w, bin_h,
                 step=HOLE_GRID_STEP, min_area=HOLE_MIN_AREA):
    """
    Grid-scan for free rectangular regions, using offset shapes so that
    clearance zones are included in the occupied mask.
    """
    if not placed_vis:
        return [{'rect': (0, 0, bin_w, bin_h), 'area': bin_w*bin_h}]

    max_y = max((_bbox(shape)[3] for shape in placed_vis), default=bin_h)
    scan_h = min(bin_h, max_y + step * 2)

    cols = max(1, int(bin_w / step))
    rows = max(1, int(scan_h / step))
    occ  = [[False]*cols for _ in range(rows)]

    for shape in placed_vis:
        x0, y0, x1, y1 = _bbox(shape)
        c0 = max(0, int(x0/step)); c1 = min(cols-1, int(x1/step)+1)
        r0 = max(0, int(y0/step)); r1 = min(rows-1, int(y1/step)+1)
        for r in range(r0, r1+1):
            cy = (r+0.5)*step
            for c in range(c0, c1+1):
                if _pip_py(((c+0.5)*step, cy), shape):
                    occ[r][c] = True

    holes = []; vis = [[False]*cols for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            if occ[r][c] or vis[r][c]: continue
            cells = []; q = [(r,c)]; vis[r][c] = True
            while q:
                rr, cc = q.pop(); cells.append((rr, cc))
                for dr, dc in ((0,1),(0,-1),(1,0),(-1,0)):
                    nr, nc = rr+dr, cc+dc
                    if (0 <= nr < rows and 0 <= nc < cols
                            and not occ[nr][nc] and not vis[nr][nc]):
                        vis[nr][nc] = True; q.append((nr, nc))
            if not cells: continue
            mr = min(rr for rr,_ in cells); Mr = max(rr for rr,_ in cells)
            mc = min(cc for _,cc in cells); Mc = max(cc for _,cc in cells)
            x = mc*step; y = mr*step
            w = (Mc-mc+1)*step; h = (Mr-mr+1)*step
            if w*h >= min_area:
                holes.append({'rect': (x, y, w, h), 'area': w*h})

    holes.sort(key=lambda h: h['area'], reverse=True)
    return holes


# ── Hole filling ──────────────────────────────────────────────────────────────

def fill_holes(holes, unplaced_idx, shapes, placed_vis,
               clearance, rotations):
    new_placements = []; remaining = list(unplaced_idx)

    for hole in holes[:HOLE_TOP_K]:
        hx, hy, hw, hh = hole['rect']
        hole_bin = [(hx,hy),(hx+hw,hy),(hx+hw,hy+hh),(hx,hy+hh)]

        for shape_idx in list(remaining[:HOLE_SHAPE_K]):
            shape = shapes[shape_idx]
            fits = False
            for deg in rotations:
                rot = rotate_poly(shape, deg); b = _bbox(rot)
                if ((b[2]-b[0]) + 2*clearance <= hw and
                        (b[3]-b[1]) + 2*clearance <= hh):
                    fits = True; break
            if not fits: continue

            p = _place_one(shape, placed_vis, hole_bin, clearance, rotations)
            if p is not None:
                placed_vis.append(p['_placed_off'])
                new_placements.append(
                    {**p, 'shape_idx': shape_idx, 'hole_filled': True})
                remaining.remove(shape_idx)
                break

    return new_placements, remaining


# ── Main nest() ───────────────────────────────────────────────────────────────

def nest(shapes, bin_w, bin_h,
         clearance=1.0,
         rotations=(0, 45, 90, 135, 180, 225, 270, 315),
         sequence=None,
         sort_area=True,
         grain_direction=False,
         bin_margin=0.0,
         enable_hole_fill=True):
    """
    Greedy nesting.

    Clearance (kerf + mechanical gap) is enforced by:
      - offsetting each incoming shape by `clearance` before computing NFP
      - computing NFP against the OFFSET silhouettes of already-placed shapes
    This gives exactly one gap of `clearance` mm between adjacent raw shapes.

    Utilisation:
      numerator   = sum of raw part areas
      denominator = bounding box of all raw placed shapes
                    (minimum sheet rectangle consumed, kerf NOT included in
                     denominator — denominator reflects the layout footprint)
      penalty     = placed / total  (discourages dropping shapes)
      utilisation = (numerator / denominator) * penalty * 100
    """
    ew = bin_w - 2*bin_margin
    eh = bin_h - 2*bin_margin
    ex, ey = bin_margin, bin_margin
    bin_rect = [(ex,ey),(ex+ew,ey),(ex+ew,ey+eh),(ex,ey+eh)]

    rot_set = build_rotation_set(rotations, grain_direction)

    if sequence is not None:
        order = list(sequence)
    elif sort_area:
        _, order = sort_by_area(shapes)
    else:
        order = list(range(len(shapes)))

    placed_vis = []   # offset shapes — NFP inputs only, never drawn
    result     = []
    unplaced   = []

    for idx in order:
        p = _place_one(shapes[idx], placed_vis, bin_rect, clearance, rot_set)
        if p is None:
            unplaced.append(idx); continue
        placed_vis.append(p['_placed_off'])   # offset shape for future NFP
        result.append({**p, 'shape_idx': idx, 'hole_filled': False})

    if enable_hole_fill and unplaced:
        holes = detect_holes(placed_vis, ew, eh, HOLE_GRID_STEP, HOLE_MIN_AREA)
        if holes:
            extras, _ = fill_holes(
                holes, unplaced, shapes, placed_vis, clearance, rot_set)
            result.extend(extras)

    # ── Utilisation ───────────────────────────────────────────────────────────
    # Denominator: bounding box of raw placed shapes × placement_rate penalty.
    # Raw shapes = what you actually cut. Their bbox = minimum consumed sheet.
    if not result:
        utilisation = 0.0
    else:
        placed_area = sum(_poly_area(r['placed_raw']) for r in result)

        all_x = [x for r in result for x, _ in r['placed']]
        all_y = [y for r in result for _, y in r['placed']]
        bbox_w    = max(all_x) - min(all_x)
        bbox_h    = max(all_y) - min(all_y)
        bbox_area = bbox_w * bbox_h

        placement_rate = len(result) / len(shapes)

        utilisation = (round((placed_area / bbox_area) * placement_rate * 100, 1)
                       if bbox_area > 0 else 0.0)
        density = (placed_area / bbox_area) if bbox_area > 0 else 0.0
        placement_rate = len(result) / len(shapes)

        #internal fitness (used by GA)
        utilisation = round(density * placement_rate * 100, 1)

        # reported metric (clean)
        density_percent = round(density * 100, 1)
                       

    return {
    'placements':    result,

    # internal (used by GA)
    'utilisation':   utilisation,

    # reported metrics
    'density':       density_percent,
    'placement_rate': round(placement_rate * 100, 1),

    'placed':        len(result),
    'total':         len(shapes),
    'bin':           (bin_w, bin_h),
    'sequence_used': order,
}


# ── Genetic Algorithm ─────────────────────────────────────────────────────────

def _ox(p1, p2):
    n = len(p1); a, b = sorted(random.sample(range(n), 2))
    child = [None]*n; child[a:b+1] = p1[a:b+1]
    seg = set(p1[a:b+1]); fill = [g for g in p2 if g not in seg]
    fi = 0
    for i in range(n):
        if child[i] is None: child[i] = fill[fi]; fi += 1
    return child

def _mutate(c, rate=0.1):
    c = c[:]
    if random.random() < rate:
        if random.random() < 0.5:
            i, j = random.sample(range(len(c)), 2); c[i], c[j] = c[j], c[i]
        else:
            i, j = sorted(random.sample(range(len(c)), 2))
            c[i:j+1] = c[i:j+1][::-1]
    return c

def _tournament(pop, fits, k=3):
    cont = random.sample(list(zip(pop, fits)), k)
    return max(cont, key=lambda x: x[1])[0]

def ga_nest(shapes, bin_w, bin_h,
            clearance=1.0,
            rotations=(0, 45, 90, 135, 180, 225, 270, 315),
            grain_direction=False,
            bin_margin=0.0,
            enable_hole_fill=True,
            pop_size=20,
            generations=30,
            mutation_rate=0.15,
            elite_k=2,
            time_limit=120.0,
            verbose=True):
    """
    GA wrapper around nest().

    Chromosome : permutation of shape indices [0..n-1]
    Fitness    : utilisation % (raw bbox density × placement rate)
    Selection  : tournament (k=3)
    Crossover  : Order Crossover (OX)
    Mutation   : swap, linearly annealed mutation_rate → 0.02
    """
    n = len(shapes); t0 = time.time()

    kw = dict(bin_w=bin_w, bin_h=bin_h, clearance=clearance,
              rotations=rotations, grain_direction=grain_direction,
              bin_margin=bin_margin, enable_hole_fill=enable_hole_fill,
              sort_area=False)

    def evaluate(seq):
        r = nest(shapes, sequence=seq, **kw)
        return r['utilisation'], r

    _, area_order = sort_by_area(shapes)
    pop = [area_order[:]] + [random.sample(range(n), n)
                              for _ in range(pop_size-1)]

    fits = []; ress = []
    for ch in pop:
        f, r = evaluate(ch); fits.append(f); ress.append(r)

    bi = max(range(pop_size), key=lambda i: fits[i])
    best_fit = fits[bi]; best_res = ress[bi]

    if verbose:
        print(f"[GA] gen  0  best={best_fit:.1f}%  "
              f"mean={sum(fits)/len(fits):.1f}%")

    history = [best_fit]
    MUTATION_FLOOR = 0.02

    for gen in range(1, generations+1):
        if time.time()-t0 > time_limit:
            if verbose: print(f"[GA] time limit at gen {gen}"); break

        cur_rate = (mutation_rate
                    - (mutation_rate - MUTATION_FLOOR) * (gen / generations))

        ranked = sorted(zip(pop, fits, ress), key=lambda x: x[1], reverse=True)
        pop  = [r[0] for r in ranked]
        fits = [r[1] for r in ranked]
        ress = [r[2] for r in ranked]

        new_pop = pop[:elite_k]
        while len(new_pop) < pop_size:
            new_pop.append(_mutate(
                _ox(_tournament(pop, fits), _tournament(pop, fits)),
                cur_rate))

        new_fits = fits[:elite_k][:]
        new_ress = ress[:elite_k][:]
        for ch in new_pop[elite_k:]:
            f, r = evaluate(ch); new_fits.append(f); new_ress.append(r)

        pop = new_pop; fits = new_fits; ress = new_ress
        gi = max(range(pop_size), key=lambda i: fits[i])
        if fits[gi] > best_fit:
            best_fit = fits[gi]; best_res = ress[gi]

        history.append(best_fit)
        if verbose and (gen % 5 == 0 or gen == generations):
            print(f"[GA] gen {gen:3d}  best={best_fit:.1f}%  "
                  f"mean={sum(fits)/len(fits):.1f}%  "
                  f"rate={cur_rate:.3f}  ({time.time()-t0:.1f}s)")

    elapsed = time.time() - t0
    if verbose:
        print(f"[GA] done {elapsed:.1f}s — best: {best_fit:.1f}%")

    best_res['ga_stats'] = {
        'generations':      len(history),
        'best_utilisation': best_fit,
        'history':          history,
        'elapsed_s':        round(elapsed, 2),
    }
    return best_res