# Otimizador de posicionamento de PCB com Algoritmo Genético (KiCad / pcbnew)
#
# Hierarquia de regras (da mais forte para a mais fraca):
#   1. CONTENÇÃO  - todo footprint termina dentro do contorno Edge.Cuts. É garantida
#                   por construção (o AG só trabalha com posições válidas) e conferida
#                   de novo na placa real depois de mover.
#   2. SOBREPOSIÇÃO - corpos não se invadem (folga mínima). Pesa mais que qualquer distância.
#   3. ROTEAMENTO - cristal, desacoplamento, antena etc. perto dos pinos certos.
#
# A única exceção física à regra 1 é um componente MAIOR que a placa: ele é
# centralizado e o script informa a falha de forma explícita.
#
# Uso no terminal (Python do KiCad):  python script_IA.py placa.kicad_pcb
# Uso no console do KiCad:            script_IA.run_ga_placement(pcbnew.GetBoard())

import argparse
import csv
import math
import os
import random
import sys
from dataclasses import dataclass, field

import pcbnew


# ----------------------------------------------------------------------------
# REGRAS DA PLACA
# ----------------------------------------------------------------------------
@dataclass
class BoardRules:
    movable_refs: list                  # componentes que o AG pode otimizar
    anchor_refs: list                   # CIs de referência (só se movem se estiverem fora)
    fallback_map: dict                  # {ref: [(CI, pino), ...]} usado se não houver net
    ignore_net_keywords: tuple = ("GND",)   # nets ignoradas (terra não "atrai")
    clearance_mm: float = 0.25          # folga mínima entre corpos
    edge_margin_mm: float = 0.30        # folga mínima até a borda da placa
    default_weight: float = 10.0        # peso padrão da distância
    weights: dict = field(default_factory=dict)       # peso por referência
    init_radius_mm: float = 5.0         # raio da população inicial ao redor do alvo
    attract_to: dict = field(default_factory=dict)    # {ref: [âncoras que o atraem]}
    edge_weights: dict = field(default_factory=dict)  # atração pela borda da placa
    group_nets: tuple = ()              # nets cujos componentes ficam agrupados
    group_weight: float = 12.0
    ratsnest_weight: float = 1.0        # atração fraca para quem não tem regra própria
    anchor_gap_mm: float = 6.0          # espaço desejado entre CIs que precisem ser reposicionados


BOARD_RULES = BoardRules(
    movable_refs=["Y1", "Y2", "F1", "C1", "C2", "C3", "R2", "R3", "L1", "L2",
                  "R4", "L3", "C4", "J1"],
    anchor_refs=["U1", "U2", "BT2"],    # a ordem importa: o primeiro é o mais central
    group_nets=("ANT_SIG",),
    group_weight=12.0,
    edge_weights={"J1": 30.0, "R4": 5.0, "L3": 5.0, "C4": 5.0},
    attract_to={
        "R4": ["U1"], "L3": ["U1"], "C4": ["U1"], "J1": ["U1"],   # antena -> ESP32-S2
        "Y1": ["U1"],     # cristal do micro  -> ESP32-S2
        "Y2": ["U2"],     # oscilador externo -> acelerômetro
        "F1": ["BT2"],    # fusível           -> conector da bateria
        "C1": ["U1"], "C3": ["U1"], "L1": ["U1"], "L2": ["U1"],   # desacoplamento do micro
        "R2": ["U1"], "R3": ["U1"],
        "C2": ["U2"],     # desacoplamento do acelerômetro
    },
    # Usado só se a net não for encontrada na PCB (confira a numeração dos footprints):
    fallback_map={
        "Y1": [("U1", "52"), ("U1", "53")],   # XTAL_N / XTAL_P
        "Y2": [("U2", "3")],                  # SCX (ACC_CLK_EXT)
        "F1": [("BT2", "1")],                 # pino + da bateria
        "C1": [("U1", "1")], "L1": [("U1", "1")], "R3": [("U1", "1")],   # VDDA
        "C3": [("U1", "3")], "L2": [("U1", "3")], "R2": [("U1", "3")],   # VDD3P3
        "C2": [("U2", "8")],                  # VDD do LSM6DS3
        "R4": [("U1", "2")], "L3": [("U1", "2")], "C4": [("U1", "2")], "J1": [("U1", "2")],  # LNA_IN
    },
    weights={"Y1": 25.0, "Y2": 20.0, "F1": 15.0, "C1": 15.0, "C2": 15.0, "C3": 15.0,
             "R4": 12.0, "L3": 12.0, "C4": 12.0, "J1": 4.0},
)

ORIENTATIONS = (0, 90, 180, 270)
EPS = 1e-5   # tolerância numérica em mm (10 nm)


# ----------------------------------------------------------------------------
# GEOMETRIA
# ----------------------------------------------------------------------------
def mm(v):
    return pcbnew.ToMM(v)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def rot(x, y, deg):
    """Rotação no sentido do KiCad (Y para baixo, ângulo positivo = anti-horário)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x * c + y * s, -x * s + y * c)


def box_to_rect(bb):
    x0, y0 = mm(bb.GetX()), mm(bb.GetY())
    return (x0, y0, x0 + mm(bb.GetWidth()), y0 + mm(bb.GetHeight()))


def union(rects):
    return (min(r[0] for r in rects), min(r[1] for r in rects),
            max(r[2] for r in rects), max(r[3] for r in rects))


def courtyard_rects(fp):
    out = []
    try:
        fp.BuildCourtyardCaches()
    except Exception:
        pass
    for layer in ("F_CrtYd", "B_CrtYd"):
        try:
            poly = fp.GetCourtyard(getattr(pcbnew, layer))
            if poly.OutlineCount() > 0:
                out.append(box_to_rect(poly.BBox()))
        except Exception:
            pass
    return out


def measure_rects(fp):
    """(caixa completa, caixa essencial) do footprint na posição atual, sem textos.
    completa  = desenhos + pads + courtyard
    essencial = pads + courtyard (usada só se a completa não couber na placa)"""
    try:
        body = box_to_rect(fp.GetBoundingBox(False))
    except TypeError:
        body = box_to_rect(fp.GetBoundingBox(False, False))
    core = courtyard_rects(fp) + [box_to_rect(p.GetBoundingBox()) for p in fp.Pads()]
    core = union(core) if core else body
    return union([body, core]), core


class Part:
    """Footprint descrito em orientação 0°, relativo ao CENTRO da sua caixa completa."""

    def __init__(self, fp):
        self.fp = fp
        self.ref = fp.GetReference()
        pos = fp.GetPosition()
        self.start_pos = (mm(pos.x), mm(pos.y))
        self.start_ori = fp.GetOrientationDegrees()
        self.start_full, self.start_core = measure_rects(fp)

        # Mede em 0° para não "inchar" a caixa de componentes que estejam em 45°, por exemplo
        fp.SetOrientationDegrees(0)
        try:
            full, core = measure_rects(fp)
            px, py = self.start_pos
            cx, cy = (full[0] + full[2]) / 2, (full[1] + full[3]) / 2
            self.off = (cx - px, cy - py)                       # centro - origem do footprint
            self.full = (full[0] - cx, full[1] - cy, full[2] - cx, full[3] - cy)
            self.core = (core[0] - cx, core[1] - cy, core[2] - cx, core[3] - cy)
            self.pads = []
            self.through = False
            for pad in fp.Pads():
                p = pad.GetPosition()
                self.pads.append((str(pad.GetNumber()), pad.GetNetname(), mm(p.x) - cx, mm(p.y) - cy))
                try:
                    self.through = self.through or bool(pad.HasHole())
                except Exception:
                    pass
        finally:
            fp.SetOrientationDegrees(self.start_ori)

        self.back = bool(fp.IsFlipped())
        self.locked = bool(fp.IsLocked())
        self.area = (self.full[2] - self.full[0]) * (self.full[3] - self.full[1])
        ox, oy = rot(self.off[0], self.off[1], self.start_ori)
        self.start_center = (self.start_pos[0] + ox, self.start_pos[1] + oy)
        self.start_gene = (self.start_center[0], self.start_center[1],
                           int(round(self.start_ori / 90.0)) * 90 % 360)

    @staticmethod
    def _turn(box, gene):
        x, y, ori = gene
        ax, ay = rot(box[0], box[1], ori)
        bx, by = rot(box[2], box[3], ori)
        return (x + min(ax, bx), y + min(ay, by), x + max(ax, bx), y + max(ay, by))

    def rect(self, gene):
        return self._turn(self.full, gene)

    def core_rect(self, gene):
        return self._turn(self.core, gene)

    def pad_xy(self, gene, lx, ly):
        dx, dy = rot(lx, ly, gene[2])
        return (gene[0] + dx, gene[1] + dy)

    def origin(self, gene):
        """Posição que deve ser gravada no footprint para o centro ficar em gene."""
        ox, oy = rot(self.off[0], self.off[1], gene[2])
        return (gene[0] - ox, gene[1] - oy)


class Outline:
    """Contorno da placa: um ou mais laços (bordas externas e recortes internos)."""

    def __init__(self, loops):
        self.loops = loops
        self.segs = [(l[i], l[(i + 1) % len(l)]) for l in loops for i in range(len(l))]
        xs = [p[0] for l in loops for p in l]
        ys = [p[1] for l in loops for p in l]
        self.bounds = (min(xs), min(ys), max(xs), max(ys))
        self.center = ((self.bounds[0] + self.bounds[2]) / 2, (self.bounds[1] + self.bounds[3]) / 2)

    def point_inside(self, px, py):
        inside = False
        for (ax, ay), (bx, by) in self.segs:          # regra par-ímpar: recortes ficam "fora"
            if (ay > py) != (by > py) and px < (bx - ax) * (py - ay) / (by - ay) + ax:
                inside = not inside
        return inside

    def contains(self, rect, margin=0.0):
        """Teste exato: nenhum segmento do contorno atravessa o retângulo (com margem)
        e o centro dele está dentro da placa."""
        x0, y0 = rect[0] - margin + EPS, rect[1] - margin + EPS
        x1, y1 = rect[2] + margin - EPS, rect[3] + margin - EPS
        for (ax, ay), (bx, by) in self.segs:
            if ((ax <= x0 and bx <= x0) or (ax >= x1 and bx >= x1) or
                    (ay <= y0 and by <= y0) or (ay >= y1 and by >= y1)):
                continue
            t0, t1 = 0.0, 1.0                           # recorte de Liang-Barsky
            dx, dy = bx - ax, by - ay
            hit = True
            for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
                if p == 0:
                    if q <= 0:
                        hit = False
                        break
                else:
                    t = q / p
                    if p < 0:
                        t0 = max(t0, t)
                    else:
                        t1 = min(t1, t)
            if hit and t0 < t1:
                return False
        return self.point_inside((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2)

    def distance(self, rect):
        """Menor distância entre o corpo (cantos e meios dos lados) e a borda."""
        x0, y0, x1, y1 = rect
        xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
        best = float("inf")
        for px, py in ((x0, y0), (xm, y0), (x1, y0), (x0, ym), (x1, ym), (x0, y1), (xm, y1), (x1, y1)):
            for (ax, ay), (bx, by) in self.segs:
                dx, dy = bx - ax, by - ay
                L2 = dx * dx + dy * dy
                t = 0.0 if L2 == 0 else clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
                best = min(best, math.hypot(px - ax - t * dx, py - ay - t * dy))
        return best


def read_outline(board):
    """Lê o Edge.Cuts. Nunca usa a caixa dos componentes como substituto da placa."""
    poly = pcbnew.SHAPE_POLY_SET()
    try:
        ok = board.GetBoardPolygonOutlines(poly, False)   # KiCad recente
    except TypeError:
        poly = pcbnew.SHAPE_POLY_SET()
        ok = board.GetBoardPolygonOutlines(poly)          # versões anteriores

    def points(chain):
        return [(mm(chain.CPoint(i).x), mm(chain.CPoint(i).y)) for i in range(chain.PointCount())]

    loops = []
    if ok is not False:
        for i in range(poly.OutlineCount()):
            loops.append(points(poly.COutline(i)))
            for j in range(poly.HoleCount(i)):
                hole = poly.CHole(i, j) if hasattr(poly, "CHole") else poly.Hole(i, j)
                loops.append(points(hole))
    loops = [l for l in loops if len(l) >= 3]
    if not loops:
        raise RuntimeError("Contorno da placa não encontrado. Desenhe um contorno FECHADO na "
                           "camada Edge.Cuts. Nada foi alterado.")
    return Outline(loops)


def has_edge_cuts(fp):
    """Footprints que desenham Edge.Cuts fazem parte do contorno e não devem ser movidos."""
    try:
        return any(g.GetLayer() == pcbnew.Edge_Cuts for g in fp.GraphicalItems())
    except Exception:
        return False


def linspace(lo, hi, step):
    if hi < lo - 2 * EPS:
        return []
    if hi <= lo:
        return [(lo + hi) / 2]
    n = int(math.ceil((hi - lo) / step))
    return [lo + (hi - lo) * i / n for i in range(n + 1)]


# ----------------------------------------------------------------------------
# CONTEXTO: quem se move, onde pode ficar e o que atrai cada um
# ----------------------------------------------------------------------------
class Context:
    def __init__(self, board, rules, rng):
        self.rules = rules
        self.rng = rng
        self.outline = read_outline(board)
        self.warnings = []
        self.impossible = []

        self.parts = {}
        for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
            part = Part(fp)
            if part.ref in self.parts:
                raise RuntimeError(f"Referência repetida na placa: {part.ref}. Reanote o esquemático.")
            self.parts[part.ref] = part

        for ref in rules.movable_refs + rules.anchor_refs:
            if ref not in self.parts:
                self.warnings.append(f"{ref} está nas regras, mas não existe na PCB (ignorado).")

        # 1) Quem está fora da placa? (vale para QUALQUER footprint, listado ou não)
        self.outside = [r for r, p in self.parts.items()
                        if not has_edge_cuts(p.fp) and not self.outline.contains(p.start_full)]
        self.anchors_out = [r for r in rules.anchor_refs if r in self.outside]

        # 2) Quem o AG otimiza: móveis das regras (destravados) + qualquer um que esteja fora
        self.movable = []
        for ref in rules.movable_refs:
            p = self.parts.get(ref)
            if p and not has_edge_cuts(p.fp) and (not p.locked or ref in self.outside):
                self.movable.append(ref)
        self.movable += [r for r in self.outside
                         if r not in self.movable and r not in self.anchors_out]
        for ref in self.outside:
            if self.parts[ref].locked:
                self.warnings.append(f"{ref} estava travado FORA da placa: será movido para dentro.")

        # 3) Para cada peça a posicionar, define margem/caixa que cabem e as rotações possíveis
        self.fit = {}      # ref -> (usa_caixa_essencial, margem, rotações válidas)
        self._grid = {}
        for ref in self.anchors_out + self.movable:
            self._define_fit(ref)

        placing = set(self.anchors_out + self.movable)
        self.fixed = {r: p.rect(p.start_gene) if p.start_ori % 90 == 0 else p.start_full
                      for r, p in self.parts.items() if r not in placing}
        self.genes_anchor = {}

    # ---------- posições válidas ----------
    def _define_fit(self, ref):
        m = self.rules.edge_margin_mm
        for core, margin in ((False, m), (False, 0.0), (True, 0.0)):
            self.fit[ref] = (core, margin, ORIENTATIONS)
            oris = tuple(o for o in ORIENTATIONS if self.positions(ref, o))
            if oris:
                self.fit[ref] = (core, margin, oris)
                if core:
                    self.warnings.append(f"{ref}: só cabe considerando pads+courtyard "
                                         f"(desenhos podem passar da borda).")
                elif margin < m:
                    self.warnings.append(f"{ref}: só cabe sem a margem de borda de {m} mm.")
                return
        self.fit[ref] = (False, 0.0, ())
        self.impossible.append(ref)

    def valid(self, ref, gene):
        core, margin, oris = self.fit[ref]
        if gene[2] not in oris:
            return False
        p = self.parts[ref]
        return self.outline.contains(p.core_rect(gene) if core else p.rect(gene), margin)

    def positions(self, ref, ori):
        """Grade de centros válidos (contenção garantida). Calculada uma vez e guardada."""
        core, margin, _ = self.fit[ref]
        key = (ref, ori, core, margin)
        if key in self._grid:
            return self._grid[key]
        p = self.parts[ref]
        r = (p.core_rect if core else p.rect)((0.0, 0.0, ori))
        bx0, by0, bx1, by1 = self.outline.bounds
        lo_x, hi_x = bx0 + margin - r[0], bx1 - margin - r[2]
        lo_y, hi_y = by0 + margin - r[1], by1 - margin - r[3]
        area = max((bx1 - bx0) * (by1 - by0), 1.0)
        found = []
        for step, aligned in ((max(0.25, math.sqrt(area / 4000)), False),
                              (max(0.05, math.sqrt(area / 60000)), True)):
            xs, ys = linspace(lo_x, hi_x, step), linspace(lo_y, hi_y, step)
            if not xs or not ys:
                break
            verts = [v for l in self.outline.loops for v in l]
            if aligned and len(verts) <= 60:      # encaixes justos junto aos vértices
                xs += [x for v in verts for x in (v[0] - r[0] + margin, v[0] - r[2] - margin)
                       if lo_x <= x <= hi_x]
                ys += [y for v in verts for y in (v[1] - r[1] + margin, v[1] - r[3] - margin)
                       if lo_y <= y <= hi_y]
            found = [(x, y) for x in xs for y in ys
                     if self.outline.contains((x + r[0], y + r[1], x + r[2], y + r[3]), margin)]
            if found:
                break
        self._grid[key] = found
        return found

    def repair(self, ref, gene):
        """Devolve sempre um gene DENTRO da placa (o mais próximo possível do pedido)."""
        core, margin, oris = self.fit[ref]
        x, y, ori = gene
        if ref in self.impossible:
            return (self.outline.center[0], self.outline.center[1], 0)
        if ori not in oris:
            ori = self.rng.choice(oris)
        if self.valid(ref, (x, y, ori)):
            return (x, y, ori)
        p = self.parts[ref]
        r = (p.core_rect if core else p.rect)((0.0, 0.0, ori))
        bx0, by0, bx1, by1 = self.outline.bounds
        cx = clamp(x, bx0 + margin - r[0], bx1 - margin - r[2])
        cy = clamp(y, by0 + margin - r[1], by1 - margin - r[3])
        if self.valid(ref, (cx, cy, ori)):
            return (cx, cy, ori)
        nx, ny = min(self.positions(ref, ori), key=lambda q: (q[0] - x) ** 2 + (q[1] - y) ** 2)
        return (nx, ny, ori)

    def random_gene(self, ref):
        if ref in self.impossible:
            return self.repair(ref, (0, 0, 0))
        ori = self.rng.choice(self.fit[ref][2])
        x, y = self.rng.choice(self.positions(ref, ori))
        return (x, y, ori)

    # ---------- atrações ----------
    def _net_ok(self, net):
        return bool(net) and not any(k.upper() in net.upper() for k in self.rules.ignore_net_keywords)

    def _is_group(self, net):
        return bool(net) and any(k.upper() in net.upper() for k in self.rules.group_nets)

    def build_connections(self):
        """Chamado depois que as âncoras já têm posição definitiva."""
        rules = self.rules
        static = {r: self.genes_anchor.get(r, p.start_gene)
                  for r, p in self.parts.items() if r not in self.movable}

        def world_pads(ref):
            p = self.parts[ref]
            if ref in self.genes_anchor or p.start_ori % 90 == 0:
                return [(n, net) + p.pad_xy(static[ref], lx, ly) for n, net, lx, ly in p.pads]
            out = []                                    # fixo em ângulo qualquer: lê da placa
            for pad in p.fp.Pads():
                q = pad.GetPosition()
                out.append((str(pad.GetNumber()), pad.GetNetname(), mm(q.x), mm(q.y)))
            return out

        pads = {r: world_pads(r) for r in static}
        self.targets = {}   # ref -> [(lx, ly, [pontos], peso, descrição)]
        for ref in self.movable:
            part = self.parts[ref]
            weight = rules.weights.get(ref, rules.default_weight)
            allowed = [a for a in rules.attract_to.get(ref, rules.anchor_refs) if a in pads]
            conns = []
            if ref in rules.movable_refs:
                for _, net, lx, ly in part.pads:
                    if not self._net_ok(net):
                        continue
                    for a in allowed:
                        found = [(n, x, y) for n, an, x, y in pads[a] if an == net]
                        if found:
                            pins = ",".join(sorted({n for n, _, _ in found}))
                            conns.append((lx, ly, [(x, y) for _, x, y in found], weight,
                                          f"{a} pino {pins} (net {net})"))
                if not conns:
                    for a, pin in rules.fallback_map.get(ref, []):
                        found = [(x, y) for n, _, x, y in pads.get(a, []) if n == pin]
                        if found:
                            conns.append((0.0, 0.0, found, weight, f"{a} pino {pin} (mapa manual)"))
            self.targets[ref] = conns

        # Ligações entre pads: agrupamento (ex.: antena) e ratsnest fraco para os demais
        self.links = []     # (refA, ax, ay, refB | None, bx, by, peso); se refB é None, b é fixo
        for i, ref in enumerate(self.movable):
            has_rule = bool(self.targets[ref])
            for _, net, lx, ly in self.parts[ref].pads:
                if not self._net_ok(net):
                    continue
                group = self._is_group(net)
                for other in self.movable[i + 1:]:
                    weak = not has_rule or not self.targets[other]
                    w = rules.group_weight if group else (rules.ratsnest_weight if weak else 0.0)
                    for _, onet, ox, oy in self.parts[other].pads:
                        if onet == net and w > 0:
                            self.links.append((ref, lx, ly, other, ox, oy, w))
                if not has_rule and rules.ratsnest_weight > 0:
                    pts = [(x, y) for r in pads for _, an, x, y in pads[r] if an == net]
                    if pts:
                        self.targets[ref].append((lx, ly, pts, rules.ratsnest_weight,
                                                  f"ratsnest (net {net})"))
        self.links_of = {r: [l for l in self.links if r in (l[0], l[3])] for r in self.movable}

    def describe(self):
        for ref in self.movable:
            alvos = sorted({c[4] for c in self.targets[ref]})
            if self.rules.edge_weights.get(ref, 0) > 0:
                alvos.append("borda da placa")
            print(f"[INFO] {ref} -> {'; '.join(alvos) or '(só contenção e sobreposição)'}")


# ----------------------------------------------------------------------------
# FITNESS  (comparação em ordem de prioridade: sobreposição, depois custo)
# ----------------------------------------------------------------------------
def overlap(r1, r2, clearance):
    dx = min(r1[2], r2[2]) - max(r1[0], r2[0]) + clearance
    dy = min(r1[3], r2[3]) - max(r1[1], r2[1]) + clearance
    if dx > EPS and dy > EPS:
        return 0.5 + min(dx, dy)      # base fixa + profundidade (dá direção ao AG)
    return 0.0


def collides(ctx, a, b):
    """Lados opostos só colidem se um deles tiver furo passante."""
    pa, pb = ctx.parts[a], ctx.parts[b]
    return pa.back == pb.back or pa.through or pb.through


def own_cost(ctx, ref, gene):
    p = ctx.parts[ref]
    cost = 0.0
    for lx, ly, pts, w, _ in ctx.targets[ref]:
        px, py = p.pad_xy(gene, lx, ly)
        cost += w * min(math.hypot(px - ax, py - ay) for ax, ay in pts)
    we = ctx.rules.edge_weights.get(ref, 0.0)
    if we > 0:
        cost += we * ctx.outline.distance(p.rect(gene))
    return cost


def link_cost(ctx, link, ind):
    a, ax, ay, b, bx, by, w = link
    if a not in ind or b not in ind:      # indivíduo parcial (construção gulosa)
        return 0.0
    p = ctx.parts[a].pad_xy(ind[a], ax, ay)
    q = ctx.parts[b].pad_xy(ind[b], bx, by)
    return w * math.hypot(p[0] - q[0], p[1] - q[1])


def evaluate(ind, ctx):
    """(sobreposição total, custo de roteamento). Menor é melhor."""
    clearance = ctx.rules.clearance_mm
    refs = list(ind)
    rects = [ctx.parts[r].rect(ind[r]) for r in refs]
    over = cost = 0.0
    for i, a in enumerate(refs):
        cost += own_cost(ctx, a, ind[a])
        for j in range(i + 1, len(refs)):
            v = overlap(rects[i], rects[j], clearance)
            if v and collides(ctx, a, refs[j]):
                over += v
        for f, fr in ctx.fixed.items():
            v = overlap(rects[i], fr, clearance)
            if v and collides(ctx, a, f):
                over += v
    cost += sum(link_cost(ctx, l, ind) for l in ctx.links)
    return (over, cost)


def evaluate_one(ind, ref, gene, ctx):
    """Parcela da avaliação que depende só de um componente (rápida, para busca local)."""
    clearance = ctx.rules.clearance_mm
    rect = ctx.parts[ref].rect(gene)
    over = 0.0
    for other, g in ind.items():
        if other != ref:
            v = overlap(rect, ctx.parts[other].rect(g), clearance)
            if v and collides(ctx, ref, other):
                over += v
    for f, fr in ctx.fixed.items():
        v = overlap(rect, fr, clearance)
        if v and collides(ctx, ref, f):
            over += v
    trial = dict(ind)
    trial[ref] = gene
    return (over, own_cost(ctx, ref, gene) + sum(link_cost(ctx, l, trial) for l in ctx.links_of[ref]))


def better(a, b):
    if a[0] < b[0] - 1e-9:
        return True
    return a[0] <= b[0] + 1e-9 and a[1] < b[1] - 1e-9


# ----------------------------------------------------------------------------
# CIs (âncoras) que estejam fora da placa são posicionados primeiro
# ----------------------------------------------------------------------------
def place_anchors(ctx):
    placed = {r: rect for r, rect in ctx.fixed.items() if r not in ctx.movable}
    for ref in ctx.anchors_out:
        others = [placed[a] for a in ctx.rules.anchor_refs if a in placed]

        def score(g):
            rect = ctx.parts[ref].rect(g)
            over = sum(overlap(rect, r, ctx.rules.clearance_mm)
                       for f, r in placed.items() if collides(ctx, ref, f))
            crowd = sum(max(0.0, ctx.rules.anchor_gap_mm -
                            max(rect[0] - r[2], r[0] - rect[2], rect[1] - r[3], r[1] - rect[3]))
                        for r in others)
            center = math.hypot(g[0] - ctx.outline.center[0], g[1] - ctx.outline.center[1])
            return (over, crowd, center)

        if ref in ctx.impossible:
            gene = ctx.repair(ref, (0, 0, 0))
        else:
            gene = min(((x, y, o) for o in ctx.fit[ref][2] for x, y in ctx.positions(ref, o)), key=score)
        ctx.genes_anchor[ref] = gene
        placed[ref] = ctx.fixed[ref] = ctx.parts[ref].rect(gene)


# ----------------------------------------------------------------------------
# OPERADORES GENÉTICOS
# ----------------------------------------------------------------------------
def near_target(ctx, ref):
    """Gene aleatório perto do alvo elétrico (ou em qualquer ponto válido)."""
    conns = ctx.targets[ref]
    edge = ctx.rules.edge_weights.get(ref, 0.0) > 0
    if conns and not (edge and ctx.rng.random() < 0.5):
        ax, ay = ctx.rng.choice(ctx.rng.choice(conns)[2])
        r = ctx.rules.init_radius_mm
        return ctx.repair(ref, (ax + ctx.rng.uniform(-r, r), ay + ctx.rng.uniform(-r, r),
                                ctx.rng.choice(ORIENTATIONS)))
    if edge:
        (a, b), t = ctx.rng.choice(ctx.outline.segs), ctx.rng.random()
        return ctx.repair(ref, (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                                ctx.rng.choice(ORIENTATIONS)))
    return ctx.random_gene(ref)


def greedy_individual(ctx, tries=30):
    """Coloca um componente por vez (os prioritários primeiro) escolhendo o melhor candidato."""
    order = sorted(ctx.movable, key=lambda r: -ctx.rules.weights.get(r, ctx.rules.default_weight)
                   * (1 + ctx.rng.random()))
    ind = {}
    for ref in order:
        cands = [near_target(ctx, ref) for _ in range(tries)]
        ind[ref] = min(cands, key=lambda g: evaluate_one(ind, ref, g, ctx))
    return {r: ind[r] for r in ctx.movable}


def random_individual(ctx):
    return {r: near_target(ctx, r) for r in ctx.movable}


def tournament(population, scores, ctx, k=3):
    return population[min(ctx.rng.sample(range(len(population)), k), key=lambda i: scores[i])]


def crossover_uniform(p1, p2, ctx):
    c1, c2 = {}, {}
    for ref in p1:
        if ctx.rng.random() < 0.5:
            c1[ref], c2[ref] = p1[ref], p2[ref]
        else:
            c1[ref], c2[ref] = p2[ref], p1[ref]
    return c1, c2


def mutation(ind, rate, ctx, step):
    rng = ctx.rng
    out = dict(ind)
    for ref, (x, y, ori) in ind.items():
        if rng.random() >= rate:
            continue
        kind = rng.random()
        if kind < 0.05:
            out[ref] = near_target(ctx, ref)                    # recomeça perto do alvo
            continue
        if kind < 0.20:
            ori = rng.choice(ORIENTATIONS)                      # gira
        s = step * (8.0 if rng.random() < 0.1 else 1.0)         # salto ocasional
        out[ref] = ctx.repair(ref, (x + rng.gauss(0, s), y + rng.gauss(0, s), ori))
    if len(out) > 1 and rng.random() < 0.15:                    # troca dois componentes de lugar
        a, b = rng.sample(list(out), 2)
        ga, gb = out[a], out[b]
        out[a] = ctx.repair(a, (gb[0], gb[1], ga[2]))
        out[b] = ctx.repair(b, (ga[0], ga[1], gb[2]))
    return out


def remove_overlaps(ind, ctx, passes=4):
    """Para cada componente em conflito, procura em toda a placa a melhor posição livre."""
    cur = dict(ind)
    for _ in range(passes):
        changed = False
        for ref in sorted(ctx.movable, key=lambda r: -ctx.parts[r].area):
            now = evaluate_one(cur, ref, cur[ref], ctx)
            if now[0] <= 0 or ref in ctx.impossible:
                continue
            x0, y0 = cur[ref][:2]
            best, best_s = cur[ref], now
            for o in ctx.fit[ref][2]:
                for x, y in sorted(ctx.positions(ref, o),
                                   key=lambda q: (q[0] - x0) ** 2 + (q[1] - y0) ** 2):
                    s = evaluate_one(cur, ref, (x, y, o), ctx)
                    if better(s, best_s):
                        best, best_s = (x, y, o), s
            if best != cur[ref]:
                cur[ref], changed = best, True
        if evaluate(cur, ctx)[0] <= 0 or not changed:
            break
    if evaluate(cur, ctx)[0] > 0:       # último recurso: remonta tudo, dos maiores para os menores
        packed = {}
        for ref in sorted(ctx.movable, key=lambda r: -ctx.parts[r].area):
            if ref in ctx.impossible:
                packed[ref] = cur[ref]
                continue
            cands = ((x, y, o) for o in ctx.fit[ref][2] for x, y in ctx.positions(ref, o))
            packed[ref] = min(cands, key=lambda g: evaluate_one(packed, ref, g, ctx))
        packed = {r: packed[r] for r in ctx.movable}
        if better(evaluate(packed, ctx), evaluate(cur, ctx)):
            cur = packed
    return cur


def local_refinement(ind, ctx):
    """Ajuste fino: passos cada vez menores, aceitando só melhorias."""
    cur = dict(ind)
    for step in (1.0, 0.5, 0.25, 0.1, 0.05, 0.02):
        improved = True
        rounds = 0
        while improved and rounds < 100:
            improved, rounds = False, rounds + 1
            for ref in ctx.movable:
                if ref in ctx.impossible:
                    continue
                cur_s = evaluate_one(cur, ref, cur[ref], ctx)
                x, y, ori = cur[ref]
                cands = [(x + dx * step, y + dy * step, ori) for dx in (-1, 0, 1)
                         for dy in (-1, 0, 1) if dx or dy]
                cands += [(x, y, o) for o in ORIENTATIONS if o != ori]
                for c in cands:
                    if ctx.valid(ref, c):
                        s = evaluate_one(cur, ref, c, ctx)
                        if better(s, cur_s):
                            cur[ref], cur_s, improved = c, s, True
    return cur


def evolve(ctx, pop_size, generations, elite=2, verbose=True, mutation_rate=0.08):
    """Laço principal do AG. Retorna (melhor indivíduo, histórico por geração)."""
    n_greedy = max(1, pop_size // 10)
    current = {r: ctx.repair(r, ctx.parts[r].start_gene) for r in ctx.movable}
    population = [current] + [greedy_individual(ctx) for _ in range(n_greedy)]
    while len(population) < pop_size:
        population.append(random_individual(ctx))
    population = population[:pop_size]

    history = []
    for gen in range(generations + 1):
        scores = [evaluate(i, ctx) for i in population]
        order = sorted(range(len(population)), key=lambda i: scores[i])
        b = scores[order[0]]
        history.append({"geracao": gen, "sobreposicao": b[0], "custo": b[1],
                        "validos": sum(1 for s in scores if s[0] <= 0),
                        "custo_mediano": sorted(s[1] for s in scores)[len(scores) // 2]})
        if verbose and (gen % 25 == 0 or gen == generations):
            print(f"  geração {gen:3d} | sobreposição={b[0]:7.3f} | custo={b[1]:8.2f} "
                  f"| indivíduos sem sobreposição: {history[-1]['validos']}/{pop_size}")
        if gen == generations:
            return population[order[0]], history
        step = 3.0 * (1.0 - gen / generations) + 0.05        # passo diminui com o tempo
        nxt = [dict(population[i]) for i in order[:elite]]   # elitismo
        while len(nxt) < pop_size:
            c1, c2 = crossover_uniform(tournament(population, scores, ctx),
                                       tournament(population, scores, ctx), ctx)
            nxt.append(mutation(c1, mutation_rate, ctx, step))
            if len(nxt) < pop_size:
                nxt.append(mutation(c2, mutation_rate, ctx, step))
        population = nxt


# ----------------------------------------------------------------------------
# APLICAÇÃO, CONFERÊNCIA E RELATÓRIO
# ----------------------------------------------------------------------------
def set_footprint(part, gene):
    x, y = part.origin(gene)
    part.fp.SetOrientationDegrees(float(gene[2]))
    part.fp.SetPosition(pcbnew.VECTOR2I(int(pcbnew.FromMM(x)), int(pcbnew.FromMM(y))))


def check_board(ctx):
    """Confere a CONTENÇÃO lendo as posições reais gravadas nos footprints."""
    outside = []
    for ref, part in ctx.parts.items():
        if has_edge_cuts(part.fp):
            continue
        full, core = measure_rects(part.fp)
        use_core = ref in ctx.fit and ctx.fit[ref][0]
        if not ctx.outline.contains(core if use_core else full):
            outside.append(ref)
    return outside


def find_overlaps(ctx, genes):
    rects = dict(ctx.fixed)
    rects.update({r: ctx.parts[r].rect(g) for r, g in genes.items()})
    moved = set(genes) | set(ctx.genes_anchor)
    refs = sorted(rects)
    return [(a, b) for i, a in enumerate(refs) for b in refs[i + 1:]
            if (a in moved or b in moved) and collides(ctx, a, b)
            and overlap(rects[a], rects[b], ctx.rules.clearance_mm)]


def rule_distances(ctx, genes):
    """Distância (mm) de cada componente com regra até o pino-alvo mais próximo."""
    out = {}
    for ref, gene in genes.items():
        real = [c for c in ctx.targets[ref] if not c[4].startswith("ratsnest")]
        if real:
            p = ctx.parts[ref]
            out[ref] = min(math.hypot(p.pad_xy(gene, c[0], c[1])[0] - x,
                                      p.pad_xy(gene, c[0], c[1])[1] - y)
                           for c in real for x, y in c[2])
    return out


def run_ga_placement(board, pop_size=120, generations=300, seed=42, elite=2,
                     refine=True, apply=True, history_path=None, rules=None, verbose=True,
                     mutation_rate=0.08):
    """Posiciona os componentes. Retorna um dicionário com o resultado (ou None se não
    for possível nem ler a placa)."""
    def log(msg):
        if verbose:
            print(msg)

    if pop_size < 4 or generations < 0 or not 0 <= elite < pop_size:
        raise ValueError("Use pop_size >= 4, generations >= 0 e 0 <= elite < pop_size.")
    try:
        ctx = Context(board, rules or BOARD_RULES, random.Random(seed))
    except RuntimeError as e:
        print(f"[ERRO] {e}")
        return None

    b = ctx.outline.bounds
    log(f"[INFO] Placa: X({b[0]:.1f} a {b[2]:.1f} mm), Y({b[1]:.1f} a {b[3]:.1f} mm), "
        f"{len(ctx.outline.loops)} laço(s) de contorno | semente={seed}")
    log(f"[INFO] Fora da placa antes: {len(ctx.outside)} -> {', '.join(ctx.outside) or 'nenhum'}")

    place_anchors(ctx)
    ctx.build_connections()
    for w in ctx.warnings:
        log(f"[AVISO] {w}")
    if verbose:
        ctx.describe()

    history, best = [], {}
    if ctx.movable:
        before = rule_distances(ctx, {r: ctx.parts[r].start_gene for r in ctx.movable})
        log(f"[INFO] Iniciando AG ({pop_size} indivíduos, {generations} gerações)...")
        best, history = evolve(ctx, pop_size, generations, elite, verbose, mutation_rate)
        best = remove_overlaps(best, ctx)
        if refine:
            log("[INFO] Refinamento local...")
            best = local_refinement(best, ctx)
    else:
        before = {}
    over, cost = evaluate(best, ctx)

    if history_path and history:
        with open(history_path, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(history[0]))
            wr.writeheader()
            wr.writerows(history)

    all_genes = dict(ctx.genes_anchor)
    all_genes.update(best)
    outside_after = None
    if apply:
        for ref, gene in all_genes.items():
            set_footprint(ctx.parts[ref], gene)
        outside_after = check_board(ctx)
        try:
            pcbnew.Refresh()
        except Exception:
            pass

    pairs = find_overlaps(ctx, best)
    result = {"outside_before": ctx.outside, "outside_after": outside_after,
              "impossible": ctx.impossible, "overlaps": pairs, "overlap": over, "cost": cost,
              "genes": all_genes, "movable": ctx.movable, "history": history,
              "distances_before": before, "distances_after": rule_distances(ctx, best),
              "warnings": ctx.warnings}

    log("\n================ RELATÓRIO ================")
    for ref, g in sorted(all_genes.items()):
        x, y = ctx.parts[ref].origin(g)
        d = result["distances_after"].get(ref)
        extra = f" | distância ao alvo: {before[ref]:7.2f} -> {d:5.2f} mm" if d is not None else ""
        log(f"- {ref:5s} X={x:8.2f} Y={y:8.2f} rot={g[2]:3d}{extra}")
    if not apply:
        log("[SIMULAÇÃO] Nada foi gravado na placa.")
    elif outside_after:
        log(f"[FALHA DE CONTENÇÃO] Ainda fora da placa: {', '.join(outside_after)}")
        if ctx.impossible:
            log(f"   Não existe posição (0/90/180/270 graus) em que caibam: "
                f"{', '.join(ctx.impossible)}. Aumente o Edge.Cuts ou troque o footprint.")
    else:
        log("[OK] CONTENÇÃO: todos os footprints estão dentro da placa.")
    if pairs:
        log(f"[ATENÇÃO] SOBREPOSIÇÃO restante: {pairs}. Revise o tamanho da placa ou rode "
            f"com mais gerações/outra semente.")
    else:
        log("[OK] SOBREPOSIÇÃO: nenhum componente reposicionado invade outro.")
    log(f"[INFO] Custo de roteamento final = {cost:.2f}")
    try:
        if apply and len(board.GetTracks()) > 0:
            log("[AVISO] A placa já tem trilhas/vias. Apague-as e roteie de novo no Freerouting.")
    except Exception:
        pass
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description="Posicionamento de PCB com Algoritmo Genético (KiCad)")
    ap.add_argument("placa", help="arquivo .kicad_pcb de entrada")
    ap.add_argument("-o", "--saida", help="arquivo de saída (padrão: <placa>_posicionado.kicad_pcb)")
    ap.add_argument("--pop", type=int, default=120)
    ap.add_argument("--geracoes", type=int, default=300)
    ap.add_argument("--mutacao", type=float, default=0.08, help="chance de mutar cada componente")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--elite", type=int, default=2)
    ap.add_argument("--sem-refinamento", action="store_true", help="mostra o resultado só do AG")
    ap.add_argument("--simular", action="store_true", help="calcula, mas não grava a placa")
    ap.add_argument("--historico", help="grava a convergência por geração em CSV")
    a = ap.parse_args(argv)

    if not os.path.isfile(a.placa):
        print(f"[ERRO] Arquivo não encontrado: {a.placa}")
        return 1
    board = pcbnew.LoadBoard(a.placa)
    res = run_ga_placement(board, a.pop, a.geracoes, a.seed, a.elite,
                           refine=not a.sem_refinamento, apply=not a.simular,
                           history_path=a.historico, mutation_rate=a.mutacao)
    if res is None:
        return 1
    if not a.simular:
        out = a.saida or os.path.splitext(a.placa)[0] + "_posicionado.kicad_pcb"
        pcbnew.SaveBoard(out, board)
        print(f"[INFO] Placa salva em: {out}")
        if res["outside_after"]:
            return 2
    return 0


if __name__ == "__main__":
    if len(sys.argv) > 1:
        sys.exit(main())
    board = pcbnew.GetBoard()           # executado de dentro do KiCad
    if board is None:
        print("Uso: python script_IA.py placa.kicad_pcb  (veja --help)")
        sys.exit(1)
    run_ga_placement(board)
