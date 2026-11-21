# *****************************************************************************
# OTIMIZADOR DE POSICIONAMENTO DE PCB COM ALGORITMO GENÉTICO (KiCad / pcbnew)
# *****************************************************************************
#
# Visão Geral do Funcionamento:
#   Este script utiliza um Algoritmo Genético (AG) aliado a uma busca local
#   (Descida por Coordenadas) para posicionar automaticamente componentes SMD/THT
#   em uma placa de circuito impresso dentro do KiCad.
#
# Garantias Físicas e Elétricas:
#   * Zero sobreposição: nenhum componente móvel invade o corpo de outro (móvel
#     ou fixo), respeitando uma folga mínima (clearance_mm) para soldagem.
#   * Respeito aos limites: todos os componentes permanecem dentro do contorno
#     da placa (Edge.Cuts) com margem de segurança (edge_margin_mm).
#   * Atração baseada em pinos reais: as distâncias são calculadas diretamente
#     entre os pads conectados pelas NETS da PCB (ex.: Y1 aos pinos do cristal
#     do U1), usando FALLBACK_MAP apenas se a net não existir.
#   * Limpeza de serigrafia: oculta valores poluídos e organiza designadores
#     (Reference) na camada F.Silkscreen sem colisões.
# ****************************************************************************

import math
import random
from dataclasses import dataclass, field

import pcbnew



# ============================================================================
# REGRAS DA PLACA E PARÂMETROS DE ENGENHARIA (BOARD_RULES)
# ============================================================================
# Define quais componentes são fixos (âncoras), quais podem ser movidos pelo AG,
# as restrições mecânicas de fabricação e os pesos elétricos de cada circuito.

@dataclass
class BoardRules:
    movable_refs: list                  # componentes que o AG pode mover
    anchor_refs: list                  # Componentes fixos (CIs, conectores grandes, chaves) usados como referência
    fallback_map: dict                  # {ref: [(CI, pino), ...]} usado se não houver net
    ignore_net_keywords: tuple = ("GND",)   # Redes ignoradas no cálculo de distância (plano GND cobre toda a placa)
    clearance_mm: float = 0.25          # Folga mínima (mm) entre componentes
    edge_margin_mm: float = 1.20        # folga mínima até a borda da placa
    default_weight: float = 10.0        # Peso padrão de atração quando não especificado em 'weights'
    weights: dict = field(default_factory=dict)  # Pesos individuais (quanto maior, mais curto o caminho)
    init_radius_mm: float = 5.0         # raio da população inicial ao redor do pino-alvo
    # Restringe a quais âncoras cada componente é atraído (só nets compartilhadas com elas contam).
    # Quem não estiver aqui é atraído por todas as âncoras.
    attract_to: dict = field(default_factory=dict)
    # Peso da atração pela BORDA da placa (distância do corpo à borda mais próxima).
    edge_weights: dict = field(default_factory=dict)
    # Nets cujos componentes móveis devem ficar agrupados entre si (ex.: casamento de antena).
    group_nets: tuple = ()
    group_weight: float = 12.0


BOARD_RULES = BoardRules(
    movable_refs=[
        "Y1", "Y2", "F1",           
        "C1", "C2", "C3", "C4",   
        "R1", "R2", "R3", "R4",
        "L1", "L2", "L3", "J1"
    ],
    anchor_refs=["U1", "U2", "BT2", "SW1", "SW2"],
    group_nets=(
        "/ANT_SIG",             # Linha de sinal de RF entre U1, C4 e R4
        "Net-(J1-In)",          # Saída da rede de casamento (R4, L3) para o conector coaxial J1
        "/ACC_CLK_EXT",         # Clock externo entre oscilador Y2, R1 e acelerômetro U2
        "Net-(L1-Pad1)",        # Filtro de alimentação entre R2 e L1
        "Net-(L2-Pad1)",        # Filtro de alimentação entre R3 e L2
    ),

    clearance_mm=0.35,      # 0.35 mm garante espaço seguro para soldagem manual/reflow sem curto    
    edge_margin_mm=1.50,       # 1.50 mm protege componentes contra fresagem e anel de GND na borda
    group_weight=28.0,          # Alta coesão para manter redes de RF e filtros compactos
    edge_weights={"J1": 4.0},   # Leve preferência para manter o conector coaxial J1 perto da borda


    # Direcionamento de atração: evita que um capacitor de um CI seja puxado por outro CI
    attract_to={
        # --- U1 (Microcontrolador / RF) ---
        "C1": ["U1"], "C2": ["U1"], "C3": ["U1"],
        "C4": ["U1"], "R4": ["U1"], "L3": ["U1"],
        "L1": ["U1"], "L2": ["U1"],

        # --- U2 (Acelerômetro) ---
        "R1": ["U2"], "Y2": ["U2"],

        # --- BT2 (Entrada de Bateria e Proteção) ---
        "F1": ["BT2"],
        "R2": ["BT2"], "R3": ["BT2"],
    },


    fallback_map={
        "F1": [("BT2", "1")],       # Garante fusível F1 colado ao polo positivo (+) da bateria
        "Y2": [("U2", "13")],       #Cristal/oscilador Y2 conectado ao pino de clock do U2
        "L1": [("U1", "11"), ("U1", "12")],
        "L2": [("U1", "11"), ("U1", "12")],
    },


    weights={
        # Cristal do microcontrolador
        "Y1": 35.0,

        # Circuito de RF e antena (pesos altos para encurtar trilhas de 2.4 GHz)
        "R4": 30.0,
        "C4": 30.0,
        "L3": 28.0,
        "J1": 22.0,     

        # Desacoplamento
        "C1": 14.0,
        "C2": 20.0,
        "C3": 20.0,

        # Oscilador externo e acelerômetro
        "R1": 15.0,
        "Y2": 14.0,

        # Alimentação e filtros
        "F1": 45.0,
        "L1": 12.0,
        "L2": 12.0,
        "R2": 10.0,
        "R3": 10.0,
    },
)


# ----------------------------------------------------------------------------
# GEOMETRIA, TRANSFORMAÇÕES E DETECÇÃO DE COLISÕES
# ----------------------------------------------------------------------------
# Funções matemáticas para conversão de unidades do KiCad (nanômetros -> mm),
# rotação de coordenadas de pads/caixas e verificação de intersecções e bordas.

def mm(v):
    return pcbnew.ToMM(v)


def rot(x, y, deg):
    """Rotação no mesmo sentido do KiCad (Y para baixo, ângulo positivo = anti-horário)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (x * c + y * s, -x * s + y * c)


def get_rect(fp):
    """Retângulo (x0, y0, x1, y1) em mm do footprint SEM textos de referência/valor."""
    try:
        r = fp.GetFootprintRect()
    except Exception:
        r = fp.GetBoundingBox(False)
    x0, y0 = mm(r.GetX()), mm(r.GetY())
    return (x0, y0, x0 + mm(r.GetWidth()), y0 + mm(r.GetHeight()))


def build_model(fp):
    """Modelo do footprint em orientação 0°: offset do centro da caixa, tamanho e pads locais."""
    x0, y0, x1, y1 = get_rect(fp)
    pos = fp.GetPosition()
    px, py = mm(pos.x), mm(pos.y)
    ang = fp.GetOrientationDegrees()
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    off = rot(cx - px, cy - py, -ang)
    w, h = x1 - x0, y1 - y0
    if int(round(ang)) % 180 == 90:
        w, h = h, w
    pads = []
    for pad in fp.Pads():
        p = pad.GetPosition()
        lx, ly = rot(mm(p.x) - px, mm(p.y) - py, -ang)
        pads.append((str(pad.GetNumber()), pad.GetNetname(), lx, ly))
    return {"off": off, "size": (w, h), "pads": pads}


def rect_of(model, x, y, ori):
    ox, oy = rot(model["off"][0], model["off"][1], ori)
    cx, cy = x + ox, y + oy
    w, h = model["size"]
    if int(round(ori)) % 180 == 90:
        w, h = h, w
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


def overlap_violation(r1, r2, clearance):
    dx = min(r1[2], r2[2]) - max(r1[0], r2[0]) + clearance
    dy = min(r1[3], r2[3]) - max(r1[1], r2[1]) + clearance
    if dx > 0 and dy > 0:
        return 0.5 + min(dx, dy)   # base fixa + profundidade (dá gradiente ao AG)
    return 0.0


def point_in_polygon(px, py, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py):
            xint = (xj - xi) * (py - yi) / (yj - yi + 1e-18) + xi
            if px < xint:
                inside = not inside
        j = i
    return inside


def get_outline_polygon(board):
    """Contorno real da placa (Edge.Cuts) como lista de pontos; None se indisponível."""
    try:
        poly = pcbnew.SHAPE_POLY_SET()
        board.GetBoardPolygonOutlines(poly)
        if poly.OutlineCount() == 0:
            return None
        o = poly.COutline(0)
        pts = [(mm(o.CPoint(i).x), mm(o.CPoint(i).y)) for i in range(o.PointCount())]
        return pts if len(pts) >= 3 else None
    except Exception:
        return None


def get_board_bounds(board):
    try:
        bb = board.GetBoardEdgesBoundingBox()
        if bb.GetWidth() <= 0 or bb.GetHeight() <= 0:
            raise ValueError
    except Exception:
        bb = board.ComputeBoundingBox(False)
    x0, y0 = mm(bb.GetX()), mm(bb.GetY())
    return (x0, y0, x0 + mm(bb.GetWidth()), y0 + mm(bb.GetHeight()))


def boundary_violation(rect, bounds, outline, margin):
    """0 se o retângulo (com margem) está totalmente dentro da placa; senão, quanto saiu."""
    bx0, by0, bx1, by1 = bounds
    x0, y0, x1, y1 = rect
    v = (max(0.0, bx0 + margin - x0) + max(0.0, x1 - (bx1 - margin)) +
         max(0.0, by0 + margin - y0) + max(0.0, y1 - (by1 - margin)))
    if outline:
        xs = [x0 - margin, (x0 + x1) / 2.0, x1 + margin]
        ys = [y0 - margin, (y0 + y1) / 2.0, y1 + margin]
        for px in xs:
            for py in ys:
                if not point_in_polygon(px, py, outline):
                    v += 1.0
    return v


# ----------------------------------------------------------------------------
# CONTEXTO (dados pré-calculados da placa)
# ----------------------------------------------------------------------------
class Context:
    def __init__(self, board, rules):
        self.rules = rules
        self.fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

        missing = [r for r in rules.movable_refs + rules.anchor_refs if r not in self.fps]
        if missing:
            raise RuntimeError(f"Componentes não encontrados na PCB: {missing}")

        self.bounds = get_board_bounds(board)
        self.outline = get_outline_polygon(board)

        # Pinos dos CIs-âncora, por net e por número
        self.anchor_by_net = {}
        self.anchor_by_pin = {}
        for aref in rules.anchor_refs:
            for pad in self.fps[aref].Pads():
                p = pad.GetPosition()
                pt = (mm(p.x), mm(p.y))
                self.anchor_by_pin.setdefault((aref, str(pad.GetNumber())), []).append(pt)
                net = pad.GetNetname()
                if self._net_ok(net):
                    self.anchor_by_net.setdefault((aref, net), []).append(pt)

        # Modelos e conexões dos móveis
        self.models = {}
        self.conns = {}
        self.movable = []
        for ref in rules.movable_refs:
            m = build_model(self.fps[ref])
            conns = []
            allowed = rules.attract_to.get(ref, rules.anchor_refs)
            for num, net, lx, ly in m["pads"]:
                if not self._net_ok(net):
                    continue
                for aref in allowed:
                    pts = self.anchor_by_net.get((aref, net))
                    if pts:
                        conns.append((lx, ly, pts, f"{aref} via net {net}"))
            if not conns:
                for (aref, pin) in rules.fallback_map.get(ref, []):
                    pts = self.anchor_by_pin.get((aref, pin))
                    if pts:
                        conns.append((0.0, 0.0, pts, f"{aref}.{pin} (mapa manual)"))
            in_group = any(self._is_group_net(n) for _, n, _, _ in m["pads"])
            if conns or in_group or rules.edge_weights.get(ref, 0) > 0:
                self.models[ref] = m
                self.conns[ref] = conns
                self.movable.append(ref)
            else:
                print(f"[AVISO] {ref}: sem conexão com {rules.anchor_refs} (nem por net, nem pelo mapa). "
                      f"Será mantido fixo na posição atual.")

        # Pads dos móveis em nets de agrupamento (ex.: ANT_SIG): {net: [(ref, lx, ly), ...]}
        self.group_pads = {}
        for ref in self.movable:
            for _, net, lx, ly in self.models[ref]["pads"]:
                if self._is_group_net(net):
                    self.group_pads.setdefault(net, []).append((ref, lx, ly))

        # Segmentos da borda da placa (para a atração pela borda)
        if self.outline:
            n = len(self.outline)
            self.edge_segs = [(self.outline[i], self.outline[(i + 1) % n]) for i in range(n)]
        else:
            bx0, by0, bx1, by1 = self.bounds
            c = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
            self.edge_segs = [(c[i], c[(i + 1) % 4]) for i in range(4)]

        # Obstáculos fixos (tudo que não for móvel)
        self.fixed_rects = {ref: get_rect(fp) for ref, fp in self.fps.items()
                            if ref not in self.movable}

        # Região de busca: ao redor dos alvos; placa inteira se algum componente é atraído pela borda
        bx0, by0, bx1, by1 = self.bounds
        pts = [p for ref in self.movable for c in self.conns[ref] for p in c[2]]
        if pts and not rules.edge_weights:
            r = rules.init_radius_mm * 2.5
            self.search = (max(bx0, min(p[0] for p in pts) - r), max(by0, min(p[1] for p in pts) - r),
                           min(bx1, max(p[0] for p in pts) + r), min(by1, max(p[1] for p in pts) + r))
        else:
            self.search = (bx0, by0, bx1, by1)

    def _is_group_net(self, net):
        if not net:
            return False

        return net.upper() in {
            k.upper() for k in self.rules.group_nets
        }

    def _net_ok(self, net):
        if not net:
            return False
        up = net.upper()
        return not any(k in up for k in self.rules.ignore_net_keywords)

    def describe(self):
        for ref in self.movable:
            alvos = sorted({c[3] for c in self.conns[ref]})
            extra = []
            if self.rules.edge_weights.get(ref, 0) > 0:
                extra.append("borda da placa")
            if any(r == ref for pads in self.group_pads.values() for r, _, _ in pads):
                extra.append("grupo " + "/".join(self.rules.group_nets))
            print(f"[INFO] {ref} -> {', '.join(alvos + extra) or '(nenhum alvo)'}")
        for net, pads in self.group_pads.items():
            print(f"[INFO] Net de grupo '{net}': {sorted({r for r, _, _ in pads})}")


def dist_to_edge(rect, segs):
    """Menor distância entre o corpo (cantos + pontos médios dos lados) e a borda da placa."""
    x0, y0, x1, y1 = rect
    xm, ym = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    samples = [(x0, y0), (xm, y0), (x1, y0), (x0, ym), (x1, ym), (x0, y1), (xm, y1), (x1, y1)]
    best = float("inf")
    for px, py in samples:
        for (ax, ay), (bx, by) in segs:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
            best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


# ----------------------------------------------------------------------------
# FITNESS
# ----------------------------------------------------------------------------
def violation_and_cost(ind, ctx):
    rules = ctx.rules
    rects = {}
    violation = 0.0
    cost = 0.0

    for ref, (x, y, ori) in ind.items():
        rect = rect_of(ctx.models[ref], x, y, ori)
        rects[ref] = rect
        violation += boundary_violation(rect, ctx.bounds, ctx.outline, rules.edge_margin_mm)

        w = rules.weights.get(ref, rules.default_weight)
        for lx, ly, pts, _ in ctx.conns[ref]:
            dx, dy = rot(lx, ly, ori)
            px, py = x + dx, y + dy
            cost += w * min(math.hypot(px - ax, py - ay) for ax, ay in pts)

        we = rules.edge_weights.get(ref, 0.0)
        if we > 0:
            cost += we * dist_to_edge(rect, ctx.edge_segs)

    # Agrupamento por net (ex.: ANT_SIG): pads de componentes diferentes na mesma net ficam juntos
    for pads in ctx.group_pads.values():
        pos = []
        for ref, lx, ly in pads:
            if ref not in ind:      # indivíduo parcial (durante a geração inicial)
                continue
            x, y, ori = ind[ref]
            dx, dy = rot(lx, ly, ori)
            pos.append((ref, x + dx, y + dy))
        for i in range(len(pos)):
            for j in range(i + 1, len(pos)):
                if pos[i][0] != pos[j][0]:
                    cost += rules.group_weight * math.hypot(pos[i][1] - pos[j][1], pos[i][2] - pos[j][2])

    refs = list(rects.keys())
    for i, r1 in enumerate(refs):
        for r2 in refs[i + 1:]:
            violation += overlap_violation(rects[r1], rects[r2], rules.clearance_mm)
        for fr in ctx.fixed_rects.values():
            violation += overlap_violation(rects[r1], fr, rules.clearance_mm)
    return violation, cost


def evaluate_fitness(ind, ctx):
    v, c = violation_and_cost(ind, ctx)
    return -c - 1e4 * v


# ----------------------------------------------------------------------------
# OPERADORES GENÉTICOS
# ----------------------------------------------------------------------------
ORIENTATIONS = [0.0, 90.0, 180.0, 270.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def create_individual(ctx, tries=80):
    """Posicionamento guloso: para cada componente, melhor de N candidatos já respeitando os anteriores."""
    ind = {}
    r = ctx.rules.init_radius_mm
    sx0, sy0, sx1, sy1 = ctx.search
    for ref in ctx.movable:
        best, best_f = None, -float("inf")
        edge_w = ctx.rules.edge_weights.get(ref, 0.0) > 0
        for _ in range(tries):
            if (edge_w and random.random() < 0.5) or not ctx.conns[ref]:
                # sorteia um ponto sobre a borda e recua um pouco para dentro da placa
                (ax1, ay1), (ax2, ay2) = random.choice(ctx.edge_segs)
                t = random.random()
                ax, ay = ax1 + t * (ax2 - ax1), ay1 + t * (ay2 - ay1)
                x = clamp(ax + random.uniform(-3.0, 3.0), sx0, sx1)
                y = clamp(ay + random.uniform(-3.0, 3.0), sy0, sy1)
            else:
                _, _, pts, _ = random.choice(ctx.conns[ref])
                ax, ay = random.choice(pts)
                x = clamp(ax + random.uniform(-r, r), sx0, sx1)
                y = clamp(ay + random.uniform(-r, r), sy0, sy1)
            cand = dict(ind)
            cand[ref] = (x, y, random.choice(ORIENTATIONS))
            f = evaluate_fitness(cand, ctx)
            if f > best_f:
                best_f, best = f, cand[ref]
        ind[ref] = best
    return ind


def crossover_uniform(p1, p2):
    c1, c2 = {}, {}
    for ref in p1:
        if random.random() < 0.5:
            c1[ref], c2[ref] = p1[ref], p2[ref]
        else:
            c1[ref], c2[ref] = p2[ref], p1[ref]
    return c1, c2


def mutation(ind, rate, ctx, step):
    sx0, sy0, sx1, sy1 = ctx.search
    out = dict(ind)
    for ref, (x, y, ori) in ind.items():
        if random.random() < rate:
            s = step * (8.0 if random.random() < 0.1 else 1.0)   # saltos ocasionais maiores
            x = clamp(x + random.gauss(0, s), sx0, sx1)
            y = clamp(y + random.gauss(0, s), sy0, sy1)
            if random.random() < 0.15:
                ori = random.choice(ORIENTATIONS)
            out[ref] = (x, y, ori)
    return out


def local_refinement(ind, ctx):
    """Descida por coordenadas em passos finos; só aceita melhorias."""
    cur = dict(ind)
    cur_f = evaluate_fitness(cur, ctx)
    for step in (0.25, 0.1, 0.05, 0.02, 0.01):
        improved = True
        while improved:
            improved = False
            for ref in ctx.movable:
                x, y, ori = cur[ref]
                cands = [(x + dx, y + dy, ori) for dx, dy in
                         ((step, 0), (-step, 0), (0, step), (0, -step),
                          (step, step), (-step, -step), (step, -step), (-step, step))]
                cands += [(x, y, o) for o in ORIENTATIONS if o != ori]
                for c in cands:
                    test = dict(cur)
                    test[ref] = c
                    f = evaluate_fitness(test, ctx)
                    if f > cur_f + 1e-9:
                        cur, cur_f, improved = test, f, True
    return cur


# ----------------------------------------------------------------------------
# EXECUÇÃO
# ----------------------------------------------------------------------------
def clean_old_copper(board):
    """Remove trilhas, vias e formas gráficas soltas em F.Cu (mantendo o anel de GND de 1.0 mm)."""
    to_remove = []

    # Remove trilhas estreitas (< 0.95 mm) e todas as vias
    for track in board.GetTracks():
        if isinstance(track, pcbnew.PCB_VIA):
            to_remove.append(track)
        elif isinstance(track, pcbnew.PCB_TRACK):
            if round(pcbnew.ToMM(track.GetWidth()), 2) < 0.95:
                to_remove.append(track)

    # Remove linhas/formas gráficas desenhadas na camada F.Cu
    for drawing in board.GetDrawings():
        if drawing.GetLayer() == pcbnew.F_Cu:
            to_remove.append(drawing)

    for item in to_remove:
        board.Remove(item)

def auto_clean_silkscreen(board, ctx):
    """Oculta valores (Value) poluídos e posiciona os designadores (Reference) alinhados ao componente sem colisão."""
    # Pega retângulos finais de TODOS os footprints (com margem extra)
    occupied = []
    for ref, fp in ctx.fps.items():
        x0, y0, x1, y1 = get_rect(fp)
        margin = 0.85 if (x1 - x0) > 5.0 else 0.25
        occupied.append((ref, x0 - margin, y0 - margin, x1 + margin, y1 + margin))

    placed_labels = []

    for ref, fp in ctx.fps.items():
        # Oculta o texto de Valor (ex.: "L_Small", "0.1uF", "SW_Nidec...")
        val_field = fp.Value()
        val_field.SetVisible(False)

        # Configura o Designador (Reference: R1, C4, U1...)
        ref_field = fp.Reference()
        ref_field.SetLayer(pcbnew.F_SilkS)
        ref_field.SetVisible(True)
        ref_field.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.75), pcbnew.FromMM(0.75)))
        ref_field.SetTextThickness(pcbnew.FromMM(0.14))

        # Alinha o ângulo do texto com a orientação do componente (0° horizontal ou 90° vertical)
        fp_ori = int(round(fp.GetOrientationDegrees())) % 180
        text_angle = 90.0 if fp_ori == 90 else 0.0
        ref_field.SetTextAngleDegrees(text_angle)

        x0, y0, x1, y1 = get_rect(fp)
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        w, h = x1 - x0, y1 - y0

        # Ajusta largura e altura da caixa do texto conforme o ângulo (0° ou 90°)
        base_w, base_h = len(ref) * 0.60, 0.90
        lw, lh = (base_h, base_w) if text_angle == 90.0 else (base_w, base_h)

        # Se for CI grande (U1, BT2), centraliza o texto no meio do corpo do componente
        if w > 5.0 and h > 5.0:
            ref_field.SetTextAngleDegrees(0.0)
            best_pos = (cx, cy)
            lw, lh = base_w, base_h
        else:
            # Testa 4 posições ao redor do componente: cima, baixo, direita, esquerda
            offset_y = h / 2.0 + lh / 2.0 + 0.35
            offset_x = w / 2.0 + lw / 2.0 + 0.35
            candidates = [
                (cx, cy - offset_y),  # cima
                (cx, cy + offset_y),  # baixo
                (cx + offset_x, cy),  # direita
                (cx - offset_x, cy),  # esquerda
            ]
            best_pos = candidates[0]
            best_score = float("inf")

            for px, py in candidates:
                lrect = (px - lw / 2.0, py - lh / 2.0, px + lw / 2.0, py + lh / 2.0)
                score = 0.0
                # Penaliza sair da placa
                score += 60.0 * boundary_violation(lrect, ctx.bounds, ctx.outline, 0.5)
                # Penaliza colidir com outro componente
                for oref, ox0, oy0, ox1, oy1 in occupied:
                    if oref != ref:
                        score += 25.0 * overlap_violation(lrect, (ox0, oy0, ox1, oy1), 0.15)
                # Penaliza colidir com outro texto já posicionado
                for prect in placed_labels:
                    score += 40.0 * overlap_violation(lrect, prect, 0.20)

                if score < best_score:
                    best_score = score
                    best_pos = (px, py)

        bx, by = best_pos
        ref_field.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(bx), pcbnew.FromMM(by)))
        placed_labels.append((bx - lw / 2.0, by - lh / 2.0, bx + lw / 2.0, by + lh / 2.0))



def run_ga_placement(board, pop_size=160, generations=250, seed=None, elite=2):
    clean_old_copper(board)

    if seed is not None:
        random.seed(seed)

    try:
        ctx = Context(board, BOARD_RULES)
    except RuntimeError as e:
        print(f"[ERRO] {e}")
        return None

    b = ctx.bounds
    print(f"[INFO] Placa: X({b[0]:.1f} a {b[2]:.1f}mm), Y({b[1]:.1f} a {b[3]:.1f}mm) "
          f"| contorno poligonal: {'sim' if ctx.outline else 'não (usando caixa)'}")
    ctx.describe()

    # Layout atual entra na população (se for válido, o AG nunca fica pior que ele)
    current = {}
    for ref in ctx.movable:
        fp = ctx.fps[ref]
        p = fp.GetPosition()
        current[ref] = (mm(p.x), mm(p.y), float(round(fp.GetOrientationDegrees() / 90.0) * 90 % 360))

    print(f"[INFO] Iniciando AG ({pop_size} indivíduos, {generations} gerações)...")
    population = [create_individual(ctx) for _ in range(pop_size - 1)] + [current]

    best, best_f = None, -float("inf")
    for gen in range(generations):
        fits = [evaluate_fitness(ind, ctx) for ind in population]
        order = sorted(range(len(population)), key=lambda i: fits[i], reverse=True)
        if fits[order[0]] > best_f:
            best_f, best = fits[order[0]], population[order[0]]

        if gen % 50 == 0 or gen == generations - 1:
            v, c = violation_and_cost(best, ctx)
            print(f"  geração {gen:3d} | custo={c:8.2f} | violações={v:.3f}")

        pairs = list(zip(population, fits))
        selected = [max(random.sample(pairs, 3), key=lambda t: t[1])[0] for _ in range(pop_size)]

        step = 0.6 * (1.0 - gen / generations) + 0.05   # passo decai ao longo das gerações
        nxt = [dict(population[i]) for i in order[:elite]]
        i = 0
        while len(nxt) < pop_size:
            p1, p2 = selected[i % pop_size], selected[(i + 1) % pop_size]
            c1, c2 = crossover_uniform(p1, p2)
            nxt.append(mutation(c1, 0.3, ctx, step))
            if len(nxt) < pop_size:
                nxt.append(mutation(c2, 0.3, ctx, step))
            i += 2
        population = nxt

    print("[INFO] Refinamento local...")
    best = local_refinement(best, ctx)

    v, c = violation_and_cost(best, ctx)
    if v > 1e-9:
        print(f"\n[AVISO] Violação residual ({v:.3f}). Aplicando melhor solução encontrada mesmo assim...")

    for ref in ctx.movable:
        x, y, ori = best[ref]
        fp = ctx.fps[ref]
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(ori)

    auto_clean_silkscreen(board, ctx)
    pcbnew.Refresh()
    if board.GetFileName():
        board.Save(board.GetFileName())

    print(f"\n[SUCESSO] Posicionamento e serigrafia aplicados. Custo final = {c:.2f} | Violações = {v:.3f}")
    for ref in ctx.movable:
        x, y, ori = best[ref]
        print(f"- {ref}: X={x:.2f}mm Y={y:.2f}mm rot={ori:.0f}°")
    print("[DICA] Rode o DRC (Inspecionar > Verificador de Regras de Projeto) para confirmar.")
    return best


if __name__ == "__main__":
    run_ga_placement(pcbnew.GetBoard())