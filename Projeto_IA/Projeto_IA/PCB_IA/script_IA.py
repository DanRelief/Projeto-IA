import math
import random
from dataclasses import dataclass
import pcbnew

@dataclass
class BoardRules:
    crystal_ref: str        # ex: "Y1"
    crystal_target: str     # ex: "U1" (chip ao qual o cristal está associado)
    decoupling_map: dict    # ex: {"C1": "U1", "C2": "U2"} -> Capacitor C1 perto de U1, C2 perto de U2, etc.
    movable_refs: list      # Lista de todos os componentes que a IA vai mover

# Definição modular e escalável: adicione quantos quiser aqui no futuro!
BOARD_RULES = BoardRules(
    crystal_ref="Y1",
    crystal_target="U1",
    decoupling_map={
        "C1": "U1",  # C1 deve ficar próximo do microcontrolador U1
        "C2": "U2"   # C2 deve ficar próximo do acelerômetro U2 (ou outro chip)
    },
    movable_refs=["Y1", "C1", "C2"]
)

def get_approx_dimensions(fp):
    """Obtém a largura e altura reais do footprint usando o BoundingBox do KiCad."""
    bbox = fp.GetBoundingBox()
    width = pcbnew.ToMM(bbox.GetWidth())
    height = pcbnew.ToMM(bbox.GetHeight())
    return width, height

def check_box_overlap(pos1, size1, pos2, size2, clearance_mm=0.2):
    """Verifica sobreposição entre duas caixas delimitadoras com margem de segurança."""
    x1, y1 = pos1
    w1, h1 = size1
    x2, y2 = pos2
    w2, h2 = size2

    left1, right1 = x1 - (w1 / 2.0) - clearance_mm, x1 + (w1 / 2.0) + clearance_mm
    top1, bottom1 = y1 - (h1 / 2.0) - clearance_mm, y1 + (h1 / 2.0) + clearance_mm

    left2, right2 = x2 - (w2 / 2.0) - clearance_mm, x2 + (w2 / 2.0) + clearance_mm
    top2, bottom2 = y2 - (h2 / 2.0) - clearance_mm, y2 + (h2 / 2.0) + clearance_mm

    intersect_x = (left1 < right2) and (right1 > left2)
    intersect_y = (top1 < bottom2) and (bottom1 > top2)

    if intersect_x and intersect_y:
        overlap_w = min(right1, right2) - max(left1, left2)
        overlap_h = min(bottom1, bottom2) - max(top1, top2)
        return overlap_w * overlap_h
    
    return 0.0

def evaluate_fitness(individual, all_fps_pos, all_fps_sizes, board_bounds, rules):
    total_fitness = 0.0
    
    current_positions = {}
    current_sizes = {}
    
    xmin_board, xmax_board, ymin_board, ymax_board = board_bounds
    
    # Atualiza as posições dos componentes móveis propostas pelo indivíduo atual
    for ref in rules.movable_refs:
        if ref in individual:
            x, y, _ = individual[ref]
            current_positions[ref] = (x, y)
            w, h = all_fps_sizes[ref]
            current_sizes[ref] = (w, h)

            # 0. Restrição de Borda: Se o componente móvel sair da placa, penalidade severa
            half_w, half_h = w / 2.0, h / 2.0
            if (x - half_w < xmin_board) or (x + half_w > xmax_board) or \
               (y - half_h < ymin_board) or (y + half_h > ymax_board):
                total_fitness -= 50000.0

    # Combina posições e tamanhos de todos os componentes (fixos + móveis correntes)
    combined_positions = dict(all_fps_pos)
    for ref, pos in current_positions.items():
        combined_positions[ref] = pos

    combined_sizes = dict(all_fps_sizes)
    for ref, size in current_sizes.items():
        combined_sizes[ref] = size

    # 1. Colisão entre todos os componentes da placa (móveis entre si e contra fixos)
    refs_list = list(combined_positions.keys())
    for i in range(len(refs_list)):
        for j in range(i + 1, len(refs_list)):
            r1, r2 = refs_list[i], refs_list[j]
            overlap = check_box_overlap(combined_positions[r1], combined_sizes[r1], 
                                         combined_positions[r2], combined_sizes[r2], clearance_mm=0.3)
            if overlap > 0.0:
                # Penalidade pesada por colisão
                total_fitness -= 30000.0 * (overlap + 1.0)

    # 2. Avaliação do Cristal (deve ficar próximo ao seu chip alvo, respeitando zona de segurança contra ruído)
    if rules.crystal_ref in individual and rules.crystal_target in all_fps_pos:
        crystal_x, crystal_y, _ = individual[rules.crystal_ref]
        target_pos = all_fps_pos[rules.crystal_target]
        dist_crystal = math.hypot(target_pos[0] - crystal_x, target_pos[1] - crystal_y)
        MIN_SAFE_DISTANCE_MM = 7.5  
        
        if dist_crystal < MIN_SAFE_DISTANCE_MM:
            total_fitness += -1000.0 * (MIN_SAFE_DISTANCE_MM - dist_crystal) - 500.0
        else:
            total_fitness += -dist_crystal * 1.2

    # 3. Avaliação Escalável dos Capacitores de Desacoplamento (cada um atrelado ao seu respectivo chip alvo)
    for cap_ref, target_ref in rules.decoupling_map.items():
        if cap_ref in individual and target_ref in all_fps_pos:
            cap_x, cap_y, _ = individual[cap_ref]
            target_pos = all_fps_pos[target_ref]
            dist_cap = math.hypot(target_pos[0] - cap_x, target_pos[1] - cap_y)
            MAX_DECOUP_DIST_MM = 3.5  # Boa prática: bem próximo ao pino de alimentação do chip correspondente
            
            if dist_cap > MAX_DECOUP_DIST_MM:
                total_fitness += -800.0 * (dist_cap - MAX_DECOUP_DIST_MM) - 300.0
            else:
                total_fitness += -dist_cap * 0.5  # Recompensa por estar colado ao chip alvo

    return total_fitness

def create_individual(rules, search_bounds):
    min_x, max_x, min_y, max_y = search_bounds
    chromosome = {}
    for ref in rules.movable_refs:
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        orientation = random.choice([0.0, 90.0, 180.0, 270.0])
        chromosome[ref] = (x, y, orientation)
    return chromosome

def crossover_uniform(p1, p2):
    c1, c2 = {}, {}
    for ref in p1.keys():
        if random.random() < 0.5:
            c1[ref], c2[ref] = p1[ref], p2[ref]
        else:
            c1[ref], c2[ref] = p2[ref], p1[ref]
    return c1, c2

def mutation(individual, mutation_rate, search_bounds, step_mm=1.0):
    min_x, max_x, min_y, max_y = search_bounds
    mutated = dict(individual)
    for ref in mutated.keys():
        if random.random() < mutation_rate:
            x, y, ori = mutated[ref]
            nx = min(max(x + random.uniform(-step_mm, step_mm), min_x), max_x)
            ny = min(max(y + random.uniform(-step_mm, step_mm), min_y), max_y)
            if random.random() < 0.2:
                ori = random.choice([0.0, 90.0, 180.0, 270.0])
            mutated[ref] = (nx, ny, ori)
    return mutated

def run_ga_placement(board, pop_size=70, generations=100):
    rules = BOARD_RULES
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    # Valida se todos os componentes configurados nas regras existem na PCB
    required_refs = [rules.crystal_ref, rules.crystal_target] + list(rules.decoupling_map.keys()) + list(rules.decoupling_map.values())
    missing = [ref for ref in set(required_refs) if ref not in fps]
    if missing:
        print(f"[ERRO] Componentes obrigatórios não encontrados na PCB: {missing}")
        return

    # Mapeia posições e tamanhos de TODOS os footprints da placa
    all_fps_pos = {ref: (pcbnew.ToMM(fp.GetPosition().x), pcbnew.ToMM(fp.GetPosition().y)) for ref, fp in fps.items()}
    all_fps_sizes = {ref: get_approx_dimensions(fp) for ref, fp in fps.items()}

    # Limites físicos da placa
    board_bbox = board.ComputeBoundingBox(True)
    board_bounds = (
        pcbnew.ToMM(board_bbox.GetX()),
        pcbnew.ToMM(board_bbox.GetRight()),
        pcbnew.ToMM(board_bbox.GetY()),
        pcbnew.ToMM(board_bbox.GetBottom())
    )
    print(f"[INFO] Limites da placa detectados: X({board_bounds[0]:.1f} a {board_bounds[1]:.1f}mm), Y({board_bounds[2]:.1f} a {board_bounds[3]:.1f}mm)")

    # Define a área de busca baseada na posição do chip principal (crystal_target) ou centro geral
    target_pos = all_fps_pos[rules.crystal_target]
    search_bounds = (
        max(board_bounds[0] + 2.0, target_pos[0] - 20.0),
        min(board_bounds[1] - 2.0, target_pos[0] + 20.0),
        max(board_bounds[2] + 2.0, target_pos[1] - 20.0),
        min(board_bounds[3] - 2.0, target_pos[1] + 20.0)
    )

    population = [create_individual(rules, search_bounds) for _ in range(pop_size)]
    best_individual = None
    best_fitness = -float('inf')

    for gen in range(generations):
        fitnesses = [evaluate_fitness(ind, all_fps_pos, all_fps_sizes, board_bounds, rules) for ind in population]

        for ind, fit in zip(population, fitnesses):
            if fit > best_fitness:
                best_fitness = fit
                best_individual = ind

        selected = []
        for _ in range(pop_size):
            tournament = random.sample(list(zip(population, fitnesses)), k=3)
            winner = max(tournament, key=lambda x: x[1])[0]
            selected.append(winner)

        next_population = []
        for i in range(0, pop_size, 2):
            p1 = selected[i]
            p2 = selected[i + 1 if i + 1 < pop_size else 0]
            c1, c2 = crossover_uniform(p1, p2)
            next_population.append(mutation(c1, 0.2, search_bounds))
            next_population.append(mutation(c2, 0.2, search_bounds))

        population = next_population

    # Aplica as posições finais otimizadas na PCB
    for ref in rules.movable_refs:
        fp = fps[ref]
        best_x, best_y, best_ori = best_individual[ref]
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(best_x), pcbnew.FromMM(best_y)))
        fp.SetOrientationDegrees(best_ori)

    pcbnew.Refresh()
    
    print(f"\n[SUCESSO] Otimização escalável concluída com sucesso!")
    for ref in rules.movable_refs:
        best_x, best_y, _ = best_individual[ref]
        print(f"- {ref} posicionado em X:{best_x:.2f}mm, Y:{best_y:.2f}mm")

if __name__ == "__main__":
    board = pcbnew.GetBoard()
    run_ga_placement(board)