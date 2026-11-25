# Otimizador de Posicionamento de Componentes em PCBs com Algoritmo Genético (KiCad)

Este repositório contém uma ferramenta de automação desenvolvida em Python para otimizar o posicionamento (*placement*) de componentes eletrônicos em Placas de Circuito Impresso (PCBs). Integrado diretamente à API nativa do **KiCad (`pcbnew`)**, o sistema utiliza um **Algoritmo Genético (AG)** híbrido (com busca local determinística) para distribuir componentes respeitando regras elétricas, integridade de sinal e restrições mecânicas de fabricação.


---

## Visão Geral e Motivação

No projeto de hardware, a disposição física dos componentes impacta diretamente o desempenho eletromagnético, a imunidade a ruídos, a dissipação térmica e a viabilidade de soldagem. O otimizador traduz boas práticas de engenharia de PCB em restrições matemáticas e pesos de atração:

- **Integridade de Sinal e RF (2.4 GHz):** Agrupamento compacto das redes de antena e casamento de impedância (`/ANT_SIG`, `Net-(J1-In)`), mantendo filtros e conectores coaxiais próximos aos pinos de RF do microcontrolador.
- **Estabilidade de Clock e Desacoplamento:** Alta prioridade de proximidade para cristais osciladores (`Y1`, `Y2`) e capacitores de desacoplamento (`C1`–`C4`) junto aos pinos de alimentação e oscilação dos CIs.
- **Proteção na Entrada de Energia:** Ancoragem do fusível de proteção (`F1`) diretamente próximo ao terminal positivo da bateria (`BT2`).
- **Design for Manufacturing (DFM):** Garantia matemática de zero sobreposição física entre corpos/pads (com folga mínima configurável para soldagem) e respeito rigoroso ao contorno real da placa (`Edge.Cuts`).
- **Organização Automática da Serigrafia:** Ocultação de textos de valores poluídos (`Value`) e reposicionamento automático dos designadores (`Reference`) na camada `F.Silkscreen` sem colisões.
- **Relatório de Evolução do Algoritmo:** Registro das gerações executadas, com informações sobre cada componente e as mutações sofridas durante o processo evolutivo.
- **Limpeza Opcional de Serigrafia e Cobre:** Disponibilização de funções para limpeza automática da serigrafia e das trilhas de cobre, permitindo que o usuário escolha quais operações deseja executar.

---

## Tecnologias e Dependências

- **Linguagem:** Python 3.11+
- **EDA / API:** KiCad (v7.0 ou superior, compatível com KiCad 8/9/10) via módulo nativo `pcbnew`
- **Bibliotecas Padrões:** `math`, `random`, `dataclasses`

---

## Arquitetura da Solução

O script foi estruturado em módulos funcionais independentes:

1. **Regras de Engenharia (`BoardRules`):**
   - Separação clara entre **componentes móveis** (`movable_refs`) e **componentes âncora fixos** (`anchor_refs`: microcontrolador `U1`, acelerômetro `U2`, suporte de bateria `BT2` e chaves `SW1`/`SW2`).
   - Pesos individuais por componente, agrupamento mútuo por rede (`group_nets`) e atração de borda (`edge_weights`).
2. **Extração de Contexto da PCB (`Context`):**
   - Leitura dinâmica das coordenadas reais dos *pads* a partir das *nets* da placa (com suporte a mapa de contingência `fallback_map`).
   - Extração do polígono real de contorno (`Edge.Cuts`) para validação geométrica.
3. **Motor Evolutivo (Algoritmo Genético):**
   - **Representação (Cromossomo):** Dicionário mapeando cada componente móvel para $(X, Y, \theta)$, onde $\theta \in \{0^\circ, 90^\circ, 180^\circ, 270^\circ\}$.
   - **Inicialização Gulosa (`create_individual`):** Gera indivíduos iniciais posicionando peças sequencialmente ao redor de seus pinos-alvo sem colisões.
   - **Função de Aptidão (*Fitness*):** Minimiza a soma ponderada das distâncias euclidianas entre pinos conectados e aplica penalidades severas ($10^4 \times \text{violação}$) para invasão de folga mínima (`clearance_mm = 0.35 mm`) ou saída da margem de borda (`edge_margin_mm = 1.50 mm`).
   - **Operadores Genéticos:** Seleção por torneio ($k=3$), elitismo, *crossover* uniforme (50%) e mutação gaussiana com decaimento linear de passo e saltos ocasionais para escapar de ótimos locais.
4. **Refinamento Local e Pós-Processamento:**
   - **Descida por Coordenadas (`local_refinement`):** Ajuste fino determinístico em passos decrescentes ($0.25\text{ mm}$ até $0.01\text{ mm}$) ao final das gerações.
   - **Limpeza de Serigrafia (`tidy_silk`):** Função opcional responsável pela organização dos textos de referência na camada `F.Silkscreen`, evitando colisões e melhorando a legibilidade.
   - **Limpeza de Cobre e Trilhas (`clean_copper`):** Função opcional que permite executar a limpeza de cobre e trilhas da placa, conforme a implementação do script.
   - **Controle pelo Usuário:** As operações de limpeza podem ser ativadas ou desativadas individualmente durante a chamada do algoritmo, por meio de parâmetros booleanos.
---



---

## Parâmetros Configuráveis (`BoardRules` e Execução)

Abaixo estão os principais hiperparâmetros evolutivos e restrições mecânicas que controlam o comportamento do otimizador:

| Parâmetro | Valor Padrão | Local | Descrição |
| :--- | :---: | :---: | :--- |
| `clearance_mm` | `0.35` | `BoardRules` | Folga mínima (mm) entre corpos/pads de componentes adjacentes para garantir soldagem segura sem curtos. |
| `edge_margin_mm` | `1.50` | `BoardRules` | Distância mínima (mm) entre qualquer componente móvel e o contorno externo (`Edge.Cuts`) da placa. |
| `group_weight` | `28.0` | `BoardRules` | Peso de coesão aplicado a componentes que compartilham redes críticas (como linhas de RF e filtros). |
| `init_radius_mm` | `5.0` | `BoardRules` | Raio máximo (mm) ao redor dos pinos-alvo utilizado na geração gulosa da população inicial. |
| `pop_size` | `160` | `run_ga_placement` | Número de indivíduos (layouts candidatos) avaliados simultaneamente em cada geração. |
| `generations` | `250` | `run_ga_placement` | Total de gerações evolutivas executadas antes do refinamento local (*Hill Climbing*). |
| `elite` | `2` | `run_ga_placement` | Quantidade dos melhores indivíduos preservados intactos para a geração seguinte (elitismo). |
| `seed` | `None` | `run_ga_placement` | Semente do gerador pseudoaleatório; defina um inteiro (ex.: `42`) para obter resultados 100% reproduzíveis. |
| `clean_copper` | `True` | `run_ga_placement` | Ativa ou desativa a limpeza de cobre e trilhas. |
| `tidy_silk` | `True` | `run_ga_placement` | Ativa ou desativa a organização automática da serigrafia. |

---

## Guia de Adaptação para Outras Placas

Para reutilizar este otimizador em outro projeto de PCB no KiCad, basta editar a instância `BOARD_RULES` no início do script[cite: 12]:

1. **Defina os componentes móveis e fixos:**
   - Adicione em `movable_refs` os designadores de passivos, cristais, fusíveis e conectores que podem ser reposicionados (`"R1"`, `"C1"`, `"Y1"`, etc.)[cite: 12].
   - Adicione em `anchor_refs` os circuitos integrados principais e componentes mecânicos fixos (`"U1"`, `"BT2"`, `"SW1"`, etc.)[cite: 12].
2. **Configure as atrações e grupos críticos:**
   - Em `attract_to`, associe cada componente móvel ao CI de referência correspondente para evitar atrações cruzadas indesejadas[cite: 12].
   - Em `group_nets`, liste as redes que exigem alta proximidade mútua entre passivos (ex.: casamento de impedância de antena ou filtros LC)[cite: 12].
3. **Ajuste os pesos de prioridade (`weights`):**
   - Atribua valores maiores (ex.: `30.0` a `45.0`) para cristais osciladores, circuitos de RF e proteção de entrada, e valores intermediários (`10.0` a `20.0`) para capacitores de desacoplamento e resistores de *pull-up*[cite: 12].
   - Caso alguma conexão não seja identificada automaticamente pelas *nets* da placa, declare o par `(CI, pino)` correspondente em `fallback_map`[cite: 12].

---


## Como Executar

### Pré-requisitos

- KiCad instalado com suporte ao console de *scripting* Python (`pcbnew`).
- Arquivo da placa (`.kicad_pcb`) devidamente atualizado com a *netlist* do esquemático e o contorno fechado na camada `Edge.Cuts`.

### Execução no Console Python do KiCad

1. Abra o arquivo `.kicad_pcb` no **Editor de PCB do KiCad**.
2. Abra o console interativo em **Ferramentas > Console de Scripting Python**.
3. Execute os comandos abaixo ajustando o caminho para a pasta do seu projeto:

```python
import sys
import importlib
import pcbnew

# Adicione o diretório onde o script está salvo
sys.path.append(r"C:\Caminho\Para\Seu\Projeto")

import script_IA
importlib.reload(script_IA)

# Captura a placa aberta no KiCad e executa a otimização
board = pcbnew.GetBoard()

# Use seed=None para busca estocástica ou fixe um inteiro (ex.: seed=42) para reprodutibilidade
script_IA.run_ga_placement(
    pcbnew.GetBoard(),
    clean_copper=True,
    tidy_silk=True
)

Após a conclusão no console, abra o Visualizador 3D (Alt + 3) ou execute o DRC (Inspecionar > Verificador de Regras de Projeto) para validar as folgas físicas.


## Limitações Conhecidas e Boas Práticas

- **Camada Única (`Top Layer` / `F.Cu`):** A versão atual otimiza apenas componentes posicionados na face superior da placa (`F.Cu`), sem realizar inversão automática (*flip*) para a camada inferior (`B.Cu`).
- **Contorno Fechado (`Edge.Cuts`):** Para que a validação poligonal funcione corretamente, o desenho da borda da placa deve formar um polígono fechado e contínuo; caso contrário, o algoritmo utilizará apenas o retângulo envolvente (*bounding box*).
- **Natureza Estocástica:** Por utilizar operadores probabilísticos, execuções com `seed=None` podem convergir para variações simétricas de custo equivalente. Recomenda-se testar algumas execuções e fixar o valor de `seed` assim que encontrar o layout desejado.
- **Validação Final pelo DRC:** Embora o script garanta folgas entre caixas delimitadoras (*bounding boxes*) dos footprints, sempre execute o **Verificador de Regras de Projeto (DRC)** nativo do KiCad após a execução para validar restrições específicas de máscara de solda e serigrafia.




