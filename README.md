# Otimizador de Posicionamento e Pré-Roteamento de PCBs com Algoritmo Genético no KiCad

Este repositório contém uma solução automatizada desenvolvida em Python para otimização do posicionamento (*placement*) e roteamento de componentes em Placas de Circuito Impresso (PCBs). Utilizando **Algoritmos Genéticos (AG)** integrados à API nativa do **KiCad (`pcbnew`)**, o projeto visa atender a regras de negócio e boas práticas de projeto eletrônico, garantindo menor distância entre componentes críticos e prevenindo colisões físicas.

---

## 📌 Visão Geral do Projeto

No desenvolvimento de hardware, a distribuição física dos componentes afeta diretamente o desempenho eletrônico, a integridade do sinal, a facilidade de roteamento e os custos de fabricação. A automação desse processo por meio de algoritmos evolucionários reduz a necessidade de intervenção manual ostensiva, permitindo otimizar requisitos como:

* **Proximidade Crítica:** Aproximação de componentes sensíveis a ruídos ou alta frequência (ex.: cristais osciladores e capacitores de desacoplamento) aos picos/pinos dos microcontroladores (MCU).


* **Design for Manufacturing (DFM) & Colisões:** Aplicação de margens de segurança para evitar sobreposição geométrica e invasão de footprints.


* **Roteamento Eficiente:** Otimização prévia do layout para simplificar a etapa posterior de roteamento de trilhas (*routing*) realizada por ferramentas como o *Freerouting*.



---

## 🛠️ Tecnologias e Ferramentas Utilizadas

* **Linguagem:** Python 3.11+.


* **EDAs / Suporte de Layout:** KiCad 10.0 (aplicando API `pcbnew`).


* **Bibliotecas Python:** `math`, `random`, `dataclasses`, `sys`, `importlib`.


* **Roteamento de Trilhas:** Integration via arquivo `.dsn` (Specctra) com *Freerouting*.



---

## 🧬 Arquitetura da Solução (Algoritmo Genético)

O motor evolutivo foi construído do zero, implementando as fases do AG adaptadas ao espaço bidimensional da PCB:

1. **Representação (Cromossomo):** Mapeia os componentes móveis com coordenadas e rotações: $(X, Y, \text{Orientação} \in \{0^\circ, 90^\circ, 180^\circ, 270^\circ\})$.


2. **População Inicial:** Gerada estocasticamente respeitando o contorno (*outline*) e limites da placa.


3. **Função de Aptidão (*Fitness Function*):**
* **Objetivo:** Maximizar a aptidão (minimizar custo total).


* **Custo de Distância:** Calcula a distância euclidiana de componentes críticos ao MCU ($U1$).


* **Penalidade de Colisão:** Aplica uma penalidade severa (ex.: $-1000.0$) caso a distância entre componentes fique abaixo da margem de segurança física (ex.: $7.5\text{ mm}$).



4. **Operadores Genéticos:**
* **Seleção:** Método de Torneio ($k=3$).


* **Recombinação (Crossover):** *Crossover Uniforme* trocando parâmetros de coordenadas entre pais com 50% de probabilidade.


* **Mutação:** Deslocamento estocástico nos eixos $X$ e $Y$ com alteração ocasional de rotação.



## 🚀 Como Executar

### Pré-requisitos

* KiCad v7 (ou superior com suporte para `pcbnew`) instalado.


* Arquivo do projeto e script salvos na máquina local (incluindo arquivos CAD da PCB e demais libs do projeto **em um mesmo diretório**).


### Execução no Console Python Interno do KiCad

1. Abra a placa no **KiCad PCB Editor**.


2. Acesse o console interativo do Python: **Ferramentas > Console de Scripting Python**.


3. Execute os seguintes comandos no console para rodar o algoritmo e ver as mudanças em tempo real na tela:



```python
import sys
import pcbnew

# Adicione o caminho do diretório onde o arquivo script_IA.py está localizado
sys.path.append(r"C:\Caminho\Para\Seu\Projeto")

# Importa o módulo do script
import script_IA

# Captura a placa ativa aberta no KiCad e executa a otimização
board = pcbnew.GetBoard()
script_IA.run_ga_placement(board)

```

---

## 🧪 Validação e Testes Modulares

A validação da lógica do AG concentrou-se na **regra isolada do Cristal Oscilador ($Y1$) e Microcontrolador ($U1$)**:

* **Objetivo:** Otimizar o posicionamento de $Y1$ para ficar o mais próximo possível de $U1$, respeitando a margem física de segurança.


* **Resultados:** Observou-se a convergência do algoritmo ao longo das gerações (com decréscimo na distância euclidiana) e a atualização automática da tela gráfica do KiCad através do comando `pcbnew.Refresh()`.

