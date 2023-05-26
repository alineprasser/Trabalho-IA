import time  # utilizado para calcular o tempo decorrido da execução dos algoritmos
import random  # utilizado para gerar labirintos aleatórios

from collections import deque  # utilizado para as operações com fila/pilha
from viewer import MazeViewer  # utilizado para a visualização do labirinto gerado
from math import inf, sqrt  # utilizado para operações matemáticas


def gera_labirinto(n_linhas, n_colunas, inicio, goal):
  # cria labirinto vazio
  labirinto = [[0] * n_colunas for _ in range(n_linhas)]

  # adiciona celulas ocupadas em locais aleatorios de
  # forma que 50% do labirinto esteja ocupado
  numero_de_obstaculos = int(0.50 * n_linhas * n_colunas)
  for _ in range(numero_de_obstaculos):
    linha = random.randint(0, n_linhas - 1)
    coluna = random.randint(0, n_colunas - 1)
    labirinto[linha][coluna] = 1

  # remove eventuais obstaculos adicionados na posicao
  # inicial e no goal
  labirinto[inicio.y][inicio.x] = 0
  labirinto[goal.y][goal.x] = 0

  return labirinto


class Celula:

  def __init__(self, y, x, anterior):
    self.y = y  # localização vertical
    self.x = x  # localização horizontal
    self.anterior = anterior  # memória de seu antecessor
    self.custo = 0  # memória de seu custo acumulado


def insere_ordenado(lista, valor):
  # adiciona o valor à lista
  lista.append(valor)
  # gera uma lista nova ordenando os valores pelo atributo custo
  # fazendo com que os menores custos estejam no início da lista
  lista_ordenada = sorted(lista, key=lambda celula: celula.custo)
  return deque(lista_ordenada)


def distancia(celula_1, celula_2):
  # cálculo da distância euclidiana de duas células
  dx = celula_1.x - celula_2.x
  dy = celula_1.y - celula_2.y
  return sqrt(dx**2 + dy**2)


def heuristica(celula, goal):
  # cálculo da distância manhattan de duas células
  return abs(celula.y - goal.y) + abs(celula.y - goal.y)


def esta_contido(lista, celula):
  # valida se a célula já foi inserida dentro da lista
  for elemento in lista:
    if (elemento.y == celula.y) and (elemento.x == celula.x):
      return True
  return False


def custo_caminho(caminho):
  # calcula o custo do caminho fornecido
  if len(caminho) == 0:
    return inf

  custo_total = 0
  for i in range(1, len(caminho)):
    # o custo total se dará pela soma das distâncias de cada célula e seu próximo vizinho
    custo_total += distancia(caminho[i].anterior, caminho[i])

  return custo_total


def obtem_caminho(goal):
  # retorna o caminho final
  caminho = []

  celula_atual = goal
  while celula_atual is not None:
    caminho.append(celula_atual)
    celula_atual = celula_atual.anterior

  # o caminho foi gerado do final para o
  # comeco, entao precisamos inverter.
  caminho.reverse()

  return caminho


def celulas_vizinhas_livres(celula_atual, labirinto):
  # gera os vizinhos da célula atual
  vizinhos = [
    Celula(y=celula_atual.y - 1, x=celula_atual.x - 1,
           anterior=celula_atual),  # vizinho diagonal inferior esquerda
    Celula(y=celula_atual.y + 0, x=celula_atual.x - 1,
           anterior=celula_atual),  # vizinho diretamente à esquerda
    Celula(y=celula_atual.y + 1, x=celula_atual.x - 1,
           anterior=celula_atual),  # vizinho diagonal superior esquerda
    Celula(y=celula_atual.y - 1, x=celula_atual.x + 0,
           anterior=celula_atual),  # vizinho diretamente abaixo
    Celula(y=celula_atual.y + 1, x=celula_atual.x + 0,
           anterior=celula_atual),  # vizinho diretamente acima
    Celula(y=celula_atual.y + 1, x=celula_atual.x + 1,
           anterior=celula_atual),  # vizinho diagonal superior direita
    Celula(y=celula_atual.y + 0, x=celula_atual.x + 1,
           anterior=celula_atual),  # vizinho diretamente à direita
    Celula(y=celula_atual.y - 1, x=celula_atual.x + 1,
           anterior=celula_atual),  # vizinho diagonal inferior direita
  ]

  # seleciona as celulas livres
  vizinhos_livres = []
  for v in vizinhos:
    # verifica se a celula esta dentro dos limites do labirinto
    if (v.y < 0) or (v.x < 0) or (v.y >= len(labirinto)) or (v.x >= len(
        labirinto[0])):
      continue
    # verifica se a celula esta livre de obstaculos.
    if labirinto[v.y][v.x] == 0:
      vizinhos_livres.append(v)

  return vizinhos_livres


def breadth_first_search(labirinto, inicio, goal, viewer):
  # nos gerados e que podem ser expandidos (vermelhos)
  fronteira = deque()
  # nos ja expandidos (amarelos)
  expandidos = set()

  # adiciona o no inicial na fronteira
  fronteira.append(inicio)

  # variavel para armazenar o goal quando ele for encontrado
  goal_encontrado = None

  # Repete enquanto nos nao encontramos o goal e ainda
  # existem para serem expandidos na fronteira. Se
  # acabarem os nos da fronteira antes do goal ser encontrado,
  # entao ele nao eh alcancavel.
  while (len(fronteira) > 0) and (goal_encontrado is None):
    # seleciona o no mais antigo para ser expandido
    no_atual = fronteira.popleft()

    # busca os vizinhos do no
    vizinhos = celulas_vizinhas_livres(no_atual, labirinto)

    # para cada vizinho verifica se eh o goal e adiciona na
    # fronteira se ainda nao foi expandido e nao esta na fronteira
    for v in vizinhos:
      if v.y == goal.y and v.x == goal.x:
        goal_encontrado = v
        # encerra o loop interno
        break
      else:
        if (not esta_contido(expandidos, v)) and (not esta_contido(
            fronteira, v)):
          fronteira.append(v)

    expandidos.add(no_atual)

    viewer.update(generated=fronteira, expanded=expandidos)
    # viewer.pause()

  caminho = obtem_caminho(goal_encontrado)
  custo = custo_caminho(caminho)

  return caminho, custo, expandidos


def depth_first_search(labirinto, inicio, goal, viewer):
  # nos gerados e que podem ser expandidos (vermelhos)
  fronteira = deque()
  # nos ja expandidos (amarelos)
  expandidos = set()

  # adiciona o no inicial na fronteira
  fronteira.append(inicio)

  # variavel para armazenar o goal quando ele for encontrado.
  goal_encontrado = None

  # Repete enquanto nos nao encontramos o goal e ainda
  # existem para serem expandidos na fronteira. Se
  # acabarem os nos da fronteira antes do goal ser encontrado,
  # entao ele nao eh alcancavel.
  while (len(fronteira) > 0) and (goal_encontrado is None):
    # seleciona o nó recém adicionado na lista para ser expandido
    no_atual = fronteira.pop()

    # busca os vizinhos do no
    vizinhos = celulas_vizinhas_livres(no_atual, labirinto)

    # para cada vizinho verifica se eh o goal e adiciona na
    # fronteira se ainda nao foi expandido e nao esta na fronteira
    for v in vizinhos:
      if v.y == goal.y and v.x == goal.x:
        goal_encontrado = v
        break
      else:
        if (not esta_contido(expandidos, v)) and (not esta_contido(
            fronteira, v)):
          fronteira.append(v)

    expandidos.add(no_atual)

    viewer.update(generated=fronteira, expanded=expandidos)
    # viewer.pause()

  caminho = obtem_caminho(goal_encontrado)
  custo = custo_caminho(caminho)

  return caminho, custo, expandidos


def a_star_search(labirinto, inicio, goal, viewer):
  # nos gerados e que podem ser expandidos (vermelhos)
  fronteira = deque()
  # nos ja expandidos (amarelos)
  expandidos = set()

  # adiciona o no inicial na fronteira
  fronteira.append(inicio)

  # variavel para armazenar o goal quando ele for encontrado.
  goal_encontrado = None

  # Repete enquanto nos nao encontramos o goal e ainda
  # existem para serem expandidos na fronteira. Se
  # acabarem os nos da fronteira antes do goal ser encontrado,
  # entao ele nao eh alcancavel.
  while (len(fronteira) > 0) and (goal_encontrado is None):
    # seleciona o no com menor custo para ser expandido
    no_atual = fronteira.popleft()
    # busca os vizinhos do no
    vizinhos = celulas_vizinhas_livres(no_atual, labirinto)
    # para cada vizinho verifica se eh o goal e adiciona na
    # fronteira se ainda nao foi expandido e nao esta na fronteira
    for v in vizinhos:
      if v.y == goal.y and v.x == goal.x:
        goal_encontrado = v
        # encerra o loop interno
        break
      else:
        if (not esta_contido(expandidos, v)) and (not esta_contido(
            fronteira,
            v)):  # para cada vizinho, calcula as distâncias e as heurísticas
          # faz o cálculo do custo considerando f(x) = g(x) + h(x)
          custo_g = distancia(v, inicio)  # distância do nó atual e o inicial
          custo_h = heuristica(v, goal)  # distÂncia do nó atual e o objetivo
          custo_f = custo_g + custo_h
          v.custo = custo_f

          fronteira = insere_ordenado(
            fronteira, v
          )  # insere de forma ordenada para manter o menor custo no início da lista

    expandidos.add(no_atual)

    viewer.update(generated=fronteira, expanded=expandidos)
    # viewer.pause()

  caminho = obtem_caminho(goal_encontrado)
  custo = custo_caminho(caminho)

  return caminho, custo, expandidos


def uniform_cost_search(labirinto, inicio, goal, viewer):
  # nos gerados e que podem ser expandidos (vermelhos)
  fronteira = deque()
  # nos ja expandidos (amarelos)
  expandidos = set()

  # adiciona o no inicial na fronteira
  fronteira.append(inicio)

  # variavel para armazenar o goal quando ele for encontrado.
  goal_encontrado = None

  # Repete enquanto nos nao encontramos o goal e ainda
  # existem para serem expandidos na fronteira. Se
  # acabarem os nos da fronteira antes do goal ser encontrado,
  # entao ele nao eh alcancavel.
  while (len(fronteira) > 0) and (goal_encontrado is None):
    # seleciona o no com menor custo para ser expandido
    no_atual = fronteira.popleft()
    # busca os vizinhos do no
    vizinhos = celulas_vizinhas_livres(no_atual, labirinto)
    # para cada vizinho verifica se eh o goal e adiciona na
    # fronteira se ainda nao foi expandido e nao esta na fronteira
    for v in vizinhos:
      if v.y == goal.y and v.x == goal.x:
        goal_encontrado = v
        # encerra o loop interno
        break
      else:
        if (not esta_contido(expandidos, v)) and (not esta_contido(
            fronteira,
            v)):  # para cada vizinho, calcula as distâncias e as heurísticas
          # faz o cálculo do custo considerando a heurística = 0
          custo_g = distancia(v, inicio)
          custo_h = 0  # a heurística = 0 faz com que apenas a distância já percorrida seja levada em consideração
          custo_f = custo_g + custo_h
          v.custo = custo_f

          fronteira = insere_ordenado(fronteira, v)

    expandidos.add(no_atual)

    viewer.update(generated=fronteira, expanded=expandidos)
    # viewer.pause()

  caminho = obtem_caminho(goal_encontrado)
  custo = custo_caminho(caminho)

  return caminho, custo, expandidos


#-------------------------------


def main():
  i = 0
  while i < 1:
    i += 1
    #SEED = 42  # coloque None no lugar do 42 para deixar aleatorio
    #random.seed(SEED)
    N_LINHAS = 20
    N_COLUNAS = 30
    INICIO = Celula(y=0, x=0, anterior=None)
    GOAL = Celula(y=N_LINHAS - 1, x=N_COLUNAS - 1, anterior=None)
    """
        O labirinto sera representado por uma matriz (lista de listas)
        em que uma posicao tem 0 se ela eh livre e 1 se ela esta ocupada.
        """
    labirinto = gera_labirinto(N_LINHAS, N_COLUNAS, INICIO, GOAL)

    viewer = MazeViewer(labirinto,
                        INICIO,
                        GOAL,
                        step_time_miliseconds=20,
                        zoom=5)

    #----------------------------------------
    # BFS Search
    #----------------------------------------
    viewer._figname = "BFS"
    start_time = time.time()
    caminho, custo_total, expandidos = \
            breadth_first_search(labirinto, INICIO, GOAL, viewer)
    elapsed_time = time.time() - start_time

    if len(caminho) == 0:
      print("Goal é inalcançavel neste labirinto.")

    print(f"BFS:\n"
          f"\tTempo de execução: {elapsed_time}.\n"
          f"\tNumero total de nos expandidos: {len(expandidos)}.\n"
          f"\tCusto total do caminho: {custo_total}.\n"
          f"\tTamanho do caminho: {len(caminho)-1}.\n\n")

    viewer.update(path=caminho)
    # viewer.pause()

    #----------------------------------------
    # DFS Search
    #----------------------------------------
    viewer._figname = "DFS"
    start_time = time.time()
    caminho, custo_total, expandidos = \
            depth_first_search(labirinto, INICIO, GOAL, viewer)
    elapsed_time = time.time() - start_time

    if len(caminho) == 0:
      print("Goal é inalcançavel neste labirinto.")

    print(f"DFS:\n"
          f"\tTempo de execução: {elapsed_time}.\n"
          f"\tNumero total de nos expandidos: {len(expandidos)}.\n"
          f"\tCusto total do caminho: {custo_total}.\n"
          f"\tTamanho do caminho: {len(caminho)-1}.\n\n")

    viewer.update(path=caminho)
    # viewer.pause()

    #----------------------------------------
    # A-Star Search
    #----------------------------------------
    # h = 0 -> dijkstra ou custo uniforme
    # g = 0 -> guloso
    # f(n) = g(n) + h(n)
    # pra ser admissivel o h tem que ser menor do que o valor real

    viewer.update(path=caminho)
    # viewer.pause()
    viewer._figname = "A-Star"
    start_time = time.time()
    caminho, custo_total, expandidos = \
            a_star_search(labirinto, INICIO, GOAL, viewer)
    elapsed_time = time.time() - start_time

    if len(caminho) == 0:
      print("Goal é inalcançavel neste labirinto.")

    print(f"A Star:\n"
          f"\tTempo de execução: {elapsed_time}.\n"
          f"\tNumero total de nos expandidos: {len(expandidos)}.\n"
          f"\tCusto total do caminho: {custo_total}.\n"
          f"\tTamanho do caminho: {len(caminho)-1}.\n\n")

    viewer.update(path=caminho)
    # viewer.pause()

    #----------------------------------------
    # Uniform Cost Search (Obs: opcional)
    #----------------------------------------

    viewer._figname = "UniformCost"
    start_time = time.time()
    caminho, custo_total, expandidos = \
            uniform_cost_search(labirinto, INICIO, GOAL, viewer)
    elapsed_time = time.time() - start_time

    if len(caminho) == 0:
      print("Goal é inalcançavel neste labirinto.")

    print(f"Uniform Cost:\n"
          f"\tTempo de execução: {elapsed_time}.\n"
          f"\tNumero total de nos expandidos: {len(expandidos)}.\n"
          f"\tCusto total do caminho: {custo_total}.\n"
          f"\tTamanho do caminho: {len(caminho)-1}.\n\n")

    viewer.update(path=caminho)
    # viewer.pause()

  print("OK! Pressione alguma tecla pra finalizar...")
  input()


if __name__ == "__main__":
  main()
