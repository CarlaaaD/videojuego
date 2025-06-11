import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import heapq
import numpy as np
from objloader import *

class limites:
    def __init__(self, min_x, min_y, min_z, max_x, max_y, max_z):
        self.min = [min_x, min_y, min_z]
        self.max = [max_x, max_y, max_z]


paredes = [
    limites(-10.0477,0.200006, 11.4, 10.0458, 2.15955, 11.6),  
    limites(-10.0477, 0.200006, -8.0, 10.0458, 2.15955, -8.0),
    limites(-10.0477, 0.200006, -11.5, -10.0477, 2.15955, 11.5),
    limites(10.0458, 0.200006, -11.5, 10.0458, 2.15955, 11.5),
    limites(-1.31419,0.200006, 4.25001, 6.79527, 2.2849, 6),
    limites(1.31295,-0.002669,1.6, 1.45603, 1.94944, 4),
    limites(6.52937,-0.002669,-1.80711, 6.67245,2.19485, 4.72524),
    limites(-4.77165,-0.00267, 8.9, -0.287045,2.19485, 9),
    limites(-2.88816, -0.00267,4.34987, -2.74508,2.19485, 8.83293),
    limites(-7.16893,-0.00267,1.9, -7.01528, 2.19485, 9.5),
    limites(-7.16574,-0.00267,-6.9,-7.0198,2.19485,0.55),
    limites(-5.14312,-0.00267,-5.0,2.60089,2.19485, -4.5),
    limites(-0.599417,-0.00267,-6.8,-0.437577,2.19485, -2.25071)
]


def colision(new_x, new_y, new_z):
    tamañopl = 0.2
    for box in paredes:
        if (
            new_x - tamañopl < box.max[0] and
            new_x + tamañopl > box.min[0] and
            new_y - tamañopl < box.max[1] and
            new_y + tamañopl > box.min[1] and
            new_z - tamañopl < box.max[2] and
            new_z + tamañopl > box.min[2]
        ):
            return True
   
    if new_x < -12 or new_x > 12 or new_z < -13 or new_z > 13:
        return True
       
    return False


def nodos(min_x, max_x, min_z, max_z, step=1.0):
    nodes = {}
    node_id = 0
   
    x_coords = np.arange(min_x, max_x, step)
    z_coords = np.arange(min_z, max_z, step)
   
    for x in x_coords:
        for z in z_coords:
            if not colision(x, 0.5, z):
                nodes[node_id] = (x, z)
                node_id += 1
               
    return nodes


def grafo(nodes, max_distancia=1.5):
    graph = {}
   
    for node_id, (x, z) in nodes.items():
        graph[node_id] = []
        for other_id, (ox, oz) in nodes.items():
            if node_id == other_id:
                continue
            distancia = math.hypot(x - ox, z - oz)
            if distancia <= max_distancia:
                if cercania((x, z), (ox, oz)):
                    graph[node_id].append((other_id, distancia))
   
    return graph


def cercania(pos1, pos2, steps=10):
    x1, z1 = pos1
    x2, z2 = pos2
   
    for i in range(steps + 1):
        t = i / steps
        x = x1 * (1 - t) + x2 * t
        z = z1 * (1 - t) + z2 * t
        if colision(x, 0.5, z):
            return False
    return True


def algoA(start_id, goal_id, nodes, graph):
    open_set = []
    heapq.heappush(open_set, (0, start_id, []))
    visited = set()
   
    gn = {node_id: float('inf') for node_id in nodes}
    gn[start_id] = 0
   
    fn = {node_id: float('inf') for node_id in nodes}
    fn[start_id] = distanciaaprox(nodes[start_id], nodes[goal_id])
   
    while open_set:
        factual, idact, path = heapq.heappop(open_set)
       
        if idact in visited:
            continue
           
        visited.add(idact)
        path = path + [idact]
       
        if idact == goal_id:
            return path
           
        for vecino, distancia in graph.get(idact, []):
            if vecino in visited:
                continue
               
            gsus = gn[idact] + distancia
           
            if gsus < gn[vecino]:
                gn[vecino] = gsus
                fn[vecino] = gsus + distanciaaprox(nodes[vecino], nodes[goal_id])
                heapq.heappush(open_set, (fn[vecino], vecino, path))
   
    return []


def distanciaaprox(pos1, pos2):
    x1, z1 = pos1
    x2, z2 = pos2
    return math.hypot(x1 - x2, z1 - z2)


def nodocerca(x, z, nodes):
    min_distancia = float('inf')
    id = None
   
    for node_id, (nx, nz) in nodes.items():
        distancia = math.hypot(x - nx, z - nz)
        if distancia < min_distancia:
            min_distancia = distancia
            id = node_id
           
    return id


screen_width, screen_height = 1280, 720
EYE_X, EYE_Y, EYE_Z = 0.0, 1.5, 0.0
YAW = -90.0
velojugador = 0.04  
sensibilidad = 0.08


def init():
    pygame.init()
    pygame.display.set_mode((screen_width, screen_height), DOUBLEBUF | OPENGL)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(60, (screen_width/screen_height), 0.1, 500.0)
    glEnable(GL_DEPTH_TEST)
    pygame.mixer.music.load("Los_Cubitos.mp3")
    pygame.mixer.music.play(-1)


def update_camera():
    front_x = math.cos(math.radians(YAW))
    front_z = math.sin(math.radians(YAW))
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(
        EYE_X, EYE_Y, EYE_Z,
        EYE_X + front_x, EYE_Y, EYE_Z + front_z,
        0.0, 1.0, 0.0
    )




def main():
    global EYE_X, EYE_Y, EYE_Z, YAW

    init()
   
    model = None
    trex_model = None
   
    try:
        model = OBJ("backroom.obj", swapyz=True)
    except:
        pass
   
    try:
        trex_model = OBJ("trex.obj", swapyz=True)
    except:
        pass
   
    nodes = nodos(-10, 10, -11, 11, step=1.0)
    graph = grafo(nodes)
   
    clock = pygame.time.Clock()
    tiempo_inicio = pygame.time.get_ticks()  
   
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)

    dino_x, dino_z = 7, 7
    dino_speed = 0.05
    dino_path = []
    path_index = 0
    ultcamino = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False

        mouse_x, mouse_y = pygame.mouse.get_rel()
       
        if abs(mouse_x) < 100:  
            YAW += mouse_x * sensibilidad

        keys = pygame.key.get_pressed()
        front_x = math.cos(math.radians(YAW))
        front_z = math.sin(math.radians(YAW))
        right_x = math.cos(math.radians(YAW - 90))
        right_z = math.sin(math.radians(YAW - 90))

        if keys[K_w]:
            new_x = EYE_X + front_x * velojugador
            if not colision(new_x, EYE_Y, EYE_Z):
                EYE_X = new_x
            new_z = EYE_Z + front_z * velojugador
            if not colision(EYE_X, EYE_Y, new_z):
                EYE_Z = new_z

        if keys[K_s]:
            new_x = EYE_X - front_x * velojugador
            if not colision(new_x, EYE_Y, EYE_Z):
                EYE_X = new_x
            new_z = EYE_Z - front_z * velojugador
            if not colision(EYE_X, EYE_Y, new_z):
                EYE_Z = new_z

        if keys[K_a]:
            new_x = EYE_X + right_x * velojugador
            if not colision(new_x, EYE_Y, EYE_Z):
                EYE_X = new_x
            new_z = EYE_Z + right_z * velojugador
            if not colision(EYE_X, EYE_Y, new_z):
                EYE_Z = new_z

        if keys[K_d]:
            new_x = EYE_X - right_x * velojugador
            if not colision(new_x, EYE_Y, EYE_Z):
                EYE_X = new_x
            new_z = EYE_Z - right_z * velojugador
            if not colision(EYE_X, EYE_Y, new_z):
                EYE_Z = new_z

        current_time = pygame.time.get_ticks()
        distjugador = math.hypot(EYE_X - dino_x, EYE_Z - dino_z)
       
        if current_time - ultcamino > 500:
            dino_node = nodocerca(dino_x, dino_z, nodes)
            player_node = nodocerca(EYE_X, EYE_Z, nodes)
           
            if dino_node is not None and player_node is not None:
                new_path = algoA(dino_node, player_node, nodes, graph)
                if new_path:
                    dino_path = new_path
                    path_index = 0
            ultcamino = current_time

        if distjugador > 1.5 and len(dino_path) > 0 and path_index < len(dino_path):
            target_id = dino_path[path_index]
            tx, tz = nodes[target_id]
           
            dx, dz = tx - dino_x, tz - dino_z
            distancia = math.hypot(dx, dz)
           
            if distancia < 0.2:
                path_index += 1
            else:
                dino_x += (dx / distancia) * dino_speed
                dino_z += (dz / distancia) * dino_speed
        elif distjugador > 0.5:
            dx, dz = EYE_X - dino_x, EYE_Z - dino_z
            dino_x += (dx / distjugador) * dino_speed
            dino_z += (dz / distjugador) * dino_speed

        if distjugador < 0.8:
            running = False

        if current_time - tiempo_inicio >= 60000:
            print("Winner Winner chicken dinner")
            running = False

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        update_camera()

        if model:
            glColor3f(1.0, 1.0, 1.0)
            glPushMatrix()
            model.render()
            glPopMatrix()

        glColor3f(1, 0, 0)
        glPushMatrix()
        glTranslatef(dino_x, 0.0, dino_z)  
       
        
        if distjugador > 0.1:
            angle = math.degrees(math.atan2(EYE_X - dino_x, EYE_Z - dino_z))
            glRotatef(angle, 0, 1, 0)  
       
        if trex_model:
            glScalef(0.3, 0.3, 0.3)  
            trex_model.render()
       
        glPopMatrix()

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()