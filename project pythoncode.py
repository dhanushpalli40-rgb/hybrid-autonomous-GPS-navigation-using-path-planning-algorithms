"""
hybrid_50x50_gps_realtime.py

Corrected constructors (__init__), fixed gps distance formula, minor robustness fixes.
"""

import math
import random
import time
import heapq
import requests
import pygame
import numpy as np
from typing import Tuple, List, Dict, Optional, Any
import logging

# Logging
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# Types
Cell = Tuple[int, int]
GPSCoord = Tuple[float, float]

# ----------------------------
# Configuration
# ----------------------------
class Config:
    # ESP32 Communication (change IP if needed)
    ESP32_IP = "192.168.202.1"
    MOVE_URL = f"http://{ESP32_IP}/move"
    ULTRA_URL = f"http://{ESP32_IP}/ultrasonic"
    STOP_URL = f"http://{ESP32_IP}/stop"
    SPEED_URL = f"http://{ESP32_IP}/speed"
    REQUEST_TIMEOUT = 0.8

    # Grid & visuals
    GRID_SIZE = 30         # you can set to 50 for original; 30 faster for demos
    CELL_SIZE = 18
    INITIAL_OBSTACLE_RATE = 0.01
    DYNAMIC_OBS_PROB = 0.01
    FPS = 12
    SELECTION_FPS = 30

    # GPS (display only)
    GPS_BASE_LAT = 10.732500
    GPS_BASE_LON = 79.015054
    GPS_LAT_STEP = 0.0001
    GPS_LON_STEP = 0.0001

    # Robot timings (these are used to sleep after sending move commands)
    MOVE_STEP_TIME = 0.55
    TURN_TIME = 0.45
    STOP_PAUSE = 0.6

    # Sensor threshold (cm)
    ULTRASONIC_THRESHOLD_CM = 25.0

    # ACO
    ACO_ANTS = 12
    ACO_ITERS = 10
    ALPHA = 1.0
    BETA = 3.0
    EVAP = 0.4
    Q = 40.0

    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRID_LINE = (200, 200, 200)
    START_GREEN = (0, 180, 0)
    GOAL_ORANGE = (200, 100, 0)
    OBSTACLE = (80, 80, 80)
    DYN_OBS = (120, 60, 60)
    ROBOT = (160, 50, 200)
    PATH_A = (40, 120, 255)    # A*
    PATH_D = (80, 200, 120)    # Dijkstra
    PATH_C = (255, 160, 40)    # ACO
    BEST_PATH = (0, 200, 0)

# ----------------------------
# GPS / utilities
# ----------------------------
def cell_to_gps(cell: Cell) -> GPSCoord:
    x, y = cell
    lat = Config.GPS_BASE_LAT + (Config.GRID_SIZE - 1 - y) * Config.GPS_LAT_STEP
    lon = Config.GPS_BASE_LON + x * Config.GPS_LON_STEP
    return (lat, lon)

def gps_distance(g1: GPSCoord, g2: GPSCoord) -> float:
    # Haversine formula
    lat1, lon1 = g1; lat2, lon2 = g2
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1); dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def in_bounds(cell: Cell) -> bool:
    x, y = cell
    return 0 <= x < Config.GRID_SIZE and 0 <= y < Config.GRID_SIZE

def neighbors4(cell: Cell) -> List[Cell]:
    x, y = cell
    res = []
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx, ny = x+dx, y+dy
        if in_bounds((nx, ny)):
            res.append((nx, ny))
    return res

def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclid(a: Cell, b: Cell) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

# ----------------------------
# Grid
# ----------------------------
class Grid:
    def __init__(self, n: int, density: float=0.01, seed: Optional[int]=None):
        self.n = n
        if seed is not None:
            random.seed(seed)
        self.grid = np.zeros((n, n), dtype=np.uint8)  # 0 free, 1 static, 2 dynamic
        if density > 0:
            for y in range(n):
                for x in range(n):
                    if random.random() < density:
                        self.grid[y, x] = 1

    def is_free(self, cell: Cell) -> bool:
        x, y = cell
        return in_bounds(cell) and self.grid[y, x] == 0

    def set_static(self, cell: Cell):
        if in_bounds(cell):
            x, y = cell
            self.grid[y, x] = 1

    def set_dynamic(self, cell: Cell):
        if in_bounds(cell):
            x, y = cell
            self.grid[y, x] = 2

    def clear(self, cell: Cell):
        if in_bounds(cell):
            x, y = cell
            self.grid[y, x] = 0

    def random_free_cell(self) -> Optional[Cell]:
        free = np.argwhere(self.grid == 0)
        if len(free) == 0:
            return None
        y, x = free[random.randrange(len(free))]
        return (int(x), int(y))

# ----------------------------
# Simulated Robot State
# ----------------------------
class SimRobot:
    # E, N, W, S vectors (0..3)
    VECTORS = [(1,0),(0,-1),(-1,0),(0,1)]
    def __init__(self, grid: Grid, start: Cell, goal: Cell, orient: int = 0):
        self.grid = grid
        self.pos = start
        self.goal = goal
        self.orient = orient % 4
        self.history: List[Cell] = [start]

    def front_cell(self) -> Cell:
        v = self.VECTORS[self.orient]
        return (self.pos[0] + v[0], self.pos[1] + v[1])

    def step_to(self, cell: Cell) -> bool:
        if not in_bounds(cell) or not self.grid.is_free(cell):
            return False
        dx = cell[0] - self.pos[0]; dy = cell[1] - self.pos[1]
        for i,v in enumerate(self.VECTORS):
            if (dx,dy) == v:
                self.orient = i
                break
        self.pos = cell
        self.history.append(cell)
        return True

    def turn_left(self):
        self.orient = (self.orient + 1) % 4

    def turn_right(self):
        self.orient = (self.orient - 1) % 4

# ----------------------------
# Robot HTTP Controller
# ----------------------------
class RobotController:
    def __init__(self, move_url: str, ultra_url: str, stop_url: str, speed_url: str, timeout: float=0.7):
        self.move_url = move_url
        self.ultra_url = ultra_url
        self.stop_url = stop_url
        self.speed_url = speed_url
        self.timeout = timeout

    def move(self, cmd: str) -> bool:
        """Sends move command to ESP32. cmd in {'F','B','L','R','S'}"""
        try:
            resp = requests.get(self.move_url, params={"dir": cmd}, timeout=self.timeout)
            logging.debug(f"Sent CMD {cmd} -> {getattr(resp, 'status_code', 'no-code')}")
            # Wait appropriate time to let real robot act
            if cmd in ("F","B"):
                time.sleep(Config.MOVE_STEP_TIME)
            elif cmd in ("L","R"):
                time.sleep(Config.TURN_TIME)
            elif cmd == "S":
                time.sleep(0.08)
            return True
        except Exception as e:
            logging.warning(f"Move {cmd} failed: {e}")
            return False

    def stop(self) -> None:
        try:
            requests.get(self.stop_url, timeout=self.timeout)
        except Exception:
            pass

    def set_speed(self, val: int) -> None:
        try:
            requests.get(self.speed_url, params={"val": str(int(val))}, timeout=self.timeout)
        except Exception:
            pass

    def get_sensor_data(self) -> Dict[str, Any]:
        """Return {'left': float, 'right': float, 'obstacle': bool}."""
        try:
            r = requests.get(self.ultra_url, timeout=self.timeout)
            data = r.json()
            left = float(data.get("left", 999.0))
            right = float(data.get("right", 999.0))
            obs = (left <= Config.ULTRASONIC_THRESHOLD_CM) or (right <= Config.ULTRASONIC_THRESHOLD_CM)
            logging.debug(f"Sensor L/R: {left:.1f}/{right:.1f} -> obs={obs}")
            return {"left": left, "right": right, "obstacle": obs}
        except Exception as e:
            logging.warning(f"Sensor read failed: {e}")
            return {"left": 999.0, "right": 999.0, "obstacle": False}

# ----------------------------
# Path Planner with A*, Dijkstra, ACO
# ----------------------------
class PathPlanner:
    def __init__(self, grid: Grid, config: Config):
        self.grid = grid
        self.config = config

    def _reconstruct_path(self, came: Dict[Cell, Optional[Cell]], goal: Cell) -> List[Cell]:
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came.get(cur)
        return list(reversed(path))

    def plan_best_path(self, start: Cell, goal: Cell) -> (Dict[str, Optional[List[Cell]]], Optional[List[Cell]], str):
        p_a = self._astar(start, goal)
        p_d = self._dijkstra(start, goal)
        p_c = self._aco(start, goal)
        paths = {"A*": p_a, "Dijkstra": p_d, "ACO": p_c}
        lengths = {k:(len(v) if v else float('inf')) for k,v in paths.items()}
        best_algo = min(lengths, key=lengths.get)
        best_path = paths[best_algo]
        logging.info(f"Planner: best={best_algo} len={lengths[best_algo]}")
        return paths, best_path, best_algo

    def _astar(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        if not self.grid.is_free(start) or not self.grid.is_free(goal):
            return None
        openq = []
        heapq.heappush(openq, (manhattan(start, goal), start))
        came = {start: None}
        gscore = {start: 0}
        visited = set()
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == goal:
                return self._reconstruct_path(came, goal)
            if cur in visited:
                continue
            visited.add(cur)
            for n in neighbors4(cur):
                if not self.grid.is_free(n) and n != goal:
                    continue
                tentative_g = gscore[cur] + 1
                if tentative_g < gscore.get(n, float('inf')):
                    came[n] = cur
                    gscore[n] = tentative_g
                    f = tentative_g + manhattan(n, goal)
                    heapq.heappush(openq, (f, n))
        return None

    def _dijkstra(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        if not self.grid.is_free(start) or not self.grid.is_free(goal):
            return None
        pq = [(0, start)]
        came = {start: None}
        dist = {start: 0}
        visited = set()
        while pq:
            d, cur = heapq.heappop(pq)
            if cur == goal:
                return self._reconstruct_path(came, goal)
            if cur in visited:
                continue
            visited.add(cur)
            for n in neighbors4(cur):
                if not self.grid.is_free(n) and n != goal:
                    continue
                nd = d + 1
                if nd < dist.get(n, float('inf')):
                    dist[n] = nd
                    came[n] = cur
                    heapq.heappush(pq, (nd, n))
        return None

    def _aco(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        nodes = [(x,y) for x in range(self.grid.n) for y in range(self.grid.n) if self.grid.grid[y,x] == 0]
        if start not in nodes or goal not in nodes:
            return None
        pher = {n: 1.0 for n in nodes}
        best_path = None
        best_len = float('inf')
        for _ in range(self.config.ACO_ITERS):
            iter_paths = []
            for _ in range(self.config.ACO_ANTS):
                path = [start]
                visited = {start}
                cur = start
                for _step in range(self.grid.n * 4):
                    if cur == goal:
                        break
                    options = [nb for nb in neighbors4(cur) if self.grid.is_free(nb) and nb not in visited]
                    if not options:
                        break
                    probs = []
                    for opt in options:
                        tau = pher.get(opt,1.0) ** self.config.ALPHA
                        eta = (1.0 / (euclid(opt, goal) + 1e-6)) ** self.config.BETA
                        probs.append(tau * eta)
                    s = sum(probs)
                    if s == 0:
                        break
                    probs = [p/s for p in probs]
                    nxt = random.choices(options, weights=probs, k=1)[0]
                    path.append(nxt)
                    visited.add(nxt)
                    cur = nxt
                if path[-1] == goal:
                    iter_paths.append(path)
                    if len(path) < best_len:
                        best_len = len(path)
                        best_path = path
            # Evaporation
            for k in list(pher.keys()):
                pher[k] *= (1 - self.config.EVAP)
            # Deposition
            for p in iter_paths:
                deposit = self.config.Q / len(p)
                for node in p:
                    if node in pher:
                        pher[node] += deposit
        return best_path

# ----------------------------
# Visualizer (pygame)
# ----------------------------
class Visualizer:
    def __init__(self, config: Config):
        pygame.init()
        self.config = config
        w = config.GRID_SIZE * config.CELL_SIZE
        h = config.GRID_SIZE * config.CELL_SIZE + 70
        self.win = pygame.display.set_mode((w, h))
        pygame.display.set_caption("GPS Hybrid Simulator")
        self.font = pygame.font.SysFont(None, 18)
        self.font_small = pygame.font.SysFont(None, 14)
        self.clock = pygame.time.Clock()
        self.flash_state = False
        self.flash_timer = 0.0

    def _draw_grid_cells(self, grid: Grid):
        for y in range(grid.n):
            for x in range(grid.n):
                v = grid.grid[y,x]
                if v == 0:
                    color = Config.WHITE
                elif v == 1:
                    color = Config.OBSTACLE
                else:
                    color = Config.DYN_OBS
                pygame.draw.rect(self.win, color, (x*Config.CELL_SIZE, y*Config.CELL_SIZE, Config.CELL_SIZE-1, Config.CELL_SIZE-1))
        # grid lines
        for i in range(Config.GRID_SIZE+1):
            pygame.draw.line(self.win, Config.GRID_LINE, (i*Config.CELL_SIZE,0), (i*Config.CELL_SIZE, Config.GRID_SIZE*Config.CELL_SIZE))
            pygame.draw.line(self.win, Config.GRID_LINE, (0, i*Config.CELL_SIZE), (Config.GRID_SIZE*Config.CELL_SIZE, i*Config.CELL_SIZE))

    def draw_selection_phase(self, grid: Grid, start_cell: Optional[Cell], goal_cell: Optional[Cell]):
        self.win.fill(Config.WHITE)
        self._draw_grid_cells(grid)
        if start_cell:
            sx, sy = start_cell
            pygame.draw.rect(self.win, Config.START_GREEN, (sx*Config.CELL_SIZE, sy*Config.CELL_SIZE, Config.CELL_SIZE-1, Config.CELL_SIZE-1))
            gps = cell_to_gps(start_cell)
            text = self.font_small.render(f"S: {gps[0]:.6f}, {gps[1]:.6f}", True, Config.BLACK)
            self.win.blit(text, (sx*Config.CELL_SIZE, sy*Config.CELL_SIZE - 14))
        if goal_cell:
            gx, gy = goal_cell
            pygame.draw.rect(self.win, Config.GOAL_ORANGE, (gx*Config.CELL_SIZE, gy*Config.CELL_SIZE, Config.CELL_SIZE-1, Config.CELL_SIZE-1))
            gps = cell_to_gps(goal_cell)
            text = self.font_small.render(f"G: {gps[0]:.6f}, {gps[1]:.6f}", True, Config.BLACK)
            self.win.blit(text, (gx*Config.CELL_SIZE, gy*Config.CELL_SIZE - 14))
        # Instructions
        y_offset = Config.GRID_SIZE*Config.CELL_SIZE + 8
        txt = "Left click: START | Right click: GOAL | SPACE: Begin"
        self.win.blit(self.font.render(txt, True, Config.BLACK), (8, y_offset))
        pygame.display.flip()

    def draw_navigation_phase(self, grid: Grid, paths: Dict[str, Optional[List[Cell]]],
                              chosen_path: Optional[List[Cell]], robot: SimRobot,
                              best_algo: str, replanning: bool, algo_stats: Dict[str, Dict]):
        self.win.fill(Config.WHITE)
        self._draw_grid_cells(grid)

        # draw all algorithm paths translucent
        def draw_path_cells(path, color, width=2):
            if not path:
                return
            pts = [(x*Config.CELL_SIZE + Config.CELL_SIZE//2, y*Config.CELL_SIZE + Config.CELL_SIZE//2) for (x,y) in path]
            if len(pts) > 1:
                pygame.draw.lines(self.win, color, False, pts, width)

        # A*, Dijkstra, ACO colors
        draw_path_cells(paths.get("A*"), Config.PATH_A, width=2)
        draw_path_cells(paths.get("Dijkstra"), Config.PATH_D, width=2)
        draw_path_cells(paths.get("ACO"), Config.PATH_C, width=2)

        # chosen best path bold
        if chosen_path:
            draw_path_cells(chosen_path, Config.BEST_PATH, width=4)

        # draw start/goal
        sx, sy = robot.history[0]
        gx, gy = robot.goal
        pygame.draw.rect(self.win, Config.START_GREEN, (sx*Config.CELL_SIZE, sy*Config.CELL_SIZE, Config.CELL_SIZE-1, Config.CELL_SIZE-1))
        pygame.draw.rect(self.win, Config.GOAL_ORANGE, (gx*Config.CELL_SIZE, gy*Config.CELL_SIZE, Config.CELL_SIZE-1, Config.CELL_SIZE-1))

        # robot
        rx, ry = robot.pos
        pygame.draw.circle(self.win, Config.ROBOT, (rx*Config.CELL_SIZE + Config.CELL_SIZE//2, ry*Config.CELL_SIZE + Config.CELL_SIZE//2), Config.CELL_SIZE//2 - 2)

        # status bar
        bar_y = Config.GRID_SIZE*Config.CELL_SIZE + 8
        status_text = f"Best: {best_algo} | Robot: {robot.pos} -> Goal: {robot.goal}"
        self.win.blit(self.font.render(status_text, True, Config.BLACK), (8, bar_y))

        # algorithm stats (length / n/a)
        x_off = 8
        for name in ("A*","Dijkstra","ACO"):
            stat = algo_stats.get(name, {})
            length = stat.get("len", "n/a")
            t_ms = stat.get("time_ms", None)
            s = f"{name}: len={length}"
            if t_ms is not None:
                s += f" ({int(t_ms)}ms)"
            self.win.blit(self.font_small.render(s, True, Config.BLACK), (x_off, bar_y + 20))
            x_off += 180

        # replanning flash
        if replanning:
            now = time.time()
            if now - self.flash_timer > 0.35:
                self.flash_state = not self.flash_state
                self.flash_timer = now
            if self.flash_state:
                self.win.blit(self.font.render("REPLANNING...", True, (200, 20, 20)), (Config.GRID_SIZE*Config.CELL_SIZE - 150, bar_y))

        pygame.display.flip()

    def quit(self):
        pygame.quit()

# ----------------------------
# utilities: convert path -> commands
# ----------------------------
def get_commands_from_path(robot: SimRobot, path: List[Cell]) -> List[str]:
    """Return list of commands from robot.pos to follow path.
       Commands: 'F', 'L', 'R' (Left/Right then Forward), no explicit 'B' used.
    """
    if not path or len(path) < 2:
        return []
    dirs = []
    orient = robot.orient
    pos = robot.pos
    for nxt in path[1:]:
        dx = nxt[0] - pos[0]; dy = nxt[1] - pos[1]
        needed = -1
        if (dx,dy) == (1,0): needed = 0  # East
        elif (dx,dy) == (0,-1): needed = 1  # North
        elif (dx,dy) == (-1,0): needed = 2  # West
        elif (dx,dy) == (0,1): needed = 3   # South
        else:
            pos = nxt; continue
        diff = (needed - orient) % 4
        if diff == 0:
            dirs.append("F")
        elif diff == 1:
            dirs.extend(["L","F"]); orient = (orient + 1) % 4
        elif diff == 2:
            dirs.extend(["L","L","F"]); orient = (orient + 2) % 4
        elif diff == 3:
            dirs.extend(["R","F"]); orient = (orient - 1) % 4
        pos = nxt
    return dirs

# ----------------------------
# Navigator main app
# ----------------------------
class Navigator:
    def __init__(self, config: Config):
        self.config = config
        self.grid = Grid(config.GRID_SIZE, config.INITIAL_OBSTACLE_RATE, seed=42)
        self.visualizer = Visualizer(config)
        self.controller = RobotController(config.MOVE_URL, config.ULTRA_URL, config.STOP_URL, config.SPEED_URL, timeout=config.REQUEST_TIMEOUT)
        self.planner = PathPlanner(self.grid, config)
        self.robot: Optional[SimRobot] = None
        self.start_cell: Optional[Cell] = None
        self.goal_cell: Optional[Cell] = None
        self.running = True
        self.replanning_flag = False

    def run(self):
        try:
            if not self._run_selection_phase():
                logging.info("Selection cancelled.")
                return
            if self.robot:
                self._run_navigation_phase()
                self._run_end_screen()
        except KeyboardInterrupt:
            logging.info("Interrupted by user.")
        finally:
            try:
                self.controller.stop()
            except:
                pass
            self.visualizer.quit()
            logging.info("App closed.")

    def _run_selection_phase(self) -> bool:
        logging.info("Select START (left) and GOAL (right). Press SPACE to start.")
        selecting = True
        while selecting and self.running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False; selecting = False; return False
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    cx = mx // self.config.CELL_SIZE; cy = my // self.config.CELL_SIZE
                    if in_bounds((cx, cy)):
                        if ev.button == 1:
                            self.start_cell = (cx, cy); self.grid.clear(self.start_cell)
                            logging.info(f"Start = {self.start_cell}")
                        elif ev.button == 3:
                            self.goal_cell = (cx, cy); self.grid.clear(self.goal_cell)
                            logging.info(f"Goal = {self.goal_cell}")
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_SPACE and self.start_cell and self.goal_cell:
                        selecting = False; logging.info("Starting navigation.")
            self.visualizer.draw_selection_phase(self.grid, self.start_cell, self.goal_cell)
            self.visualizer.clock.tick(self.config.SELECTION_FPS)
        if self.start_cell and self.goal_cell:
            self.robot = SimRobot(self.grid, self.start_cell, self.goal_cell)
            return True
        return False

    def _run_navigation_phase(self):
        iter_count = 0
        replans = 0
        # initial planning
        paths, best_path, best_algo = self.planner.plan_best_path(self.robot.pos, self.robot.goal)
        algo_stats = {k: {"len": (len(v) if v else None), "time_ms": None} for k,v in paths.items()}
        chosen_path = best_path
        while self.running and self.robot.pos != self.robot.goal and iter_count < 10000:
            iter_count += 1
            # process quit
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False; break
            if not self.running:
                break

            # compute commands for chosen path
            if not chosen_path:
                logging.info("No path available. Stopping.")
                break
            commands = get_commands_from_path(self.robot, chosen_path)

            # execute step-by-step
            for cmd in commands:
                # allow quit
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        self.running = False; break
                if not self.running:
                    break

                # sense-before-move for forward steps
                if cmd == "F":
                    sensor = self.controller.get_sensor_data()
                    # adaptive speed: update ESP32 with current speed based on distance (0..255)
                    nearest = min(sensor["left"], sensor["right"])
                    if nearest >= 100:
                        recommended = 230
                    elif nearest >= 50:
                        recommended = int(150 + (nearest - 50) * (80/50))
                    elif nearest >= 20:
                        recommended = int(90 + (nearest - 20) * (60/30))
                    else:
                        recommended = 60
                    try:
                        self.controller.set_speed(recommended)
                    except:
                        pass

                    if sensor["obstacle"]:
                        logging.info("Obstacle detected in front -> stop + replan")
                        self.controller.move("S")
                        front = self.robot.front_cell()
                        # make sure front is valid before setting
                        if in_bounds(front) and self.grid.is_free(front):
                            self.grid.set_dynamic(front)
                        replans += 1
                        # synchronous replanning: DO NOT MOVE until planner returns
                        self.replanning_flag = True
                        self.visualizer.draw_navigation_phase(self.grid, paths, chosen_path, self.robot, best_algo, True, algo_stats)
                        # call planner
                        paths, new_best, new_algo = self.planner.plan_best_path(self.robot.pos, self.robot.goal)
                        algo_stats = {k: {"len": (len(v) if v else None), "time_ms": None} for k,v in paths.items()}
                        chosen_path = new_best
                        best_algo = new_algo
                        self.replanning_flag = False
                        logging.info(f"Replan done -> best {best_algo}")
                        # after replan, stop executing old commands; break to outer while to refresh commands from new path
                        break

                # send command
                ok = self.controller.move(cmd)
                # update simulated robot
                if cmd == "F":
                    self.robot.step_to(self.robot.front_cell())
                elif cmd == "L":
                    self.robot.turn_left()
                elif cmd == "R":
                    self.robot.turn_right()

                # draw (paths always visible)
                self.visualizer.draw_navigation_phase(self.grid, paths, chosen_path, self.robot, best_algo, False, algo_stats)

                # check goal reached
                if self.robot.pos == self.robot.goal:
                    logging.info("Goal reached!")
                    self.running = False
                    break

            # random dynamic obstacles (simulate environment changes)
            if random.random() < self.config.DYNAMIC_OBS_PROB:
                c = self.grid.random_free_cell()
                if c and c != self.robot.pos and c != self.robot.goal:
                    self.grid.set_dynamic(c)

            # if we exited the for-commands loop because of an obstacle (we broke),
            # continue outer loop to re-evaluate chosen_path (already updated synchronously)
            self.visualizer.clock.tick(self.config.FPS)

        # finish
        try:
            self.controller.move("S")
        except:
            pass
        logging.info(f"Finished: iters={iter_count} replans={replans}")
        logging.info(f"Final pos={self.robot.pos} goal={self.robot.goal}")

    def _run_end_screen(self):
        logging.info("Navigation finished. Close window to exit.")
        while True:
            quit_flag = False
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    quit_flag = True; break
            if quit_flag:
                break
            self.visualizer.clock.tick(10)

# ----------------------------
# ENTRY
# ----------------------------
if __name__ == "__main__":
    random.seed(int(time.time()))
    cfg = Config()
    app = Navigator(cfg)
    app.run()
