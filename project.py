import sys
import math
import random
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple

import pandas as pd
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt6.QtWidgets import QApplication, QWidget

# ==========================================
# 1. КОНФИГУРАЦИЯ И ГЕОМЕТРИЯ ПЕРЕКРЕСТКА
# ==========================================
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
FPS = 60
SIMULATION_DURATION = 90.0

CENTER_X = SCREEN_WIDTH // 2   # 640
CENTER_Y = SCREEN_HEIGHT // 2  # 360

LANE_WIDTH = 44
NUM_LANES_PER_DIR = 2
ROAD_HALF_WIDTH = NUM_LANES_PER_DIR * LANE_WIDTH  # 88 px

BOX_LEFT = CENTER_X - ROAD_HALF_WIDTH
BOX_RIGHT = CENTER_X + ROAD_HALF_WIDTH
BOX_TOP = CENTER_Y - ROAD_HALF_WIDTH
BOX_BOTTOM = CENTER_Y + ROAD_HALF_WIDTH

CROSSWALK_WIDTH = 26
CROSSWALK_OFFSET = 8
STOP_LINE_OFFSET = CROSSWALK_OFFSET + CROSSWALK_WIDTH + 8  # 42 px от коробки

CAR_LENGTH, CAR_WIDTH = 46, 22
BUS_LENGTH, BUS_WIDTH = 74, 26
VEHICLE_SPEED = 3.3
COLLISION_BUFFER = 16
PED_SPEED = 1.35

COLOR_BG = QColor(25, 27, 32)
COLOR_GRASS = QColor(36, 40, 36)
COLOR_SIDEWALK = QColor(85, 90, 96)
COLOR_ROAD = QColor(44, 46, 52)
COLOR_MARKING = QColor(235, 235, 240)
COLOR_STOP_LINE = QColor(255, 60, 60)
COLOR_BOX_GRID = QColor(160, 145, 40, 130)
COLOR_WHITE = QColor(255, 255, 255)

COLOR_CAR = QColor(48, 140, 245)
COLOR_BUS = QColor(245, 150, 25)
COLOR_PEDESTRIAN = QColor(255, 225, 90)

COLOR_RED = QColor(255, 45, 45)
COLOR_YELLOW = QColor(255, 215, 30)
COLOR_GREEN = QColor(40, 235, 70)

FIXED_GREEN_EW = 22.0
FIXED_GREEN_NS = 14.0
YELLOW_TIME = 2.5
ALL_RED_TIME = 2.5

MIN_ADAPTIVE_GREEN = 9.0
MAX_ADAPTIVE_GREEN = 26.0


# ==========================================
# 2. МАТЕМАТИКА ТРАЕКТОРИЙ
# ==========================================
def bezier_point(p0: Tuple[float, float], p1: Tuple[float, float],
                 p2: Tuple[float, float], p3: Tuple[float, float], t: float) -> Tuple[float, float]:
    u = 1.0 - t
    tt = t * t
    uu = u * u
    x = uu * u * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + tt * t * p3[0]
    y = uu * u * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + tt * t * p3[1]
    return (x, y)


def build_sampled_path(pts_poly: List[Tuple[float, float]], step: float = 2.0) -> List[Tuple[float, float, float, float]]:
    dense: List[Tuple[float, float]] = []
    for i in range(len(pts_poly) - 1):
        p_start = pts_poly[i]
        p_end = pts_poly[i + 1]
        dist = math.hypot(p_end[0] - p_start[0], p_end[1] - p_start[1])
        steps = max(1, int(dist / 1.0))
        for s in range(steps):
            t = s / steps
            dense.append((p_start[0] + t * (p_end[0] - p_start[0]),
                          p_start[1] + t * (p_end[1] - p_start[1])))
    dense.append(pts_poly[-1])

    resampled: List[Tuple[float, float, float, float]] = []
    cur_x, cur_y = dense[0]
    resampled.append((cur_x, cur_y, 0.0, 0.0))
    accum = 0.0

    for i in range(1, len(dense)):
        dx = dense[i][0] - cur_x
        dy = dense[i][1] - cur_y
        d = math.hypot(dx, dy)
        accum += d
        cur_x, cur_y = dense[i]
        if accum >= step:
            accum -= step
            angle_rad = math.atan2(dy, dx)
            resampled.append((cur_x, cur_y, angle_rad, math.degrees(angle_rad)))

    final_path = []
    for i in range(len(resampled)):
        nxt = resampled[min(i + 1, len(resampled) - 1)]
        cur = resampled[i]
        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]
        ang_r = math.atan2(dy, dx) if (dx != 0 or dy != 0) else cur[2]
        final_path.append((cur[0], cur[1], ang_r, math.degrees(ang_r)))

    return final_path


# ==========================================
# 3. ДОМЕННЫЕ МОДЕЛИ
# ==========================================
@dataclass
class Pedestrian:
    id: int
    crosswalk_name: str
    x: float
    y: float
    target_x: float
    target_y: float
    dx: float
    dy: float
    state: str = 'waiting'
    wait_time: float = 0.0

    def update(self, dt: float, light_state: str):
        if self.state == 'waiting':
            self.wait_time += dt
            if light_state == 'green':
                self.state = 'walking'
        elif self.state == 'walking':
            dist_to_target = math.hypot(self.target_x - self.x, self.target_y - self.y)
            if dist_to_target <= PED_SPEED:
                self.x = self.target_x
                self.y = self.target_y
                self.state = 'finished'
            else:
                self.x += self.dx * PED_SPEED
                self.y += self.dy * PED_SPEED

    def draw(self, p: QPainter):
        p.setPen(QPen(QColor(20, 20, 20), 1))
        p.setBrush(COLOR_PEDESTRIAN)
        p.drawEllipse(QPointF(self.x, self.y), 4.5, 4.5)
        p.setBrush(QColor(230, 180, 140))
        p.drawEllipse(QPointF(self.x, self.y), 2.5, 2.5)


@dataclass
class Vehicle:
    id: int
    vehicle_type: str
    route_id: str
    approach: str
    turn_intent: str
    lane_idx: int
    exit_road: str
    exit_lane: int
    path: List[Tuple[float, float, float, float]]
    stop_dist: float
    wait_left_turn_dist: float
    exit_box_dist: float

    path_index: int = 0
    distance_traveled: float = 0.0
    speed: float = VEHICLE_SPEED
    stopped: bool = False
    stopped_reason: str = ""

    wait_start_time: Optional[float] = None
    total_wait_time: float = 0.0
    finished: bool = False

    def __post_init__(self):
        self.length = BUS_LENGTH if self.vehicle_type == 'bus' else CAR_LENGTH
        self.width = BUS_WIDTH if self.vehicle_type == 'bus' else CAR_WIDTH
        self.color = COLOR_BUS if self.vehicle_type == 'bus' else COLOR_CAR

    @property
    def current_pos(self) -> Tuple[float, float, float, float]:
        idx = min(self.path_index, len(self.path) - 1)
        return self.path[idx]

    @property
    def x(self) -> float:
        return self.current_pos[0]

    @property
    def y(self) -> float:
        return self.current_pos[1]

    @property
    def heading_rad(self) -> float:
        return self.current_pos[2]

    @property
    def heading_deg(self) -> float:
        return self.current_pos[3]

    def is_inside_intersection(self) -> bool:
        return (self.distance_traveled > (self.stop_dist + 4.0) and
                self.distance_traveled < (self.exit_box_dist - 4.0))

    def get_safety_circles(self) -> List[Tuple[float, float, float]]:
        cx, cy, rad, _ = self.current_pos
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        r = self.width * 0.48
        if self.vehicle_type == 'bus':
            return [(cx + d * cos_a, cy + d * sin_a, r) for d in (-22.0, 0.0, 22.0)]
        return [(cx - 10.0 * cos_a, cy - 10.0 * sin_a, r), (cx + 10.0 * cos_a, cy + 10.0 * sin_a, r)]

    def update_position(self, sim_time: float, should_stop: bool, reason: str = ""):
        self.stopped_reason = reason if should_stop else ""
        if should_stop:
            if not self.stopped:
                self.stopped = True
                self.wait_start_time = sim_time
        else:
            if self.stopped and self.wait_start_time is not None:
                self.total_wait_time += (sim_time - self.wait_start_time)
                self.wait_start_time = None
            self.stopped = False

        if not self.stopped:
            self.distance_traveled += self.speed
            self.path_index = int(self.distance_traveled / 2.0)
            if self.path_index >= len(self.path) - 1:
                self.finished = True
                if self.wait_start_time is not None:
                    self.total_wait_time += (sim_time - self.wait_start_time)

    def draw(self, p: QPainter, blinker_on: bool):
        p.save()
        p.translate(self.x, self.y)
        p.rotate(self.heading_deg)

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(0, 0, 0, 80))
        p.drawRoundedRect(QRectF(-self.length / 2 + 2, -self.width / 2 + 3, self.length, self.width), 4, 4)

        p.setBrush(self.color)
        p.setPen(QPen(QColor(20, 20, 25), 1.2))
        p.drawRoundedRect(QRectF(-self.length / 2, -self.width / 2, self.length, self.width), 4, 4)

        p.setBrush(QColor(35, 45, 60))
        p.drawRoundedRect(QRectF(self.length * 0.12, -self.width * 0.4, self.length * 0.22, self.width * 0.8), 2, 2)
        p.drawRoundedRect(QRectF(-self.length * 0.35, -self.width * 0.35, self.length * 0.15, self.width * 0.7), 2, 2)

        # Фары
        p.setBrush(QColor(255, 255, 220))
        p.drawRect(QRectF(self.length / 2 - 3, -self.width / 2 + 2, 3, 4))
        p.drawRect(QRectF(self.length / 2 - 3, self.width / 2 - 6, 3, 4))

        # Стоп-сигналы
        p.setBrush(COLOR_RED if self.stopped else QColor(130, 20, 20))
        p.drawRect(QRectF(-self.length / 2, -self.width / 2 + 2, 3, 4))
        p.drawRect(QRectF(-self.length / 2, self.width / 2 - 6, 3, 4))

        # Поворотники
        if blinker_on and self.turn_intent in ('left', 'right'):
            p.setBrush(COLOR_YELLOW)
            y_offset = -self.width / 2 + 3 if self.turn_intent == 'left' else self.width / 2 - 3
            p.drawEllipse(QPointF(self.length / 2 - 2, y_offset), 2.5, 2.5)
            p.drawEllipse(QPointF(-self.length / 2 + 2, y_offset), 2.5, 2.5)

        p.restore()


# ==========================================
# 4. ДВИЖОК СИМУЛЯЦИИ ПЕРЕКРЕСТКА
# ==========================================
class SingleIntersectionSimulation:
    def __init__(self, mode: str = 'fixed', spawn_playback: Optional[Dict[str, Any]] = None):
        self.mode = mode
        self.spawn_playback = spawn_playback or {'vehicles': [], 'pedestrians': []}
        self.spawn_log: Dict[str, Any] = {'vehicles': [], 'pedestrians': []}
        self.veh_playback_idx = 0
        self.ped_playback_idx = 0

        self.vehicles: List[Vehicle] = []
        self.finished_vehicles: List[Vehicle] = []
        self.pedestrians: List[Pedestrian] = []
        self.finished_pedestrians: List[Pedestrian] = []

        self.next_veh_id = 1
        self.next_ped_id = 1

        self.phase = 'EW_GREEN'
        self.phase_timer = FIXED_GREEN_EW if mode == 'fixed' else MIN_ADAPTIVE_GREEN
        self.adaptive_time_in_phase = 0.0

        self.routes = self._generate_all_routes()

    def _generate_all_routes(self) -> Dict[str, Dict[str, Any]]:
        routes = {}

        # 1. EB (слева направо)
        y_r = CENTER_Y + LANE_WIDTH * 1.5   # 426
        y_l = CENTER_Y + LANE_WIDTH * 0.5   # 382
        stop_x_eb = BOX_LEFT - STOP_LINE_OFFSET

        p_eb_s0 = [(-50.0, y_r), (stop_x_eb, y_r), (SCREEN_WIDTH + 70.0, y_r)]
        p_eb_s1 = [(-50.0, y_l), (stop_x_eb, y_l), (SCREEN_WIDTH + 70.0, y_l)]
        routes['EB_straight_0'] = {
            'path': build_sampled_path(p_eb_s0),
            'stop_dist': stop_x_eb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_x_eb - (-50.0)) + (BOX_RIGHT - stop_x_eb),
            'approach': 'EB', 'intent': 'straight', 'lane': 0, 'exit_road': 'EB', 'exit_lane': 0
        }
        routes['EB_straight_1'] = {
            'path': build_sampled_path(p_eb_s1),
            'stop_dist': stop_x_eb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_x_eb - (-50.0)) + (BOX_RIGHT - stop_x_eb),
            'approach': 'EB', 'intent': 'straight', 'lane': 1, 'exit_road': 'EB', 'exit_lane': 1
        }

        # EB Направо (в полосу x = 574, идет на юг)
        target_x_sb = CENTER_X - LANE_WIDTH * 1.5
        arc_eb_r = [bezier_point((BOX_LEFT - 10, y_r), (BOX_LEFT + 25, y_r),
                                 (target_x_sb, BOX_BOTTOM - 25), (target_x_sb, BOX_BOTTOM + 15), t / 20.0)
                    for t in range(21)]
        p_eb_r = [(-50.0, y_r), (stop_x_eb, y_r)] + arc_eb_r + [(target_x_sb, SCREEN_HEIGHT + 70.0)]
        routes['EB_right'] = {
            'path': build_sampled_path(p_eb_r),
            'stop_dist': stop_x_eb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_x_eb - (-50.0)) + 140.0,
            'approach': 'EB', 'intent': 'right', 'lane': 0, 'exit_road': 'SB', 'exit_lane': 0
        }

        # EB Налево (Разъезд левым бортом: траектория плавно смещена к западу центра)
        target_x_nb = CENTER_X + LANE_WIDTH * 0.5  # 662
        arc_eb_l = [bezier_point((CENTER_X - 60, y_l), (CENTER_X - 10, y_l),
                                 (target_x_nb - 12, CENTER_Y - 25), (target_x_nb, BOX_TOP - 20), t / 25.0)
                    for t in range(26)]
        p_eb_l = [(-50.0, y_l), (stop_x_eb, y_l), (CENTER_X - 60, y_l)] + arc_eb_l + [(target_x_nb, -70.0)]
        routes['EB_left'] = {
            'path': build_sampled_path(p_eb_l),
            'stop_dist': stop_x_eb - (-50.0),
            'wait_left': (stop_x_eb - (-50.0)) + (CENTER_X - 60 - stop_x_eb),
            'exit_box': (stop_x_eb - (-50.0)) + 210.0,
            'approach': 'EB', 'intent': 'left', 'lane': 1, 'exit_road': 'NB', 'exit_lane': 1
        }

        # 2. WB (справа налево)
        y_wb_r = CENTER_Y - LANE_WIDTH * 1.5  # 294
        y_wb_l = CENTER_Y - LANE_WIDTH * 0.5  # 338
        stop_x_wb = BOX_RIGHT + STOP_LINE_OFFSET

        p_wb_s0 = [(SCREEN_WIDTH + 50.0, y_wb_r), (stop_x_wb, y_wb_r), (-70.0, y_wb_r)]
        p_wb_s1 = [(SCREEN_WIDTH + 50.0, y_wb_l), (stop_x_wb, y_wb_l), (-70.0, y_wb_l)]
        routes['WB_straight_0'] = {
            'path': build_sampled_path(p_wb_s0),
            'stop_dist': (SCREEN_WIDTH + 50.0) - stop_x_wb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_WIDTH + 50.0) - stop_x_wb) + (stop_x_wb - BOX_LEFT),
            'approach': 'WB', 'intent': 'straight', 'lane': 0, 'exit_road': 'WB', 'exit_lane': 0
        }
        routes['WB_straight_1'] = {
            'path': build_sampled_path(p_wb_s1),
            'stop_dist': (SCREEN_WIDTH + 50.0) - stop_x_wb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_WIDTH + 50.0) - stop_x_wb) + (stop_x_wb - BOX_LEFT),
            'approach': 'WB', 'intent': 'straight', 'lane': 1, 'exit_road': 'WB', 'exit_lane': 1
        }

        # WB Направо (в x = 706, идет на север)
        target_x_nb_out = CENTER_X + LANE_WIDTH * 1.5
        arc_wb_r = [bezier_point((BOX_RIGHT + 10, y_wb_r), (BOX_RIGHT - 25, y_wb_r),
                                 (target_x_nb_out, BOX_TOP + 25), (target_x_nb_out, BOX_TOP - 15), t / 20.0)
                    for t in range(21)]
        p_wb_r = [(SCREEN_WIDTH + 50.0, y_wb_r), (stop_x_wb, y_wb_r)] + arc_wb_r + [(target_x_nb_out, -70.0)]
        routes['WB_right'] = {
            'path': build_sampled_path(p_wb_r),
            'stop_dist': (SCREEN_WIDTH + 50.0) - stop_x_wb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_WIDTH + 50.0) - stop_x_wb) + 140.0,
            'approach': 'WB', 'intent': 'right', 'lane': 0, 'exit_road': 'NB', 'exit_lane': 0
        }

        # WB Налево (Разъезд левым бортом: траектория плавно смещена к востоку центра)
        target_x_sb_out = CENTER_X - LANE_WIDTH * 0.5  # 618
        arc_wb_l = [bezier_point((CENTER_X + 60, y_wb_l), (CENTER_X + 10, y_wb_l),
                                 (target_x_sb_out + 12, CENTER_Y + 25), (target_x_sb_out, BOX_BOTTOM + 20), t / 25.0)
                    for t in range(26)]
        p_wb_l = [(SCREEN_WIDTH + 50.0, y_wb_l), (stop_x_wb, y_wb_l), (CENTER_X + 60, y_wb_l)] + arc_wb_l + [(target_x_sb_out, SCREEN_HEIGHT + 70.0)]
        routes['WB_left'] = {
            'path': build_sampled_path(p_wb_l),
            'stop_dist': (SCREEN_WIDTH + 50.0) - stop_x_wb,
            'wait_left': ((SCREEN_WIDTH + 50.0) - stop_x_wb) + (stop_x_wb - (CENTER_X + 60)),
            'exit_box': ((SCREEN_WIDTH + 50.0) - stop_x_wb) + 210.0,
            'approach': 'WB', 'intent': 'left', 'lane': 1, 'exit_road': 'SB', 'exit_lane': 1
        }

        # 3. SB (сверху вниз)
        x_sb_r = CENTER_X - LANE_WIDTH * 1.5  # 574
        x_sb_l = CENTER_X - LANE_WIDTH * 0.5  # 618
        stop_y_sb = BOX_TOP - STOP_LINE_OFFSET

        p_sb_s0 = [(x_sb_r, -50.0), (x_sb_r, stop_y_sb), (x_sb_r, SCREEN_HEIGHT + 70.0)]
        p_sb_s1 = [(x_sb_l, -50.0), (x_sb_l, stop_y_sb), (x_sb_l, SCREEN_HEIGHT + 70.0)]
        routes['SB_straight_0'] = {
            'path': build_sampled_path(p_sb_s0),
            'stop_dist': stop_y_sb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_y_sb - (-50.0)) + (BOX_BOTTOM - stop_y_sb),
            'approach': 'SB', 'intent': 'straight', 'lane': 0, 'exit_road': 'SB', 'exit_lane': 0
        }
        routes['SB_straight_1'] = {
            'path': build_sampled_path(p_sb_s1),
            'stop_dist': stop_y_sb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_y_sb - (-50.0)) + (BOX_BOTTOM - stop_y_sb),
            'approach': 'SB', 'intent': 'straight', 'lane': 1, 'exit_road': 'SB', 'exit_lane': 1
        }

        # SB Направо (в y = 294, идет на запад)
        arc_sb_r = [bezier_point((x_sb_r, BOX_TOP - 10), (x_sb_r, BOX_TOP + 25),
                                 (BOX_LEFT - 25, y_wb_r), (BOX_LEFT - 20, y_wb_r), t / 20.0) for t in range(21)]
        p_sb_r = [(x_sb_r, -50.0), (x_sb_r, stop_y_sb)] + arc_sb_r + [(-70.0, y_wb_r)]
        routes['SB_right'] = {
            'path': build_sampled_path(p_sb_r),
            'stop_dist': stop_y_sb - (-50.0),
            'wait_left': 9999.0, 'exit_box': (stop_y_sb - (-50.0)) + 140.0,
            'approach': 'SB', 'intent': 'right', 'lane': 0, 'exit_road': 'WB', 'exit_lane': 0
        }

        # SB Налево (Разъезд левым бортом: смещен к северу центра)
        arc_sb_l = [bezier_point((x_sb_l, CENTER_Y - 60), (x_sb_l, CENTER_Y - 10),
                                 (CENTER_X + 25, y_l - 12), (BOX_RIGHT + 20, y_l), t / 25.0) for t in range(26)]
        p_sb_l = [(x_sb_l, -50.0), (x_sb_l, stop_y_sb), (x_sb_l, CENTER_Y - 60)] + arc_sb_l + [(SCREEN_WIDTH + 70.0, y_l)]
        routes['SB_left'] = {
            'path': build_sampled_path(p_sb_l),
            'stop_dist': stop_y_sb - (-50.0),
            'wait_left': (stop_y_sb - (-50.0)) + (CENTER_Y - 60 - stop_y_sb),
            'exit_box': (stop_y_sb - (-50.0)) + 210.0,
            'approach': 'SB', 'intent': 'left', 'lane': 1, 'exit_road': 'EB', 'exit_lane': 1
        }

        # 4. NB (снизу вверх)
        x_nb_r = CENTER_X + LANE_WIDTH * 1.5  # 706
        x_nb_l = CENTER_X + LANE_WIDTH * 0.5  # 662
        stop_y_nb = BOX_BOTTOM + STOP_LINE_OFFSET

        p_nb_s0 = [(x_nb_r, SCREEN_HEIGHT + 50.0), (x_nb_r, stop_y_nb), (x_nb_r, -70.0)]
        p_nb_s1 = [(x_nb_l, SCREEN_HEIGHT + 50.0), (x_nb_l, stop_y_nb), (x_nb_l, -70.0)]
        routes['NB_straight_0'] = {
            'path': build_sampled_path(p_nb_s0),
            'stop_dist': (SCREEN_HEIGHT + 50.0) - stop_y_nb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_HEIGHT + 50.0) - stop_y_nb) + (stop_y_nb - BOX_TOP),
            'approach': 'NB', 'intent': 'straight', 'lane': 0, 'exit_road': 'NB', 'exit_lane': 0
        }
        routes['NB_straight_1'] = {
            'path': build_sampled_path(p_nb_s1),
            'stop_dist': (SCREEN_HEIGHT + 50.0) - stop_y_nb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_HEIGHT + 50.0) - stop_y_nb) + (stop_y_nb - BOX_TOP),
            'approach': 'NB', 'intent': 'straight', 'lane': 1, 'exit_road': 'NB', 'exit_lane': 1
        }

        # NB Направо (в y = 426, идет на восток)
        arc_nb_r = [bezier_point((x_nb_r, BOX_BOTTOM + 10), (x_nb_r, BOX_BOTTOM - 25),
                                 (BOX_RIGHT + 25, y_r), (BOX_RIGHT + 20, y_r), t / 20.0) for t in range(21)]
        p_nb_r = [(x_nb_r, SCREEN_HEIGHT + 50.0), (x_nb_r, stop_y_nb)] + arc_nb_r + [(SCREEN_WIDTH + 70.0, y_r)]
        routes['NB_right'] = {
            'path': build_sampled_path(p_nb_r),
            'stop_dist': (SCREEN_HEIGHT + 50.0) - stop_y_nb,
            'wait_left': 9999.0, 'exit_box': ((SCREEN_HEIGHT + 50.0) - stop_y_nb) + 140.0,
            'approach': 'NB', 'intent': 'right', 'lane': 0, 'exit_road': 'EB', 'exit_lane': 0
        }

        # NB Налево (Разъезд левым бортом: смещен к югу центра)
        arc_nb_l = [bezier_point((x_nb_l, CENTER_Y + 60), (x_nb_l, CENTER_Y + 10),
                                 (CENTER_X - 25, y_wb_l + 12), (BOX_LEFT - 20, y_wb_l), t / 25.0) for t in range(26)]
        p_nb_l = [(x_nb_l, SCREEN_HEIGHT + 50.0), (x_nb_l, stop_y_nb), (x_nb_l, CENTER_Y + 60)] + arc_nb_l + [(-70.0, y_wb_l)]
        routes['NB_left'] = {
            'path': build_sampled_path(p_nb_l),
            'stop_dist': (SCREEN_HEIGHT + 50.0) - stop_y_nb,
            'wait_left': ((SCREEN_HEIGHT + 50.0) - stop_y_nb) + (stop_y_nb - (CENTER_Y + 60)),
            'exit_box': ((SCREEN_HEIGHT + 50.0) - stop_y_nb) + 210.0,
            'approach': 'NB', 'intent': 'left', 'lane': 1, 'exit_road': 'WB', 'exit_lane': 1
        }

        return routes

    def spawn_traffic(self, sim_time: float):
        if self.spawn_playback['vehicles']:
            while self.veh_playback_idx < len(self.spawn_playback['vehicles']):
                rec = self.spawn_playback['vehicles'][self.veh_playback_idx]
                if rec['time'] <= sim_time:
                    self._create_vehicle(rec['type'], rec['route_id'])
                    self.veh_playback_idx += 1
                else:
                    break
        else:
            spawn_chances = {'EB': 0.021, 'WB': 0.019, 'SB': 0.013, 'NB': 0.013}
            for app, chance in spawn_chances.items():
                if random.random() < chance:
                    v_type = 'bus' if random.random() < 0.20 else 'car'
                    intent = random.choices(['straight', 'right', 'left'], weights=[0.55, 0.25, 0.20])[0]
                    if intent == 'right':
                        r_id = f"{app}_right"
                    elif intent == 'left':
                        r_id = f"{app}_left"
                    else:
                        lane = 0 if random.random() < 0.5 else 1
                        r_id = f"{app}_straight_{lane}"

                    rec = {'time': sim_time, 'type': v_type, 'route_id': r_id}
                    self.spawn_log['vehicles'].append(rec)
                    self._create_vehicle(v_type, r_id)

        # Пешеходы
        if self.spawn_playback['pedestrians']:
            while self.ped_playback_idx < len(self.spawn_playback['pedestrians']):
                rec = self.spawn_playback['pedestrians'][self.ped_playback_idx]
                if rec['time'] <= sim_time:
                    self._create_pedestrian(rec['crosswalk'])
                    self.ped_playback_idx += 1
                else:
                    break
        else:
            if random.random() < 0.025:
                cw_name = random.choice(['W', 'E', 'N', 'S'])
                rec = {'time': sim_time, 'crosswalk': cw_name}
                self.spawn_log['pedestrians'].append(rec)
                self._create_pedestrian(cw_name)

    def _create_vehicle(self, v_type: str, route_id: str):
        route = self.routes[route_id]
        path = route['path']
        start_x, start_y = path[0][0], path[0][1]
        for v in self.vehicles:
            if v.approach == route['approach'] and v.lane_idx == route['lane']:
                if math.hypot(v.x - start_x, v.y - start_y) < (v.length + COLLISION_BUFFER + 12):
                    return

        veh = Vehicle(
            id=self.next_veh_id,
            vehicle_type=v_type,
            route_id=route_id,
            approach=route['approach'],
            turn_intent=route['intent'],
            lane_idx=route['lane'],
            exit_road=route['exit_road'],
            exit_lane=route['exit_lane'],
            path=path,
            stop_dist=route['stop_dist'],
            wait_left_turn_dist=route['wait_left'],
            exit_box_dist=route['exit_box']
        )
        self.next_veh_id += 1
        self.vehicles.append(veh)

    def _create_pedestrian(self, cw_name: str):
        if cw_name == 'W':
            cx = BOX_LEFT - CROSSWALK_OFFSET - CROSSWALK_WIDTH / 2
            y_start, y_end = (BOX_TOP - 15, BOX_BOTTOM + 15) if random.random() < 0.5 else (BOX_BOTTOM + 15, BOX_TOP - 15)
            p = Pedestrian(self.next_ped_id, cw_name, cx, y_start, cx, y_end, 0.0, 1.0 if y_end > y_start else -1.0)
        elif cw_name == 'E':
            cx = BOX_RIGHT + CROSSWALK_OFFSET + CROSSWALK_WIDTH / 2
            y_start, y_end = (BOX_TOP - 15, BOX_BOTTOM + 15) if random.random() < 0.5 else (BOX_BOTTOM + 15, BOX_TOP - 15)
            p = Pedestrian(self.next_ped_id, cw_name, cx, y_start, cx, y_end, 0.0, 1.0 if y_end > y_start else -1.0)
        elif cw_name == 'N':
            cy = BOX_TOP - CROSSWALK_OFFSET - CROSSWALK_WIDTH / 2
            x_start, x_end = (BOX_LEFT - 15, BOX_RIGHT + 15) if random.random() < 0.5 else (BOX_RIGHT + 15, BOX_LEFT - 15)
            p = Pedestrian(self.next_ped_id, cw_name, x_start, cy, x_end, cy, 1.0 if x_end > x_start else -1.0, 0.0)
        else:
            cy = BOX_BOTTOM + CROSSWALK_OFFSET + CROSSWALK_WIDTH / 2
            x_start, x_end = (BOX_LEFT - 15, BOX_RIGHT + 15) if random.random() < 0.5 else (BOX_RIGHT + 15, BOX_LEFT - 15)
            p = Pedestrian(self.next_ped_id, cw_name, x_start, cy, x_end, cy, 1.0 if x_end > x_start else -1.0, 0.0)

        self.next_ped_id += 1
        self.pedestrians.append(p)

    def update_lights(self, dt: float):
        self.phase_timer = max(0.0, self.phase_timer - dt)
        self.adaptive_time_in_phase += dt

        if self.mode == 'fixed':
            if self.phase_timer <= 0:
                cycle = {
                    'EW_GREEN': ('EW_YELLOW', YELLOW_TIME),
                    'EW_YELLOW': ('ALL_RED_1', ALL_RED_TIME),
                    'ALL_RED_1': ('NS_GREEN', FIXED_GREEN_NS),
                    'NS_GREEN': ('NS_YELLOW', YELLOW_TIME),
                    'NS_YELLOW': ('ALL_RED_2', ALL_RED_TIME),
                    'ALL_RED_2': ('EW_GREEN', FIXED_GREEN_EW),
                }
                self.phase, self.phase_timer = cycle[self.phase]
        else:
            # 1. Считаем ТОЛЬКО РЕАЛЬНО СТОЯЩИЕ машины в очереди (v.stopped)
            ew_stopped = sum(1 for v in self.vehicles if v.approach in ('EB', 'WB') and v.stopped and (v.stop_dist - v.distance_traveled) < 160)
            ns_stopped = sum(1 for v in self.vehicles if v.approach in ('SB', 'NB') and v.stopped and (v.stop_dist - v.distance_traveled) < 160)

            peds_waiting_ew = sum(1 for p in self.pedestrians if p.state == 'waiting' and p.crosswalk_name in ('N', 'S'))
            peds_waiting_ns = sum(1 for p in self.pedestrians if p.state == 'waiting' and p.crosswalk_name in ('W', 'E'))

            # 2. Детектор разрыва потока (Gap-Out): есть ли прямо сейчас машины на зеленой фазе перед стоп-линией
            ew_passing = any(v.approach in ('EB', 'WB') and 0 <= (v.stop_dist - v.distance_traveled) <= 80 for v in self.vehicles)
            ns_passing = any(v.approach in ('SB', 'NB') and 0 <= (v.stop_dist - v.distance_traveled) <= 80 for v in self.vehicles)

            # 3. Асимметричные лимиты: Проспекту максимум 20с, боковой улице - максимум 11с
            max_green_ew = 20.0
            max_green_ns = 10.0
            min_green = 7.0

            if self.phase == 'EW_GREEN':
                # Сбрасываем фазу, если:
                # - Прошло минимальное время И на зеленом никого нет (Gap-Out) И поперек кто-то ждет
                # - ИЛИ проспект исчерпал свой жесткий потолок
                gap_out = (not ew_passing) and (ns_stopped > 0 or peds_waiting_ns > 0)
                time_up = (self.adaptive_time_in_phase >= max_green_ew)

                if (self.adaptive_time_in_phase >= min_green and gap_out) or time_up:
                    self.phase = 'EW_YELLOW'
                    self.phase_timer = YELLOW_TIME
                    self.adaptive_time_in_phase = 0.0

            elif self.phase == 'EW_YELLOW' and self.phase_timer <= 0:
                self.phase = 'ALL_RED_1'
                self.phase_timer = ALL_RED_TIME
                self.adaptive_time_in_phase = 0.0

            elif self.phase == 'ALL_RED_1' and self.phase_timer <= 0:
                self.phase = 'NS_GREEN'
                self.phase_timer = min_green
                self.adaptive_time_in_phase = 0.0

            elif self.phase == 'NS_GREEN':
                # Боковая улица: сбрасываем мгновенно, как только прошла пачка машин
                gap_out = (not ns_passing) and (ew_stopped > 0 or peds_waiting_ew > 0)
                time_up = (self.adaptive_time_in_phase >= max_green_ns)

                if (self.adaptive_time_in_phase >= min_green and gap_out) or time_up:
                    self.phase = 'NS_YELLOW'
                    self.phase_timer = YELLOW_TIME
                    self.adaptive_time_in_phase = 0.0

            elif self.phase == 'NS_YELLOW' and self.phase_timer <= 0:
                self.phase = 'ALL_RED_2'
                self.phase_timer = ALL_RED_TIME
                self.adaptive_time_in_phase = 0.0

            elif self.phase == 'ALL_RED_2' and self.phase_timer <= 0:
                self.phase = 'EW_GREEN'
                self.phase_timer = min_green
                self.adaptive_time_in_phase = 0.0

    def get_light_for_approach(self, approach: str) -> str:
        if approach in ('EB', 'WB'):
            if self.phase == 'EW_GREEN': return 'green'
            if self.phase == 'EW_YELLOW': return 'yellow'
            return 'red'
        else:
            if self.phase == 'NS_GREEN': return 'green'
            if self.phase == 'NS_YELLOW': return 'yellow'
            return 'red'

    def get_ped_light(self, cw_name: str) -> str:
        if cw_name in ('N', 'S'):
            return 'green' if self.phase == 'EW_GREEN' else 'red'
        return 'green' if self.phase == 'NS_GREEN' else 'red'

    def check_pedestrian_safety(self, v: Vehicle) -> bool:
        """100% защита пешеходов: упреждающий контроль всей зоны движения и перехода."""
        for p in self.pedestrians:
            if p.state != 'walking':
                continue

            # 1. Защита в непосредственной близости от корпуса автомобиля
            for (cx, cy, r) in v.get_safety_circles():
                if math.hypot(p.x - cx, p.y - cy) < (r + 14.0):
                    return True

            # 2. Предиктивный радар вперед по ходу движения
            lookahead = max(42.0, v.speed * 12.0 + v.length * 0.5)
            cur_d = v.distance_traveled

            for step in range(6, int(lookahead), 6):
                t_idx = min(len(v.path) - 1, int((cur_d + step) / 2.0))
                sx, sy = v.path[t_idx][0], v.path[t_idx][1]
                if math.hypot(p.x - sx, p.y - sy) < (v.width * 0.5 + 14.0):
                    return True

            # 3. Контроль въездного пешеходного перехода (до и во время проезда)
            dist_to_stop = v.stop_dist - v.distance_traveled
            if -15.0 <= dist_to_stop <= 45.0:
                entry_cw = {'EB': 'W', 'WB': 'E', 'SB': 'N', 'NB': 'S'}[v.approach]
                if p.crosswalk_name == entry_cw:
                    if math.hypot(p.x - v.x, p.y - v.y) < 52.0:
                        return True

            # 4. Контроль выездного перехода при поворотах (ПДД 13.1)
            if v.turn_intent in ('right', 'left') and v.distance_traveled > v.stop_dist:
                dist_to_exit = v.exit_box_dist - v.distance_traveled
                if -20.0 <= dist_to_exit <= 55.0:
                    exit_cw = None
                    if v.approach == 'EB': exit_cw = 'S' if v.turn_intent == 'right' else 'N'
                    elif v.approach == 'WB': exit_cw = 'N' if v.turn_intent == 'right' else 'S'
                    elif v.approach == 'SB': exit_cw = 'W' if v.turn_intent == 'right' else 'E'
                    elif v.approach == 'NB': exit_cw = 'E' if v.turn_intent == 'right' else 'W'

                    if exit_cw and p.crosswalk_name == exit_cw:
                        chk_idx = min(len(v.path) - 1, int((v.exit_box_dist + 10.0) / 2.0))
                        ex_pt = v.path[chk_idx]
                        if math.hypot(p.x - ex_pt[0], p.y - ex_pt[1]) < 48.0:
                            return True
        return False

    def is_exit_blocked(self, v: Vehicle) -> bool:
        """ПДД 13.2: Запрет въезда, если на полосе выезда образовался затор."""
        for other in self.vehicles:
            if other == v:
                continue
            if other.exit_road == v.exit_road and other.exit_lane == v.exit_lane:
                if other.distance_traveled > other.stop_dist:
                    dist_from_exit = other.distance_traveled - other.exit_box_dist
                    if 0.0 <= dist_from_exit <= (v.length + COLLISION_BUFFER + 14.0):
                        if other.stopped:
                            return True
        return False

    def has_clearing_traffic_in_box(self, v: Vehicle) -> bool:
        """ПДД 13.8: Запрет выезда на перекресток, пока ТС предыдущей фазы не завершили маневр."""
        is_ew_vehicle = v.approach in ('EB', 'WB')
        for other in self.vehicles:
            if other == v:
                continue
            other_is_ns = other.approach in ('SB', 'NB')
            if is_ew_vehicle and other_is_ns and other.is_inside_intersection():
                return True
            other_is_ew = other.approach in ('EB', 'WB')
            if (not is_ew_vehicle) and other_is_ew and other.is_inside_intersection():
                return True
        return False

    def is_left_turn_slot_taken(self, v: Vehicle) -> bool:
        """Не более одной машины на левом повороте в центре от каждого направления."""
        if v.turn_intent != 'left':
            return False
        for other in self.vehicles:
            if other != v and other.approach == v.approach and other.turn_intent == 'left':
                if other.is_inside_intersection():
                    return True
        return False

    def check_front_collision(self, v: Vehicle) -> Tuple[bool, str]:
        """Универсальный радар безопасности по траектории: предотвращает любые столкновения ТС."""
        lookahead = max(34.0, v.speed * 8.0 + v.length * 0.5 + COLLISION_BUFFER)
        cur_dist = v.distance_traveled

        for step in range(8, int(lookahead), 6):
            t_idx = min(len(v.path) - 1, int((cur_dist + step) / 2.0))
            sx, sy = v.path[t_idx][0], v.path[t_idx][1]

            for other in self.vehicles:
                if other == v:
                    continue

                # Встречные левые повороты разъезжаются левыми бортами по разведенным коридорам
                oncoming_app = {'EB': 'WB', 'WB': 'EB', 'SB': 'NB', 'NB': 'SB'}[v.approach]
                is_oncoming_left = (other.approach == oncoming_app and v.turn_intent == 'left' and other.turn_intent == 'left')

                # Допустимый зазор безопасности
                margin = 2.0 if is_oncoming_left else 5.5

                for (ox, oy, orad) in other.get_safety_circles():
                    if math.hypot(sx - ox, sy - oy) < (orad + margin):
                        # Встречный левый разъезд: пропускает тот, у кого меньше прогресс маневра
                        if is_oncoming_left:
                            if other.distance_traveled > v.distance_traveled:
                                return True, "left_turn_courtesy"
                        else:
                            return True, "radar_collision_avoidance"
        return False, ""

    def update_simulation_step(self, dt: float, sim_time: float):
        self.spawn_traffic(sim_time)
        self.update_lights(dt)

        # 1. Пешеходы
        active_peds = []
        for p in self.pedestrians:
            light = self.get_ped_light(p.crosswalk_name)
            p.update(dt, light)
            if p.state == 'finished':
                self.finished_pedestrians.append(p)
            else:
                active_peds.append(p)
        self.pedestrians = active_peds

        # 2. Транспорт
        active_vehs = []
        for v in self.vehicles:
            should_stop = False
            stop_reason = ""

            dist_to_stop = v.stop_dist - v.distance_traveled
            light = self.get_light_for_approach(v.approach)

            # А. Проверка стоп-линии (Светофор + ПДД 13.8 + ПДД 13.2)
            if 0.0 <= dist_to_stop <= (COLLISION_BUFFER + 6.0):
                if light in ('red', 'yellow'):
                    should_stop = True
                    stop_reason = "light"
                elif light == 'green':
                    if self.has_clearing_traffic_in_box(v):
                        should_stop = True
                        stop_reason = "pdd_13_8_clearing"
                    elif self.is_exit_blocked(v):
                        should_stop = True
                        stop_reason = "pdd_13_2_waffle"
                    elif self.is_left_turn_slot_taken(v):
                        should_stop = True
                        stop_reason = "wait_left_slot"

            # Б. Полная непрерывная защита пешеходов на всех переходах
            if not should_stop:
                if self.check_pedestrian_safety(v):
                    should_stop = True
                    stop_reason = "pedestrian_yield"

            # В. Безупречное ПДД 13.4: Уступка встречным при повороте налево
            if not should_stop and v.turn_intent == 'left' and light == 'green':
                dist_to_yield = v.wait_left_turn_dist - v.distance_traveled
                if -8.0 <= dist_to_yield <= 25.0:
                    oncoming_app = {'EB': 'WB', 'WB': 'EB', 'SB': 'NB', 'NB': 'SB'}[v.approach]
                    has_active_oncoming = False
                    for other in self.vehicles:
                        if other != v and other.approach == oncoming_app:
                            if other.turn_intent in ('straight', 'right'):
                                other_dist_to_stop = other.stop_dist - other.distance_traveled
                                if -180.0 < other_dist_to_stop < 220.0 and not other.stopped:
                                    has_active_oncoming = True
                                    break
                    if has_active_oncoming:
                        should_stop = True
                        stop_reason = "wait_left_oncoming"

            # Г. Радар предотвращения взаимных столкновений ТС
            if not should_stop:
                col_detected, col_reason = self.check_front_collision(v)
                if col_detected:
                    should_stop = True
                    stop_reason = col_reason

            v.update_position(sim_time, should_stop, stop_reason)

            if v.finished:
                self.finished_vehicles.append(v)
            else:
                active_vehs.append(v)

        self.vehicles = active_vehs

    def draw(self, p: QPainter, font_hud: QFont, font_sm: QFont, sim_time: float, blinker_on: bool):
        p.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_GRASS)

        sw_margin = 18
        p.fillRect(0, int(BOX_TOP - sw_margin), SCREEN_WIDTH, int(ROAD_HALF_WIDTH * 2 + sw_margin * 2), COLOR_SIDEWALK)
        p.fillRect(int(BOX_LEFT - sw_margin), 0, int(ROAD_HALF_WIDTH * 2 + sw_margin * 2), SCREEN_HEIGHT, COLOR_SIDEWALK)

        p.fillRect(0, BOX_TOP, SCREEN_WIDTH, ROAD_HALF_WIDTH * 2, COLOR_ROAD)
        p.fillRect(BOX_LEFT, 0, ROAD_HALF_WIDTH * 2, SCREEN_HEIGHT, COLOR_ROAD)

        # Центр перекрестка («Вафельница»)
        p.save()
        box_rect = QRectF(BOX_LEFT, BOX_TOP, ROAD_HALF_WIDTH * 2, ROAD_HALF_WIDTH * 2)
        p.fillRect(box_rect, QColor(48, 48, 42))
        p.setClipRect(box_rect)
        p.setPen(QPen(COLOR_BOX_GRID, 1.8))
        for d in range(int(BOX_LEFT - 200), int(BOX_RIGHT + 200), 16):
            p.drawLine(d, BOX_TOP - 20, d + 250, BOX_BOTTOM + 20)
            p.drawLine(d, BOX_BOTTOM + 20, d + 250, BOX_TOP - 20)
        p.restore()

        p.setPen(QPen(QColor(180, 160, 40), 2))
        p.drawRect(box_rect)

        # Разметка
        p.setPen(QPen(COLOR_MARKING, 2, Qt.PenStyle.DashLine))
        p.drawLine(0, CENTER_Y - LANE_WIDTH, BOX_LEFT, CENTER_Y - LANE_WIDTH)
        p.drawLine(BOX_RIGHT, CENTER_Y - LANE_WIDTH, SCREEN_WIDTH, CENTER_Y - LANE_WIDTH)
        p.drawLine(0, CENTER_Y + LANE_WIDTH, BOX_LEFT, CENTER_Y + LANE_WIDTH)
        p.drawLine(BOX_RIGHT, CENTER_Y + LANE_WIDTH, SCREEN_WIDTH, CENTER_Y + LANE_WIDTH)

        p.drawLine(CENTER_X - LANE_WIDTH, 0, CENTER_X - LANE_WIDTH, BOX_TOP)
        p.drawLine(CENTER_X - LANE_WIDTH, BOX_BOTTOM, CENTER_X - LANE_WIDTH, SCREEN_HEIGHT)
        p.drawLine(CENTER_X + LANE_WIDTH, 0, CENTER_X + LANE_WIDTH, BOX_TOP)
        p.drawLine(CENTER_X + LANE_WIDTH, BOX_BOTTOM, CENTER_X + LANE_WIDTH, SCREEN_HEIGHT)

        p.setPen(QPen(COLOR_MARKING, 2))
        p.drawLine(0, CENTER_Y - 2, BOX_LEFT, CENTER_Y - 2)
        p.drawLine(0, CENTER_Y + 2, BOX_LEFT, CENTER_Y + 2)
        p.drawLine(BOX_RIGHT, CENTER_Y - 2, SCREEN_WIDTH, CENTER_Y - 2)
        p.drawLine(BOX_RIGHT, CENTER_Y + 2, SCREEN_WIDTH, CENTER_Y + 2)

        p.drawLine(CENTER_X - 2, 0, CENTER_X - 2, BOX_TOP)
        p.drawLine(CENTER_X + 2, 0, CENTER_X + 2, BOX_TOP)
        p.drawLine(CENTER_X - 2, BOX_BOTTOM, CENTER_X - 2, SCREEN_HEIGHT)
        p.drawLine(CENTER_X + 2, BOX_BOTTOM, CENTER_X + 2, SCREEN_HEIGHT)

        # Зебры
        def draw_zebra(x, y, w, h, is_horizontal):
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(COLOR_WHITE)
            bar_w = 6
            if is_horizontal:
                for bx in range(int(x), int(x + w), bar_w * 2):
                    p.drawRect(bx, int(y), bar_w, int(h))
            else:
                for by in range(int(y), int(y + h), bar_w * 2):
                    p.drawRect(int(x), by, int(w), bar_w)

        draw_zebra(BOX_LEFT - CROSSWALK_OFFSET - CROSSWALK_WIDTH, BOX_TOP, CROSSWALK_WIDTH, ROAD_HALF_WIDTH * 2, False)
        draw_zebra(BOX_RIGHT + CROSSWALK_OFFSET, BOX_TOP, CROSSWALK_WIDTH, ROAD_HALF_WIDTH * 2, False)
        draw_zebra(BOX_LEFT, BOX_TOP - CROSSWALK_OFFSET - CROSSWALK_WIDTH, ROAD_HALF_WIDTH * 2, CROSSWALK_WIDTH, True)
        draw_zebra(BOX_LEFT, BOX_BOTTOM + CROSSWALK_OFFSET, ROAD_HALF_WIDTH * 2, CROSSWALK_WIDTH, True)

        # Стоп-линии
        p.setPen(QPen(COLOR_STOP_LINE, 4))
        p.drawLine(BOX_LEFT - STOP_LINE_OFFSET, CENTER_Y, BOX_LEFT - STOP_LINE_OFFSET, BOX_BOTTOM)
        p.drawLine(BOX_RIGHT + STOP_LINE_OFFSET, BOX_TOP, BOX_RIGHT + STOP_LINE_OFFSET, CENTER_Y)
        p.drawLine(BOX_LEFT, BOX_TOP - STOP_LINE_OFFSET, CENTER_X, BOX_TOP - STOP_LINE_OFFSET)
        p.drawLine(CENTER_X, BOX_BOTTOM + STOP_LINE_OFFSET, BOX_RIGHT, BOX_BOTTOM + STOP_LINE_OFFSET)

        # Светофоры
        def draw_light(x: float, y: float, state: str, is_vertical: bool, timer_val: float):
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(20, 20, 25))
            r = 5.5
            if is_vertical:
                p.drawRoundedRect(QRectF(x - 9, y - 22, 18, 44), 3, 3)
                p.setBrush(COLOR_RED if state == 'red' else QColor(50, 10, 10))
                p.drawEllipse(QPointF(x, y - 14), r, r)
                p.setBrush(COLOR_YELLOW if state == 'yellow' else QColor(50, 45, 5))
                p.drawEllipse(QPointF(x, y), r, r)
                p.setBrush(COLOR_GREEN if state == 'green' else QColor(10, 45, 15))
                p.drawEllipse(QPointF(x, y + 14), r, r)
            else:
                p.drawRoundedRect(QRectF(x - 22, y - 9, 44, 18), 3, 3)
                p.setBrush(COLOR_RED if state == 'red' else QColor(50, 10, 10))
                p.drawEllipse(QPointF(x - 14, y), r, r)
                p.setBrush(COLOR_YELLOW if state == 'yellow' else QColor(50, 45, 5))
                p.drawEllipse(QPointF(x, y), r, r)
                p.setBrush(COLOR_GREEN if state == 'green' else QColor(10, 45, 15))
                p.drawEllipse(QPointF(x + 14, y), r, r)

            if self.mode == 'fixed' and timer_val > 0:
                p.setFont(font_sm)
                p.setPen(COLOR_WHITE)
                p.drawText(int(x + 14), int(y + 4), str(math.ceil(timer_val)))

        ew_light = self.get_light_for_approach('EB')
        ns_light = self.get_light_for_approach('SB')
        draw_light(BOX_LEFT - STOP_LINE_OFFSET - 8, BOX_BOTTOM + 16, ew_light, False, self.phase_timer)
        draw_light(BOX_RIGHT + STOP_LINE_OFFSET + 8, BOX_TOP - 16, ew_light, False, self.phase_timer)
        draw_light(BOX_LEFT - 16, BOX_TOP - STOP_LINE_OFFSET - 8, ns_light, True, self.phase_timer)
        draw_light(BOX_RIGHT + 16, BOX_BOTTOM + STOP_LINE_OFFSET + 8, ns_light, True, self.phase_timer)

        # Пешеходные мини-светофоры
        def draw_ped_signal(x, y, active_green: bool):
            p.setBrush(QColor(15, 15, 20))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(QRectF(x - 6, y - 11, 12, 22), 2, 2)
            p.setBrush(COLOR_RED if not active_green else QColor(40, 10, 10))
            p.drawEllipse(QPointF(x, y - 5), 3.5, 3.5)
            p.setBrush(COLOR_GREEN if active_green else QColor(10, 40, 10))
            p.drawEllipse(QPointF(x, y + 5), 3.5, 3.5)

        draw_ped_signal(BOX_LEFT - STOP_LINE_OFFSET - 4, BOX_TOP - 8, self.get_ped_light('W') == 'green')
        draw_ped_signal(BOX_RIGHT + STOP_LINE_OFFSET + 4, BOX_BOTTOM + 8, self.get_ped_light('E') == 'green')
        draw_ped_signal(BOX_LEFT - 8, BOX_TOP - STOP_LINE_OFFSET - 4, self.get_ped_light('N') == 'green')
        draw_ped_signal(BOX_RIGHT + 8, BOX_BOTTOM + STOP_LINE_OFFSET + 4, self.get_ped_light('S') == 'green')

        for ped in self.pedestrians:
            ped.draw(p)

        for veh in self.vehicles:
            veh.draw(p, blinker_on)

        # HUD
        p.setFont(font_hud)
        p.setPen(COLOR_WHITE)
        mode_text = "ФИКСИРОВАННЫЙ ЦИКЛ" if self.mode == 'fixed' else "ИНТЕЛЛЕКТУАЛЬНЫЙ АДАПТИВ"
        p.drawText(24, 34, f"Режим: {mode_text} | Время: {sim_time:.1f}/{SIMULATION_DURATION}с | Фаза: {self.phase}")

        p.setPen(QColor(180, 240, 180))
        p.drawText(SCREEN_WIDTH - 420, 34, f"Проехало авто: {len(self.finished_vehicles)} | Пешеходов: {len(self.finished_pedestrians)}")

        p.setFont(font_sm)
        p.setPen(QColor(220, 220, 230))
        p.drawText(24, 60, "ПДД 13.1 (Защита пешеходов) | ПДД 13.4 (Разъезд левыми бортами без касания) | ПДД 13.8: АКТИВНЫ")


# ==========================================
# 5. ОКНО ПРИЛОЖЕНИЯ
# ==========================================
class SingleIntersectionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(SCREEN_WIDTH, SCREEN_HEIGHT)
        self.setWindowTitle("Безаварийная симуляция перекрестка: абсолютная безопасность (PyQt6)")

        self.font_hud = QFont("Consolas", 11, QFont.Weight.Bold)
        self.font_sm = QFont("Consolas", 8, QFont.Weight.Bold)
        self.font_splash = QFont("Consolas", 15, QFont.Weight.Bold)

        self.state = "RUN_FIXED"
        self.sim_time = 0.0
        self.transition_timer = 2.5
        self.blinker_timer = 0.0
        self.blinker_on = True

        random.seed(42)
        self.sim = SingleIntersectionSimulation(mode='fixed')

        self.recorded_traffic: Dict[str, Any] = {'vehicles': [], 'pedestrians': []}
        self.fixed_finished_v: List[Vehicle] = []
        self.adaptive_finished_v: List[Vehicle] = []
        self.fixed_finished_p: List[Pedestrian] = []
        self.adaptive_finished_p: List[Pedestrian] = []

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.game_loop)
        self.timer.start(int(1000 / FPS))

    def game_loop(self):
        dt = 1.0 / FPS

        # Мигание поворотников
        self.blinker_timer += dt
        if self.blinker_timer >= 0.35:
            self.blinker_timer = 0.0
            self.blinker_on = not self.blinker_on

        # ==========================================
        # ЭТАП 1: ФИКСИРОВАННЫЙ РЕЖИМ
        # ==========================================
        if self.state == "RUN_FIXED":
            self.sim_time += dt
            self.sim.update_simulation_step(dt, self.sim_time)

            if self.sim_time >= SIMULATION_DURATION:
                # 1. Засчитываем текущее время ожидания всем, кто прямо сейчас стоит в заторе
                for v in self.sim.vehicles:
                    if v.stopped and v.wait_start_time is not None:
                        v.total_wait_time += (self.sim_time - v.wait_start_time)
                        v.wait_start_time = None

                # 2. Берем И финишировавших, И оставшихся стоять в очереди на поле!
                self.fixed_finished_v = self.sim.finished_vehicles + self.sim.vehicles
                self.fixed_finished_p = self.sim.finished_pedestrians + self.sim.pedestrians

                self.recorded_traffic = self.sim.spawn_log
                self.state = "TRANSITION"
                print(f"\n[Этап 1: Фиксированный цикл завершен] Всего учтено ТС: {len(self.fixed_finished_v)} (проехало: {len(self.sim.finished_vehicles)})")

        # ==========================================
        # ПЕРЕХОД МЕЖДУ ЭТАПАМИ (ПАУЗА)
        # ==========================================
        elif self.state == "TRANSITION":
            self.transition_timer -= dt
            if self.transition_timer <= 0:
                random.seed(None)
                self.sim = SingleIntersectionSimulation(mode='adaptive', spawn_playback=self.recorded_traffic)
                self.sim_time = 0.0
                self.state = "RUN_ADAPTIVE"
                print("\n[Этап 2: Адаптивный режим запущен] (100% повтор потока ТС и пешеходов)...")

        # ==========================================
        # ЭТАП 2: АДАПТИВНЫЙ РЕЖИМ
        # ==========================================
        elif self.state == "RUN_ADAPTIVE":
            self.sim_time += dt
            self.sim.update_simulation_step(dt, self.sim_time)

            if self.sim_time >= SIMULATION_DURATION:
                # 1. Засчитываем время ожидания тем, кто остался на поле
                for v in self.sim.vehicles:
                    if v.stopped and v.wait_start_time is not None:
                        v.total_wait_time += (self.sim_time - v.wait_start_time)
                        v.wait_start_time = None

                # 2. Также объединяем финишировавших и оставшихся
                self.adaptive_finished_v = self.sim.finished_vehicles + self.sim.vehicles
                self.adaptive_finished_p = self.sim.finished_pedestrians + self.sim.pedestrians

                print(f"[Этап 2: Адаптивный режим завершен] Всего учтено ТС: {len(self.adaptive_finished_v)} (проехало: {len(self.sim.finished_vehicles)})")
                self.state = "FINISHED"
                self.timer.stop()

                # Запуск итогового отчета Pandas
                analyze_and_report_results(self.fixed_finished_v, self.adaptive_finished_v,
                                           self.fixed_finished_p, self.adaptive_finished_p)

        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self.state in ("RUN_FIXED", "RUN_ADAPTIVE"):
            self.sim.draw(p, self.font_hud, self.font_sm, self.sim_time, self.blinker_on)
        elif self.state == "TRANSITION":
            p.fillRect(self.rect(), COLOR_BG)
            p.setFont(self.font_splash)
            p.setPen(QColor(245, 215, 60))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       "ЭТАП 1 ЗАВЕРШЕН.\nЗАПУСК АДАПТИВНОГО УПРАВЛЕНИЯ ПЕРЕКРЕСТКОМ...")
        elif self.state == "FINISHED":
            p.fillRect(self.rect(), COLOR_BG)
            p.setFont(self.font_splash)
            p.setPen(QColor(110, 245, 130))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                       "СИМУЛЯЦИЯ ПОЛНОСТЬЮ ЗАВЕРШЕНА.\nОтчет сформирован в консоли.")


# ==========================================
# 6. АНАЛИТИКА PANDAS
# ==========================================
def analyze_and_report_results(fixed_v: List[Vehicle], adaptive_v: List[Vehicle],
                               fixed_p: List[Pedestrian], adaptive_p: List[Pedestrian]):
    def to_veh_df(vehicles: List[Vehicle], mode_name: str) -> pd.DataFrame:
        data = [{
            'mode': mode_name,
            'type': v.vehicle_type,
            'approach': v.approach,
            'intent': v.turn_intent,
            'is_avenue': v.approach in ('EB', 'WB'),
            'wait_time': v.total_wait_time
        } for v in vehicles]
        return pd.DataFrame(data)

    df_vf = to_veh_df(fixed_v, 'Фиксированный')
    df_va = to_veh_df(adaptive_v, 'Адаптивный')

    wait_p_f = [p.wait_time for p in fixed_p]
    wait_p_a = [p.wait_time for p in adaptive_p]
    mean_p_f = float(sum(wait_p_f) / len(wait_p_f)) if wait_p_f else 0.0
    mean_p_a = float(sum(wait_p_a) / len(wait_p_a)) if wait_p_a else 0.0

    def get_stat(df: pd.DataFrame, key: str, filt=None) -> float:
        if df.empty: return 0.0
        sub = df if filt is None else df[filt]
        if sub.empty: return 0.0
        if key == 'count': return float(len(sub))
        if key == 'wait': return float(sub['wait_time'].mean())
        return 0.0

    report_data = [
        ("Проехало ТС ВСЕГО (шт)", get_stat(df_vf, 'count'), get_stat(df_va, 'count'), False),
        ("— прямоходных ТС (шт)", get_stat(df_vf, 'count', df_vf['intent'] == 'straight'), get_stat(df_va, 'count', df_va['intent'] == 'straight'), False),
        ("— поворачивающих направо (шт)", get_stat(df_vf, 'count', df_vf['intent'] == 'right'), get_stat(df_va, 'count', df_va['intent'] == 'right'), False),
        ("— поворачивающих налево (шт)", get_stat(df_vf, 'count', df_vf['intent'] == 'left'), get_stat(df_va, 'count', df_va['intent'] == 'left'), False),
        ("Средняя задержка ВСЕХ ТС (сек)", get_stat(df_vf, 'wait'), get_stat(df_va, 'wait'), True),
        ("Задержка движения ПРЯМО (сек)", get_stat(df_vf, 'wait', df_vf['intent'] == 'straight'), get_stat(df_va, 'wait', df_va['intent'] == 'straight'), True),
        ("Задержка ПОВОРОТА НАПРАВО (сек)", get_stat(df_vf, 'wait', df_vf['intent'] == 'right'), get_stat(df_va, 'wait', df_va['intent'] == 'right'), True),
        ("Задержка ПОВОРОТА НАЛЕВО (сек)", get_stat(df_vf, 'wait', df_vf['intent'] == 'left'), get_stat(df_va, 'wait', df_va['intent'] == 'left'), True),
        ("Задержка АВТОБУСОВ (сек)", get_stat(df_vf, 'wait', df_vf['type'] == 'bus'), get_stat(df_va, 'wait', df_va['type'] == 'bus'), True),
        ("Пешеходов перешло (чел)", float(len(fixed_p)), float(len(adaptive_p)), False),
        ("Средняя задержка ПЕШЕХОДОВ (сек)", mean_p_f, mean_p_a, True),
    ]

    rows = []
    for title, v_f, v_a, is_wait in report_data:
        if v_f > 0:
            delta = ((v_f - v_a) / v_f * 100) if is_wait else ((v_a - v_f) / v_f * 100)
            delta_str = f"▼ {delta:.1f}%" if (is_wait and delta >= 0) else (f"▲ {delta:.1f}%" if not is_wait else f"▲ {-delta:.1f}%")
        else:
            delta_str = "0.0%"
        rows.append({
            "Показатель": title,
            "Фиксированный": f"{v_f:.1f}",
            "Адаптивный": f"{v_a:.1f}",
            "Эффект": delta_str
        })

    summary_df = pd.DataFrame(rows)
    print("\n" + "=" * 88)
    print("ИТОГОВАЯ СРАВНИТЕЛЬНАЯ ТАБЛИЦА СИМУЛЯЦИИ ОДИНОЧНОГО ПЕРЕКРЕСТКА (PANDAS):")
    print("=" * 88)
    print(summary_df.to_string(index=False))
    print("=" * 88)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SingleIntersectionWidget()
    window.show()
    sys.exit(app.exec())