import pygame
import random
import time
import math
from typing import List, Dict, Any, Tuple, Optional

# ==========================================
# 1. ДИНАМИЧЕСКАЯ ИНИЦИАЛИЗАЦИЯ ПОД МОНИТОР
# ==========================================
pygame.init()
pygame.font.init()

display_info = pygame.display.Info()
SCREEN_WIDTH = max(1100, int(display_info.current_w * 0.88))
SCREEN_HEIGHT = max(650, int(display_info.current_h * 0.80))

FPS = 60
SIMULATION_DURATION = 90

LANE_WIDTH = 52
TRAM_LANE_WIDTH = 38
AVENUE_Y = SCREEN_HEIGHT // 2
HALF_LANE = LANE_WIDTH / 2
HALF_TRAM = TRAM_LANE_WIDTH / 2

NUM_INTERSECTIONS = 2
spacing = SCREEN_WIDTH // (NUM_INTERSECTIONS + 1)
INTERSECTION_X_COORDS = [int((i + 1) * spacing) for i in range(NUM_INTERSECTIONS)]

CAR_LENGTH, CAR_WIDTH = 48, 26
BUS_LENGTH, BUS_WIDTH = 80, 28
TRAM_LENGTH, TRAM_WIDTH = 110, 22

COLOR_BLACK = (15, 15, 20)
COLOR_ROAD = (48, 48, 54)
COLOR_TRAM_BALLAST = (34, 34, 40)
COLOR_RAIL = (180, 180, 190)
COLOR_MARKING = (230, 230, 230)
COLOR_STOP_LINE = (240, 50, 50)
COLOR_BOX_GRID = (85, 75, 35)
COLOR_WHITE = (255, 255, 255)

COLOR_CAR = (45, 130, 240)
COLOR_BUS = (240, 150, 20)
COLOR_TRAM = (190, 45, 190)

COLOR_RED = (255, 40, 40)
COLOR_YELLOW = (255, 215, 30)
COLOR_GREEN = (40, 235, 60)
COLOR_LUNAR_WHITE = (220, 235, 255)

VEHICLE_SPEED = 3.6
COLLISION_BUFFER = 26
LIGHT_STOP_DISTANCE = 55

SPAWN_CHANCE_AVENUE = 0.038
SPAWN_CHANCE_CROSS = 0.015

BUS_PRIORITY_WEIGHT = 1.4
TRAM_PRIORITY_WEIGHT = 2.4

# Безопасные переходные интервалы
YELLOW_TIME = 2.0
ALL_RED_TIME = 1.5
ADAPTIVE_ALL_RED = 0.8  # Динамическая срезка мертвого времени перекрестка

# Фиксированный режим (цикл = 43.0 с)
FIXED_GREEN_AVENUE = 24.0
FIXED_GREEN_CROSS = 12.0

# Адаптивный режим: динамический сжатый цикл (28 - 34 с)
MIN_ADAPTIVE_AVENUE = 16.0
MAX_ADAPTIVE_AVENUE = 21.0
MIN_ADAPTIVE_CROSS = 6.0
MAX_ADAPTIVE_CROSS = 9.0


# ==========================================
# 2. СВЕТОФОРНЫЕ ОБЪЕКТЫ
# ==========================================
class TrafficLight:
    def __init__(self, x: int, y: int, orientation: str = 'vertical'):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.state = 'red'
        self.timer = 0.0

    def update(self, dt: float):
        self.timer = max(0.0, self.timer - dt)

    def draw(self, screen: pygame.Surface, font: pygame.font.Font, show_timer: bool = True):
        r_c = COLOR_RED if self.state == 'red' else (50, 10, 10)
        y_c = COLOR_YELLOW if self.state == 'yellow' else (50, 45, 5)
        g_c = COLOR_GREEN if self.state == 'green' else (10, 45, 15)
        r = 7

        if self.orientation == 'vertical':
            pygame.draw.rect(screen, (20, 20, 25), (self.x - 11, self.y - 28, 22, 56), border_radius=4)
            pygame.draw.circle(screen, r_c, (self.x, self.y - 18), r)
            pygame.draw.circle(screen, y_c, (self.x, self.y), r)
            pygame.draw.circle(screen, g_c, (self.x, self.y + 18), r)
            tx, ty = self.x + 15, self.y - 8
        else:
            pygame.draw.rect(screen, (20, 20, 25), (self.x - 28, self.y - 11, 56, 22), border_radius=4)
            pygame.draw.circle(screen, r_c, (self.x - 18, self.y), r)
            pygame.draw.circle(screen, y_c, (self.x, self.y), r)
            pygame.draw.circle(screen, g_c, (self.x + 18, self.y), r)
            tx, ty = self.x - 8, self.y + 14

        if show_timer:
            num = font.render(str(math.ceil(self.timer)), True, COLOR_WHITE)
            screen.blit(num, (tx, ty))


class TramTrafficLight:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.state = 'red'
        self.timer = 0.0

    def update(self, dt: float):
        self.timer = max(0.0, self.timer - dt)

    def draw(self, screen: pygame.Surface, font: pygame.font.Font, show_timer: bool = True):
        pygame.draw.rect(screen, (25, 25, 30), (self.x - 18, self.y - 18, 36, 36), border_radius=4)
        active = COLOR_LUNAR_WHITE if self.state == 'green' else (45, 45, 55)
        top_c = COLOR_RED if self.state == 'red' else active

        pygame.draw.circle(screen, top_c, (self.x - 10, self.y - 9), 4)
        pygame.draw.circle(screen, top_c, (self.x, self.y - 9), 4)
        pygame.draw.circle(screen, top_c, (self.x + 10, self.y - 9), 4)
        pygame.draw.circle(screen, active, (self.x, self.y + 9), 4)

        if show_timer:
            num = font.render(str(math.ceil(self.timer)), True, COLOR_WHITE)
            screen.blit(num, (self.x + 22, self.y - 8))


# ==========================================
# 3. ТРАНСПОРТНЫЕ СРЕДСТВА
# ==========================================
class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x: float, y: float, direction: str, lane_id: str,
                 vehicle_type: str = 'car', origin_intersection: Optional[int] = None):
        super().__init__()
        self.x = float(x)
        self.y = float(y)
        self.direction = direction
        self.lane_id = lane_id
        self.vehicle_type = vehicle_type
        self.speed = VEHICLE_SPEED
        self.origin_intersection = origin_intersection

        self.stopped = False
        self.stopped_by_light = False
        self.stopped_by_car = False
        self.stopped_by_gridlock = False
        self.wait_start_time: Optional[float] = None
        self.total_wait_time = 0.0

        if self.direction == 'EB':
            self.target_intersection_idx = 0
        elif self.direction == 'WB':
            self.target_intersection_idx = NUM_INTERSECTIONS - 1
        else:
            self.target_intersection_idx = origin_intersection

        if vehicle_type == 'tram':
            self.length, self.width = TRAM_LENGTH, TRAM_WIDTH
            self.color = COLOR_TRAM
        elif vehicle_type == 'bus':
            self.length, self.width = BUS_LENGTH, BUS_WIDTH
            self.color = COLOR_BUS
        else:
            self.length, self.width = CAR_LENGTH, CAR_WIDTH
            self.color = COLOR_CAR

        self.base_image = pygame.Surface((self.length, self.width), pygame.SRCALPHA)
        self.base_image.fill(self.color)
        pygame.draw.rect(self.base_image, (255, 255, 220), (self.length - 4, 2, 4, 6))
        pygame.draw.rect(self.base_image, (255, 255, 220), (self.length - 4, self.width - 8, 4, 6))

        angle_map = {'EB': 0, 'WB': 180, 'SB': -90, 'NB': 90}
        self.image = pygame.transform.rotate(self.base_image, angle_map[self.direction])
        self.rect = self.image.get_rect(center=(int(self.x), int(self.y)))

    def update_position(self, sim_time: float):
        should_wait = self.stopped_by_light or self.stopped_by_car or self.stopped_by_gridlock
        if should_wait:
            if not self.stopped:
                self.stopped = True
                self.wait_start_time = sim_time
        else:
            if self.stopped and self.wait_start_time is not None:
                self.total_wait_time += (sim_time - self.wait_start_time)
                self.wait_start_time = None
            self.stopped = False

        if not self.stopped:
            if self.direction == 'EB':
                self.x += self.speed
            elif self.direction == 'WB':
                self.x -= self.speed
            elif self.direction == 'SB':
                self.y += self.speed
            elif self.direction == 'NB':
                self.y -= self.speed

            self.rect.center = (round(self.x), round(self.y))


# ==========================================
# 4. УЗЕЛ ПЕРЕКРЕСТКА
# ==========================================
class IntersectionNode:
    def __init__(self, idx: int, center_x: int, center_y: int, mode: str, offset: float = 0.0):
        self.idx = idx
        self.center_x = center_x
        self.center_y = center_y
        self.mode = mode
        self.offset = offset

        self.box_left = center_x - LANE_WIDTH
        self.box_right = center_x + LANE_WIDTH
        self.box_top = center_y - (TRAM_LANE_WIDTH + LANE_WIDTH)
        self.box_bottom = center_y + (TRAM_LANE_WIDTH + LANE_WIDTH)
        self.rect = pygame.Rect(self.box_left, self.box_top,
                                self.box_right - self.box_left,
                                self.box_bottom - self.box_top)

        self.light_avenue_eb = TrafficLight(self.box_left - 18, self.center_y + TRAM_LANE_WIDTH + HALF_LANE, 'horizontal')
        self.light_avenue_wb = TrafficLight(self.box_right + 18, self.center_y - TRAM_LANE_WIDTH - HALF_LANE, 'horizontal')
        self.light_tram = TramTrafficLight(self.box_left - 24, self.center_y)

        self.light_cross_sb = TrafficLight(self.center_x - HALF_LANE, self.box_top - 20, 'vertical')
        self.light_cross_nb = TrafficLight(self.center_x + HALF_LANE, self.box_bottom + 20, 'vertical')

        self.phase = 'AVENUE_GREEN'
        self.phase_timer = FIXED_GREEN_AVENUE

        self._sync_all_lights()

    def is_inside_box(self, v: Vehicle) -> bool:
        return self.rect.colliderect(v.rect)

    def is_past_box(self, v: Vehicle) -> bool:
        if v.direction == 'EB':
            return v.rect.left >= self.box_right
        elif v.direction == 'WB':
            return v.rect.right <= self.box_left
        elif v.direction == 'SB':
            return v.rect.top >= self.box_bottom
        elif v.direction == 'NB':
            return v.rect.bottom <= self.box_top
        return False

    def get_distance_to_stop_line(self, v: Vehicle) -> float:
        if v.direction == 'EB':
            return self.box_left - v.rect.right
        elif v.direction == 'WB':
            return v.rect.left - self.box_right
        elif v.direction == 'SB':
            return self.box_top - v.rect.bottom
        elif v.direction == 'NB':
            return v.rect.top - self.box_bottom
        return float('inf')

    def set_phase(self, new_phase: str, duration: float):
        self.phase = new_phase
        self.phase_timer = duration
        self._sync_all_lights()

    def update_fixed(self, dt: float):
        self.phase_timer = max(0.0, self.phase_timer - dt)
        for lt in [self.light_avenue_eb, self.light_avenue_wb, self.light_cross_sb, self.light_cross_nb]:
            lt.update(dt)
        self.light_tram.update(dt)

        if self.phase_timer <= 0:
            if self.phase == 'AVENUE_GREEN':
                self.set_phase('AVENUE_YELLOW', YELLOW_TIME)
            elif self.phase == 'AVENUE_YELLOW':
                self.set_phase('ALL_RED_1', ALL_RED_TIME)
            elif self.phase == 'ALL_RED_1':
                self.set_phase('CROSS_GREEN', FIXED_GREEN_CROSS)
            elif self.phase == 'CROSS_GREEN':
                self.set_phase('CROSS_YELLOW', YELLOW_TIME)
            elif self.phase == 'CROSS_YELLOW':
                self.set_phase('ALL_RED_2', ALL_RED_TIME)
            elif self.phase == 'ALL_RED_2':
                self.set_phase('AVENUE_GREEN', FIXED_GREEN_AVENUE)

    def update_lights_only(self, dt: float):
        for lt in [self.light_avenue_eb, self.light_avenue_wb, self.light_cross_sb, self.light_cross_nb]:
            lt.update(dt)
        self.light_tram.update(dt)

    def _sync_all_lights(self):
        if self.phase == 'AVENUE_GREEN':
            for lt in [self.light_avenue_eb, self.light_avenue_wb]:
                lt.state, lt.timer = 'green', self.phase_timer
            self.light_tram.state, self.light_tram.timer = 'green', self.phase_timer
            for lt in [self.light_cross_sb, self.light_cross_nb]:
                lt.state, lt.timer = 'red', self.phase_timer + YELLOW_TIME + ALL_RED_TIME

        elif self.phase == 'AVENUE_YELLOW':
            for lt in [self.light_avenue_eb, self.light_avenue_wb]:
                lt.state, lt.timer = 'yellow', self.phase_timer
            self.light_tram.state, self.light_tram.timer = 'red', self.phase_timer
            for lt in [self.light_cross_sb, self.light_cross_nb]:
                lt.state, lt.timer = 'red', self.phase_timer + ALL_RED_TIME

        elif self.phase in ('ALL_RED_1', 'ALL_RED_2'):
            for lt in [self.light_avenue_eb, self.light_avenue_wb, self.light_cross_sb, self.light_cross_nb]:
                lt.state, lt.timer = 'red', self.phase_timer
            self.light_tram.state, self.light_tram.timer = 'red', self.phase_timer

        elif self.phase == 'CROSS_GREEN':
            for lt in [self.light_cross_sb, self.light_cross_nb]:
                lt.state, lt.timer = 'green', self.phase_timer
            for lt in [self.light_avenue_eb, self.light_avenue_wb]:
                lt.state, lt.timer = 'red', self.phase_timer + YELLOW_TIME + ALL_RED_TIME
            self.light_tram.state, self.light_tram.timer = 'red', self.phase_timer + YELLOW_TIME + ALL_RED_TIME

        elif self.phase == 'CROSS_YELLOW':
            for lt in [self.light_cross_sb, self.light_cross_nb]:
                lt.state, lt.timer = 'yellow', self.phase_timer
            for lt in [self.light_avenue_eb, self.light_avenue_wb]:
                lt.state, lt.timer = 'red', self.phase_timer + ALL_RED_TIME
            self.light_tram.state, self.light_tram.timer = 'red', self.phase_timer + ALL_RED_TIME

    def draw(self, screen: pygame.Surface, font: pygame.font.Font):
        pygame.draw.rect(screen, (34, 34, 30), self.rect)
        pygame.draw.rect(screen, COLOR_STOP_LINE, self.rect, 2)
        step = 16
        for x in range(self.box_left, self.box_right + (self.box_bottom - self.box_top), step):
            pygame.draw.line(screen, COLOR_BOX_GRID,
                             (x, self.box_top),
                             (x - (self.box_bottom - self.box_top), self.box_bottom), 1)

        show_timer = (self.mode == 'fixed')

        self.light_avenue_eb.draw(screen, font, show_timer=show_timer)
        self.light_avenue_wb.draw(screen, font, show_timer=show_timer)
        self.light_tram.draw(screen, font, show_timer=show_timer)
        self.light_cross_sb.draw(screen, font, show_timer=show_timer)
        self.light_cross_nb.draw(screen, font, show_timer=show_timer)


# ==========================================
# 5. СИМУЛЯТОР ПРОСПЕКТА
# ==========================================
class ArterialProspectSimulation:
    def __init__(self, mode: str = 'fixed', spawn_playback: Optional[List[Dict[str, Any]]] = None):
        self.mode = mode
        self.spawn_playback = spawn_playback or []
        self.spawn_log: List[Dict[str, Any]] = []
        self.playback_idx = 0

        self.vehicles = pygame.sprite.Group()
        self.finished_vehicles: List[Vehicle] = []

        # Синхронная волна: оба узла открываются в унисон
        self.intersections: List[IntersectionNode] = []
        for i, x in enumerate(INTERSECTION_X_COORDS):
            node = IntersectionNode(i, x, AVENUE_Y, mode, offset=0.0)
            self.intersections.append(node)

        # Магистральный контроллер
        self.adaptive_phase = 'AVENUE_GREEN'
        self.adaptive_timer = MIN_ADAPTIVE_AVENUE
        self.adaptive_time_in_phase = 0.0

        self.entry_queues: Dict[str, List[Dict[str, Any]]] = {
            'EB_car': [], 'EB_tram': [],
            'WB_car': [], 'WB_tram': []
        }
        for k in range(NUM_INTERSECTIONS):
            self.entry_queues[f'SB_{k}'] = []
            self.entry_queues[f'NB_{k}'] = []

    def get_lane_center_y(self, lane_id: str) -> float:
        if lane_id == 'EB_car':
            return AVENUE_Y + TRAM_LANE_WIDTH + HALF_LANE
        elif lane_id == 'EB_tram':
            return AVENUE_Y + HALF_TRAM
        elif lane_id == 'WB_tram':
            return AVENUE_Y - HALF_TRAM
        elif lane_id == 'WB_car':
            return AVENUE_Y - TRAM_LANE_WIDTH - HALF_LANE
        return AVENUE_Y

    def get_cross_center_x(self, node_idx: int, direction: str) -> float:
        node_x = INTERSECTION_X_COORDS[node_idx]
        return node_x - HALF_LANE if direction == 'SB' else node_x + HALF_LANE

    def set_master_adaptive_phase(self, new_phase: str, duration: float):
        self.adaptive_phase = new_phase
        self.adaptive_timer = duration
        self.adaptive_time_in_phase = 0.0
        for node in self.intersections:
            node.set_phase(new_phase, duration)

    def update_adaptive_coordination(self, dt: float):
        self.adaptive_time_in_phase += dt
        self.adaptive_timer = max(0.0, self.adaptive_timer - dt)

        # Подсчет очередей на боковых улицах
        cross_counts = []
        for node in self.intersections:
            q_cnt = sum(
                1 for v in self.vehicles
                if v.target_intersection_idx == node.idx and v.direction in ('SB', 'NB') and 0 <= node.get_distance_to_stop_line(v) <= 240
            )
            q_cnt += len(self.entry_queues.get(f'SB_{node.idx}', []))
            q_cnt += len(self.entry_queues.get(f'NB_{node.idx}', []))
            cross_counts.append(q_cnt)

        total_cross_waiting = sum(cross_counts)
        max_cross_q = max(cross_counts) if cross_counts else 0

        # Локальная детекция трамвая на подходе (в пределах 240 px от любого перекрестка)
        tram_close_to_stop = False
        for node in self.intersections:
            for v in self.vehicles:
                if v.vehicle_type == 'tram' and v.direction in ('EB', 'WB'):
                    dist = node.get_distance_to_stop_line(v)
                    if 0 <= dist <= 240:
                        tram_close_to_stop = True
                        break
            if tram_close_to_stop:
                break

        # Проверка плотного потока на проспекте
        avenue_cars_close = False
        for node in self.intersections:
            for v in self.vehicles:
                if v.direction in ('EB', 'WB'):
                    dist = node.get_distance_to_stop_line(v)
                    if 0 <= dist <= 180:
                        avenue_cars_close = True
                        break
            if avenue_cars_close:
                break

        # 1. ФАЗА: ПРОСПЕКТ (AVENUE_GREEN)
        if self.adaptive_phase == 'AVENUE_GREEN':
            # Если боковые улицы пусты — проспект едет непрерывно
            if total_cross_waiting == 0:
                self.adaptive_timer = 3.0
                return

            # Трамвай подходит к стоп-линии — удерживаем коридор
            if tram_close_to_stop and self.adaptive_time_in_phase < MAX_ADAPTIVE_AVENUE:
                self.adaptive_timer = max(self.adaptive_timer, 2.5)
                return

            # Сбалансированный переход: выдержан минимальный зеленый проспекта (16с) и появился разрыв
            # Либо сработал максимальный потолок проспекта (21с)
            can_gap_switch = (self.adaptive_time_in_phase >= MIN_ADAPTIVE_AVENUE and not avenue_cars_close)
            max_limit_reached = (self.adaptive_time_in_phase >= MAX_ADAPTIVE_AVENUE)

            if can_gap_switch or max_limit_reached:
                self.set_master_adaptive_phase('AVENUE_YELLOW', YELLOW_TIME)

        # 2. ПЕРЕХОД: ЖЕЛТЫЙ ПРОСПЕКТА
        elif self.adaptive_phase == 'AVENUE_YELLOW' and self.adaptive_timer <= 0:
            # Срезка мертвого времени All-Red
            boxes_clear = not any(node.is_inside_box(v) for node in self.intersections for v in self.vehicles)
            all_red_dur = ADAPTIVE_ALL_RED if boxes_clear else ALL_RED_TIME
            self.set_master_adaptive_phase('ALL_RED_1', all_red_dur)

        # 3. ПЕРЕХОД: ВСЕМ КРАСНЫЙ -> ВКЛЮЧЕНИЕ БОКОВЫХ
        elif self.adaptive_phase == 'ALL_RED_1' and self.adaptive_timer <= 0:
            # Длина боковой фазы: от 6.0с до 9.0с (в среднем 7.5с)
            allocated_cross = min(MAX_ADAPTIVE_CROSS, max(MIN_ADAPTIVE_CROSS, 5.5 + 0.9 * max_cross_q))
            self.set_master_adaptive_phase('CROSS_GREEN', allocated_cross)

        # 4. ФАЗА: БОКОВЫЕ УЛИЦЫ (CROSS_GREEN)
        elif self.adaptive_phase == 'CROSS_GREEN':
            # Проверяем зону разъезда (до 140 px от стоп-линии)
            still_discharging = False
            for node in self.intersections:
                if any(v.target_intersection_idx == node.idx and v.direction in ('SB', 'NB') and 0 <= node.get_distance_to_stop_line(v) <= 140 for v in self.vehicles):
                    still_discharging = True
                    break

            # Завершение фазы: очередь выехала после минимума 6.0 с, либо истек таймер
            if (not still_discharging and self.adaptive_time_in_phase >= MIN_ADAPTIVE_CROSS) or self.adaptive_timer <= 0:
                self.set_master_adaptive_phase('CROSS_YELLOW', YELLOW_TIME)

        # 5. ПЕРЕХОД: ЖЕЛТЫЙ БОКОВЫХ
        elif self.adaptive_phase == 'CROSS_YELLOW' and self.adaptive_timer <= 0:
            boxes_clear = not any(node.is_inside_box(v) for node in self.intersections for v in self.vehicles)
            all_red_dur = ADAPTIVE_ALL_RED if boxes_clear else ALL_RED_TIME
            self.set_master_adaptive_phase('ALL_RED_2', all_red_dur)

        # 6. ПЕРЕХОД: ВСЕМ КРАСНЫЙ -> ВОЗВРАТ ПРОСПЕКТУ
        elif self.adaptive_phase == 'ALL_RED_2' and self.adaptive_timer <= 0:
            self.set_master_adaptive_phase('AVENUE_GREEN', MIN_ADAPTIVE_AVENUE)

    def spawn_traffic(self, sim_time: float):
        if self.spawn_playback:
            while self.playback_idx < len(self.spawn_playback):
                rec = self.spawn_playback[self.playback_idx]
                if rec['time'] <= sim_time:
                    self.entry_queues[rec['queue_key']].append(dict(rec))
                    self.playback_idx += 1
                else:
                    break
            return

        if random.random() < SPAWN_CHANCE_AVENUE:
            direction = random.choice(['EB', 'WB'])
            v_type = random.choices(['car', 'bus', 'tram'], weights=[0.60, 0.22, 0.18])[0]
            lane_id = f"{direction}_{'tram' if v_type == 'tram' else 'car'}"
            rec = {'time': sim_time, 'direction': direction, 'type': v_type, 'lane_id': lane_id,
                   'node_idx': None, 'queue_key': lane_id}
            self.spawn_log.append(rec)
            self.entry_queues[lane_id].append(rec)

        for k in range(NUM_INTERSECTIONS):
            if random.random() < SPAWN_CHANCE_CROSS:
                direction = random.choice(['SB', 'NB'])
                v_type = random.choices(['car', 'bus'], weights=[0.8, 0.2])[0]
                q_key = f"{direction}_{k}"
                rec = {'time': sim_time, 'direction': direction, 'type': v_type, 'lane_id': q_key,
                       'node_idx': k, 'queue_key': q_key}
                self.spawn_log.append(rec)
                self.entry_queues[q_key].append(rec)

    def process_spawns(self):
        for q_key, queue in self.entry_queues.items():
            if not queue:
                continue

            item = queue[0]
            v_type, direction = item['type'], item['direction']
            v_len = TRAM_LENGTH if v_type == 'tram' else (BUS_LENGTH if v_type == 'bus' else CAR_LENGTH)

            can_spawn = True
            if direction == 'EB':
                x, y = -v_len, self.get_lane_center_y(item['lane_id'])
                for v in self.vehicles:
                    if v.lane_id == item['lane_id'] and v.rect.left < v_len + COLLISION_BUFFER:
                        can_spawn = False
                        break
            elif direction == 'WB':
                x, y = SCREEN_WIDTH + v_len, self.get_lane_center_y(item['lane_id'])
                for v in self.vehicles:
                    if v.lane_id == item['lane_id'] and v.rect.right > SCREEN_WIDTH - (v_len + COLLISION_BUFFER):
                        can_spawn = False
                        break
            elif direction == 'SB':
                k = item['node_idx']
                x, y = self.get_cross_center_x(k, 'SB'), -v_len
                for v in self.vehicles:
                    if v.lane_id == item['lane_id'] and v.rect.top < v_len + COLLISION_BUFFER:
                        can_spawn = False
                        break
            elif direction == 'NB':
                k = item['node_idx']
                x, y = self.get_cross_center_x(k, 'NB'), SCREEN_HEIGHT + v_len
                for v in self.vehicles:
                    if v.lane_id == item['lane_id'] and v.rect.bottom > SCREEN_HEIGHT - (v_len + COLLISION_BUFFER):
                        can_spawn = False
                        break

            if can_spawn:
                queue.pop(0)
                new_v = Vehicle(x, y, direction, item['lane_id'], v_type, origin_intersection=item['node_idx'])
                self.vehicles.add(new_v)

    def can_enter_intersection(self, v: Vehicle, node: IntersectionNode, all_v: List[Vehicle]) -> bool:
        is_cross_dir = (lambda d: d in ('SB', 'NB')) if v.direction in ('EB', 'WB') else (lambda d: d in ('EB', 'WB'))
        for other in all_v:
            if is_cross_dir(other.direction) and node.is_inside_box(other):
                return False

        required_space = v.length + COLLISION_BUFFER

        if v.direction == 'EB':
            exit_x = node.box_right
            limit_x = SCREEN_WIDTH + 200

            if node.idx + 1 < NUM_INTERSECTIONS:
                next_node = self.intersections[node.idx + 1]
                next_light = next_node.light_tram.state if v.vehicle_type == 'tram' else next_node.light_avenue_eb.state
                if next_light in ('red', 'yellow'):
                    limit_x = next_node.box_left

            for other in all_v:
                if other != v and other.lane_id == v.lane_id and other.x > v.x:
                    if other.stopped and other.rect.left >= exit_x:
                        limit_x = min(limit_x, other.rect.left)

            occupied_space = sum(
                other.length + COLLISION_BUFFER
                for other in all_v
                if other != v and other.lane_id == v.lane_id and other.x > v.x and other.rect.right > exit_x and other.rect.left < limit_x
            )
            if ((limit_x - exit_x) - occupied_space) < required_space:
                return False

        elif v.direction == 'WB':
            exit_x = node.box_left
            limit_x = -200

            if node.idx - 1 >= 0:
                prev_node = self.intersections[node.idx - 1]
                prev_light = prev_node.light_tram.state if v.vehicle_type == 'tram' else prev_node.light_avenue_wb.state
                if prev_light in ('red', 'yellow'):
                    limit_x = prev_node.box_right

            for other in all_v:
                if other != v and other.lane_id == v.lane_id and other.x < v.x:
                    if other.stopped and other.rect.right <= exit_x:
                        limit_x = max(limit_x, other.rect.right)

            occupied_space = sum(
                other.length + COLLISION_BUFFER
                for other in all_v
                if other != v and other.lane_id == v.lane_id and other.x < v.x and other.rect.left < exit_x and other.rect.right > limit_x
            )
            if ((exit_x - limit_x) - occupied_space) < required_space:
                return False

        elif v.direction == 'SB':
            exit_y = node.box_bottom
            limit_y = SCREEN_HEIGHT + 200
            for other in all_v:
                if other != v and other.lane_id == v.lane_id and other.y > v.y:
                    if other.stopped and other.rect.top >= exit_y:
                        limit_y = min(limit_y, other.rect.top)

            occupied_space = sum(
                other.length + COLLISION_BUFFER
                for other in all_v
                if other != v and other.lane_id == v.lane_id and other.y > v.y and other.rect.bottom > exit_y and other.rect.top < limit_y
            )
            if ((limit_y - exit_y) - occupied_space) < required_space:
                return False

        elif v.direction == 'NB':
            exit_y = node.box_top
            limit_y = -200
            for other in all_v:
                if other != v and other.lane_id == v.lane_id and other.y < v.y:
                    if other.stopped and other.rect.bottom <= exit_y:
                        limit_y = max(limit_y, other.rect.bottom)

            occupied_space = sum(
                other.length + COLLISION_BUFFER
                for other in all_v
                if other != v and other.lane_id == v.lane_id and other.y < v.y and other.rect.top < exit_y and other.rect.bottom > limit_y
            )
            if ((exit_y - limit_y) - occupied_space) < required_space:
                return False

        return True

    def update_vehicle_logic(self, sim_time: float):
        all_v = list(self.vehicles)

        for v in all_v:
            should_stop_light = False
            should_stop_gridlock = False
            k = v.target_intersection_idx

            if k is not None and 0 <= k < NUM_INTERSECTIONS:
                node = self.intersections[k]

                if node.is_past_box(v):
                    if v.direction == 'EB':
                        v.target_intersection_idx += 1
                        if v.target_intersection_idx >= NUM_INTERSECTIONS:
                            v.target_intersection_idx = None
                    elif v.direction == 'WB':
                        v.target_intersection_idx -= 1
                        if v.target_intersection_idx < 0:
                            v.target_intersection_idx = None
                    else:
                        v.target_intersection_idx = None

                elif not node.is_inside_box(v):
                    dist_to_stop = node.get_distance_to_stop_line(v)

                    if 0 <= dist_to_stop <= LIGHT_STOP_DISTANCE:
                        if v.direction == 'EB':
                            light_state = node.light_tram.state if v.vehicle_type == 'tram' else node.light_avenue_eb.state
                        elif v.direction == 'WB':
                            light_state = node.light_tram.state if v.vehicle_type == 'tram' else node.light_avenue_wb.state
                        elif v.direction == 'SB':
                            light_state = node.light_cross_sb.state
                        else:
                            light_state = node.light_cross_nb.state

                        if light_state in ('red', 'yellow'):
                            should_stop_light = True
                        elif light_state == 'green':
                            can_enter = self.can_enter_intersection(v, node, all_v)
                            if not can_enter:
                                should_stop_gridlock = True

            v.stopped_by_light = should_stop_light
            v.stopped_by_gridlock = should_stop_gridlock

            should_stop_car = False
            min_dist = float('inf')

            for other in all_v:
                if other == v or other.lane_id != v.lane_id:
                    continue

                d = float('inf')
                if v.direction == 'EB' and other.rect.left >= v.rect.right:
                    d = other.rect.left - v.rect.right
                elif v.direction == 'WB' and other.rect.right <= v.rect.left:
                    d = v.rect.left - other.rect.right
                elif v.direction == 'SB' and other.rect.top >= v.rect.bottom:
                    d = other.rect.top - v.rect.bottom
                elif v.direction == 'NB' and other.rect.bottom <= v.rect.top:
                    d = v.rect.top - other.rect.bottom

                if d < min_dist:
                    min_dist = d

            if min_dist < COLLISION_BUFFER:
                should_stop_car = True

            v.stopped_by_car = should_stop_car
            v.update_position(sim_time)

            out_bounds = (
                (v.direction == 'EB' and v.rect.left > SCREEN_WIDTH) or
                (v.direction == 'WB' and v.rect.right < 0) or
                (v.direction == 'SB' and v.rect.top > SCREEN_HEIGHT) or
                (v.direction == 'NB' and v.rect.bottom < 0)
            )

            if out_bounds:
                if v.wait_start_time is not None:
                    v.total_wait_time += (sim_time - v.wait_start_time)
                self.finished_vehicles.append(v)
                v.kill()

    def update(self, dt: float, sim_time: float):
        self.spawn_traffic(sim_time)
        self.process_spawns()

        if self.mode == 'fixed':
            for node in self.intersections:
                node.update_fixed(dt)
        else:
            self.update_adaptive_coordination(dt)
            for node in self.intersections:
                node.update_lights_only(dt)

        self.update_vehicle_logic(sim_time)

    def draw(self, screen: pygame.Surface, sim_time: float, font_hud: pygame.font.Font, font_sm: pygame.font.Font):
        screen.fill(COLOR_BLACK)

        ave_top = AVENUE_Y - (TRAM_LANE_WIDTH + LANE_WIDTH)
        ave_height = (TRAM_LANE_WIDTH + LANE_WIDTH) * 2
        pygame.draw.rect(screen, COLOR_ROAD, (0, ave_top, SCREEN_WIDTH, ave_height))

        tram_top = AVENUE_Y - TRAM_LANE_WIDTH
        pygame.draw.rect(screen, COLOR_TRAM_BALLAST, (0, tram_top, SCREEN_WIDTH, TRAM_LANE_WIDTH * 2))
        for dy in [-HALF_TRAM - 5, -HALF_TRAM + 5, HALF_TRAM - 5, HALF_TRAM + 5]:
            pygame.draw.line(screen, COLOR_RAIL, (0, AVENUE_Y + dy), (SCREEN_WIDTH, AVENUE_Y + dy), 2)

        for x in INTERSECTION_X_COORDS:
            pygame.draw.rect(screen, COLOR_ROAD, (x - LANE_WIDTH, 0, LANE_WIDTH * 2, SCREEN_HEIGHT))

        pygame.draw.line(screen, COLOR_MARKING, (0, AVENUE_Y), (SCREEN_WIDTH, AVENUE_Y), 2)
        pygame.draw.line(screen, COLOR_MARKING, (0, ave_top), (SCREEN_WIDTH, ave_top), 2)
        pygame.draw.line(screen, COLOR_MARKING, (0, ave_top + ave_height), (SCREEN_WIDTH, ave_top + ave_height), 2)

        for node in self.intersections:
            node.draw(screen, font_sm)
            pygame.draw.line(screen, COLOR_STOP_LINE, (node.box_left, ave_top), (node.box_left, ave_top + ave_height), 4)
            pygame.draw.line(screen, COLOR_STOP_LINE, (node.box_right, ave_top), (node.box_right, ave_top + ave_height), 4)
            pygame.draw.line(screen, COLOR_STOP_LINE, (node.box_left, node.box_top), (node.box_right, node.box_top), 4)
            pygame.draw.line(screen, COLOR_STOP_LINE, (node.box_left, node.box_bottom), (node.box_right, node.box_bottom), 4)

        self.vehicles.draw(screen)

        mode_title = "ЗЕЛЕНАЯ ВОЛНА (Синхронная)" if self.mode == 'fixed' else "КООРДИНИРОВАННЫЙ АДАПТИВ (Master-Optimization)"
        hud_l = font_hud.render(f"Время: {sim_time:.1f}/{SIMULATION_DURATION}с | Режим: {mode_title}", True, COLOR_WHITE)
        screen.blit(hud_l, (20, 15))

        hud_r = font_hud.render(f"Проехало: {len(self.finished_vehicles)} | ПДД 13.2: АКТИВНО", True, (200, 240, 200))
        screen.blit(hud_r, (SCREEN_WIDTH - 440, 15))

        legend = font_sm.render("Синий: Авто | Оранжевый: Автобус | Фиолетовый: Трамвай | Сетка: Зона вафельницы", True, (180, 180, 180))
        screen.blit(legend, (20, 42))


# ==========================================
# 6. РАННЕР СИМУЛЯЦИИ
# ==========================================
def show_transition_screen(screen: pygame.Surface, text: str, duration_sec: float = 2.0):
    font = pygame.font.SysFont("Consolas", 24, bold=True)
    start = time.time()
    while time.time() - start < duration_sec:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        screen.fill((25, 25, 30))
        txt_surf = font.render(text, True, (240, 210, 60))
        rect = txt_surf.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
        screen.blit(txt_surf, rect)
        pygame.display.flip()
        pygame.time.delay(30)


def run_simulation(screen: pygame.Surface, mode: str, playback_data: Optional[List[Dict[str, Any]]] = None) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    pygame.event.clear()
    pygame.display.set_caption(f"Проспект ({NUM_INTERSECTIONS} узла) - {mode.upper()}")
    clock = pygame.time.Clock()
    font_hud = pygame.font.SysFont("Consolas", 17, bold=True)
    font_sm = pygame.font.SysFont("Consolas", 13, bold=True)

    sim = ArterialProspectSimulation(mode=mode, spawn_playback=playback_data)
    sim_time = 0.0
    running = True

    while running and sim_time < SIMULATION_DURATION:
        dt = clock.tick(FPS) / 1000.0
        sim_time += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        sim.update(dt, sim_time)
        sim.draw(screen, sim_time, font_hud, font_sm)
        pygame.display.flip()

    finished = sim.finished_vehicles
    trams = [v for v in finished if v.vehicle_type == 'tram']
    buses = [v for v in finished if v.vehicle_type == 'bus']
    cars = [v for v in finished if v.vehicle_type == 'car']
    avenue_v = [v for v in finished if v.direction in ('EB', 'WB')]
    cross_v = [v for v in finished if v.direction in ('SB', 'NB')]

    def safe_avg(lst: List[Vehicle]) -> float:
        return sum(v.total_wait_time for v in lst) / len(lst) if lst else 0.0

    stats = {
        'total_finished': len(finished),
        'avg_wait_total': safe_avg(finished),
        'avg_wait_tram': safe_avg(trams),
        'avg_wait_bus': safe_avg(buses),
        'avg_wait_car': safe_avg(cars),
        'avg_wait_avenue': safe_avg(avenue_v),
        'avg_wait_cross': safe_avg(cross_v),
        'avenue_finished': len(avenue_v),
        'cross_finished': len(cross_v)
    }

    return stats, sim.spawn_log


if __name__ == '__main__':
    print("=" * 80)
    print(f" СИМУЛЯЦИЯ ПРОСПЕКТА ({NUM_INTERSECTIONS} УЗЛА) | ЭКРАН: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
    print("=" * 80)

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # 1. Запуск фиксированного режима
    random.seed(42)
    print("\n>>> ЭТАП 1: Запуск режима 'ЗЕЛЕНАЯ ВОЛНА'...")
    fixed_stats, recorded_spawns = run_simulation(screen, 'fixed')
    print(f"Этап 1 завершен. Проехало: {fixed_stats['total_finished']} ТС.")

    show_transition_screen(screen, "ЭТАП 1 ЗАВЕРШЕН. СТАРТ МАГИСТРАЛЬНОГО АДАПТИВА...", duration_sec=2.5)

    # 2. Запуск адаптивного режима на идентичном трафике
    random.seed(None)
    print("\n>>> ЭТАП 2: Запуск режима 'КООРДИНИРОВАННЫЙ АДАПТИВ' (100% повтор трафика)...")
    adaptive_stats, _ = run_simulation(screen, 'adaptive', playback_data=recorded_spawns)
    print(f"Этап 2 завершен. Проехало: {adaptive_stats['total_finished']} ТС.")

    pygame.quit()

    print("\n" + "=" * 80)
    print(f"{'ПОКАЗАТЕЛЬ':<38} | {'ЗЕЛЕНАЯ ВОЛНА':<16} | {'АДАПТИВНЫЙ'}")
    print("-" * 80)

    metrics = [
        ("Всего проехало ТС (шт)", 'total_finished', "{:.0f}"),
        ("По проспекту сквозных (шт)", 'avenue_finished', "{:.0f}"),
        ("По боковым улицам (шт)", 'cross_finished', "{:.0f}"),
        ("Средняя задержка ВСЕХ ТС (сек)", 'avg_wait_total', "{:.2f}"),
        ("Задержка НА ПРОСПЕКТЕ (сек)", 'avg_wait_avenue', "{:.2f}"),
        ("Задержка НА БОКОВЫХ (сек)", 'avg_wait_cross', "{:.2f}"),
        ("Задержка ТРАМВАЕВ (сек)", 'avg_wait_tram', "{:.2f}"),
        ("Задержка АВТОБУСОВ (сек)", 'avg_wait_bus', "{:.2f}"),
    ]

    for title, key, fmt in metrics:
        v_fix = fmt.format(fixed_stats[key])
        v_ada = fmt.format(adaptive_stats[key])
        print(f"{title:<38} | {v_fix:<16} | {v_ada}")
    print("=" * 80)

    diff_wait = ((fixed_stats['avg_wait_total'] - adaptive_stats['avg_wait_total']) / fixed_stats['avg_wait_total'] * 100) if fixed_stats['avg_wait_total'] > 0 else 0
    diff_ave = ((fixed_stats['avg_wait_avenue'] - adaptive_stats['avg_wait_avenue']) / fixed_stats['avg_wait_avenue'] * 100) if fixed_stats['avg_wait_avenue'] > 0 else 0
    diff_cross = ((fixed_stats['avg_wait_cross'] - adaptive_stats['avg_wait_cross']) / fixed_stats['avg_wait_cross'] * 100) if fixed_stats['avg_wait_cross'] > 0 else 0

    print(f"\nИТОГОВЫЙ ВЫВОД:")
    status_wait = f"снизилась на {diff_wait:.1f}%" if diff_wait >= 0 else f"выросла на {-diff_wait:.1f}%"
    status_ave = f"снизилась на {diff_ave:.1f}%" if diff_ave >= 0 else f"выросла на {-diff_ave:.1f}%"
    status_cross = f"снизилась на {diff_cross:.1f}%" if diff_cross >= 0 else f"выросла на {-diff_cross:.1f}%"

    print(f"1. Средняя задержка всех ТС: {status_wait} в адаптивном режиме.")
    print(f"2. Задержка проспекта: {status_ave}.")
    print(f"3. Задержка боковых улиц: {status_cross}.")
    print("=" * 80)