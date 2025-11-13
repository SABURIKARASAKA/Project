import pygame
import random
import time
import math
from typing import List, Dict, Any, Tuple

SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 1000
LANE_WIDTH = 60
NUM_CAR_LANES_PER_DIR = 1
NUM_TRAM_LANES_PER_DIR = 1

SIMULATION_DURATION = 120

TRAM_LANE_WIDTH = 40
TRAM_TOTAL_ROAD_WIDTH = TRAM_LANE_WIDTH * NUM_TRAM_LANES_PER_DIR * 2
TOTAL_CAR_ROAD_WIDTH = LANE_WIDTH * NUM_CAR_LANES_PER_DIR * 2
TOTAL_ROAD_WIDTH = TOTAL_CAR_ROAD_WIDTH + TRAM_TOTAL_ROAD_WIDTH

HALF_LANE = LANE_WIDTH / 2
HALF_TRAM_LANE = TRAM_LANE_WIDTH / 2

FPS = 60

CAR_LENGTH, CAR_WIDTH = 50, 30
TRAM_LENGTH, TRAM_WIDTH = 100, 20
BUS_LENGTH, BUS_WIDTH = 80, 30

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_GREY = (150, 150, 150)
COLOR_DARK_TRAM_ROAD = (80, 80, 80)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_BLUE_CAR = (0, 0, 200)
COLOR_PURPLE_TRAM = (150, 0, 150)
COLOR_DARK_BLUE_BUS = (0, 50, 150)
COLOR_LANE_MARKER = (255, 255, 255)
COLOR_TRAM_LIGHT_WHITE = (200, 200, 200)

COLLISION_BUFFER = 40
LIGHT_STOP_DISTANCE = 60
TRAFFIC_LIGHT_ACTIVATE_DISTANCE = 300

VEHICLE_SPEED = 4
VEHICLE_SPAWN_CHANCE = 0.015
BUS_PRIORITY_WEIGHT = 1
TRAM_PRIORITY_WEIGHT = 1

FIXED_GREEN_TIME = 22
MIN_GREEN_TIME_ADAPTIVE = 3
MAX_GREEN_TIME_ADAPTIVE = 40
YELLOW_TIME = 1
TRAM_GREEN_TIME_MULTIPLIER = 1
ADAPTIVE_QUEUE_COEFF = 0.3

CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2

CROSS_LEFT = CENTER_X - TOTAL_ROAD_WIDTH // 2
CROSS_RIGHT = CENTER_X + TOTAL_ROAD_WIDTH // 2
CROSS_TOP = CENTER_Y - TOTAL_ROAD_WIDTH // 2
CROSS_BOTTOM = CENTER_Y + TOTAL_ROAD_WIDTH // 2


def setup_lane_centers():
    centers = {}

    x_tram_n = CENTER_X + HALF_TRAM_LANE
    centers['N0T'] = {'x': x_tram_n, 'y_start': SCREEN_HEIGHT, 'type': 'tram'}
    x_tram_s = CENTER_X - HALF_TRAM_LANE
    centers['S0T'] = {'x': x_tram_s, 'y_start': 0, 'type': 'tram'}

    x_car_n = x_tram_n + HALF_TRAM_LANE + HALF_LANE
    centers['N0C'] = {'x': x_car_n, 'y_start': SCREEN_HEIGHT, 'type': 'car'}

    x_car_s = x_tram_s - HALF_TRAM_LANE - HALF_LANE
    centers['S0C'] = {'x': x_car_s, 'y_start': 0, 'type': 'car'}

    y_car_w = CENTER_Y + HALF_LANE
    centers[f'W0C'] = {'x_start': 0, 'y': y_car_w, 'type': 'car'}

    y_car_e = CENTER_Y - HALF_LANE
    centers[f'E0C'] = {'x_start': SCREEN_WIDTH, 'y': y_car_e, 'type': 'car'}

    return centers


LANE_CENTERS = setup_lane_centers()


class Vehicle(pygame.sprite.Sprite):

    def __init__(self, x, y, direction, lane_key, vehicle_type='car'):
        super().__init__()
        self.direction = direction
        self.lane_key = lane_key
        self.lane_type = lane_key[-1]
        self.vehicle_type = vehicle_type

        self.stopped = False
        self.stopped_by_light = False
        self.stopped_by_collision = False
        self.stopped_by_crossing_traffic = False
        self.wait_start_time = time.time()
        self.entered_cross = False
        self.speed = VEHICLE_SPEED

        if self.vehicle_type == 'tram':
            length, width = TRAM_LENGTH, TRAM_WIDTH
            color = COLOR_PURPLE_TRAM
        elif self.vehicle_type == 'bus':
            length, width = BUS_LENGTH, BUS_WIDTH
            color = COLOR_DARK_BLUE_BUS
        else:
            length, width = CAR_LENGTH, CAR_WIDTH
            color = COLOR_BLUE_CAR

        self.image = pygame.Surface((length, width))
        self.image.fill(color)

        angle = {'N': 90, 'S': -90, 'E': 180, 'W': 0}.get(direction, 0)
        self.image = pygame.transform.rotate(self.image, angle)

        self.rect = self.image.get_rect(center=(x, y))

    def check_entered_cross(self):
        is_on_cross = (
                self.rect.right > CROSS_LEFT and
                self.rect.left < CROSS_RIGHT and
                self.rect.bottom > CROSS_TOP and
                self.rect.top < CROSS_BOTTOM
        )
        self.entered_cross = is_on_cross

    def update(self, vehicles_ahead: List['Vehicle']):
        should_stop_for_collision = False
        for other in vehicles_ahead:
            distance = float('inf')

            if self.direction == 'N' and other.rect.bottom < self.rect.top:
                distance = self.rect.top - other.rect.bottom
            elif self.direction == 'S' and other.rect.top > self.rect.bottom:
                distance = other.rect.top - self.rect.bottom
            elif self.direction == 'W' and other.rect.left > self.rect.right:
                distance = other.rect.left - self.rect.right
            elif self.direction == 'E' and other.rect.right < self.rect.left:
                distance = self.rect.left - other.rect.right
            else:
                continue

            if distance < COLLISION_BUFFER:
                should_stop_for_collision = True
                break

        self.stopped_by_collision = should_stop_for_collision

        is_truly_stopped = self.stopped_by_light or self.stopped_by_collision

        if is_truly_stopped:
            if not self.stopped:
                self.wait_start_time = time.time()
            self.stopped = True
        else:
            self.stopped = False

        if not self.stopped:
            self.move_forward()

    def move_forward(self):
        if self.direction == 'N':
            self.rect.y -= self.speed
        elif self.direction == 'S':
            self.rect.y += self.speed
        elif self.direction == 'W':
            self.rect.x += self.speed
        elif self.direction == 'E':
            self.rect.x -= self.speed

    def set_stop_by_light(self, should_stop):
        self.stopped_by_light = should_stop

    def set_stop_by_crossing(self, should_stop):
        self.stopped_by_crossing_traffic = should_stop


class TramVehicle(Vehicle):

    def __init__(self, x, y, direction, lane_key):
        super().__init__(x, y, direction, lane_key, vehicle_type='tram')


class BusVehicle(Vehicle):

    def __init__(self, x, y, direction, lane_key):
        super().__init__(x, y, direction, lane_key, vehicle_type='bus')


class TrafficLight:

    def __init__(self, x, y, direction_type, direction):
        self.x = x
        self.y = y
        self.direction_type = direction_type
        self.direction = direction
        self.state = 'red'
        self.timer = 0.0
        self.last_green_duration = 0.0
        self.is_tram = False
        self.current_time_display = 0

    def update(self, dt):
        self.timer -= dt
        if self.timer < 0:
            self.timer = 0.0

        self.current_time_display = math.ceil(self.timer)
        if self.timer <= 0.01:
            self.current_time_display = 0

    def draw(self, screen):
        red_color = COLOR_RED if self.state == 'red' else (50, 0, 0)
        yellow_color = COLOR_YELLOW if self.state == 'yellow' else (50, 50, 0)
        green_color = COLOR_GREEN if self.state == 'green' else (0, 50, 0)

        if self.direction_type == 'vertical':
            pygame.draw.circle(screen, red_color, (self.x, self.y - 25), 10)
            pygame.draw.circle(screen, yellow_color, (self.x, self.y), 10)
            pygame.draw.circle(screen, green_color, (self.x, self.y + 25), 10)
            text_pos = (self.x + 15, self.y - 10)
        else:
            pygame.draw.circle(screen, red_color, (self.x + 25, self.y), 10)
            pygame.draw.circle(screen, yellow_color, (self.x, self.y), 10)
            pygame.draw.circle(screen, green_color, (self.x - 25, self.y), 10)
            text_pos = (self.x, self.y + 15)

        font = pygame.font.SysFont(None, 24)
        text = font.render(str(self.current_time_display), True, COLOR_WHITE)
        screen.blit(text, text_pos)


class TramTrafficLight(TrafficLight):

    def __init__(self, x, y, direction):
        super().__init__(x, y, 'vertical', direction)
        self.state = 'red'
        self.is_tram = True

    def draw(self, screen):

        RED_LIGHT = COLOR_RED
        INACTIVE_LIGHT_COLOR = (50, 50, 50)
        WHITE_LUNAR_LIGHT = COLOR_TRAM_LIGHT_WHITE

        sections = [
            (0, -15),
            (-15, 0),
            (15, 0),
            (0, 15),
        ]

        if self.state == 'green':
            for dx, dy in sections:
                pygame.draw.circle(screen, WHITE_LUNAR_LIGHT, (self.x + dx, self.y + dy), 8)
        else:
            for dx, dy in sections[0:3]:
                pygame.draw.circle(screen, RED_LIGHT, (self.x + dx, self.y + dy), 8)

            dx, dy = sections[3]
            pygame.draw.circle(screen, INACTIVE_LIGHT_COLOR, (self.x + dx, self.y + dy), 8)

        font = pygame.font.SysFont(None, 24)
        text = font.render(str(self.current_time_display), True, COLOR_WHITE)
        screen.blit(text, (self.x + 25, self.y - 10))


class Intersection:

    spawn_log: List[Dict[str, Any]] = []
    spawn_playback: List[Dict[str, Any]] = []
    spawn_index: int = 0
    dt_accumulator: float = 0.0

    def __init__(self, mode='fixed', intersection_type='standard', spawn_data: List[Dict[str, Any]] | None = None):
        self.mode = mode
        self.intersection_type = intersection_type
        self.vehicles = pygame.sprite.Group()
        self.stats = {'total_vehicles': 0, 'total_wait_time': 0.0,
                      'total_tram_wait_time': 0.0, 'total_bus_wait_time': 0.0}
        self.waiting_vehicles: Dict[int, float] = {}
        self.queues: Dict[str, List[Dict[str, Any]]] = {'N': [], 'S': [], 'E': [], 'W': []}

        self.light_stats = {'ns_cycles_count': 0, 'ew_cycles_count': 0,
                            'ns_total_green_time': 0.0, 'ew_total_green_time': 0.0,
                            'ns_tram_total_green_time': 0.0}

        self.is_crossing_occupied = False

        if spawn_data is not None:
            self.spawn_playback = spawn_data
            self.spawn_log = []
        else:
            self.spawn_playback = []
            self.spawn_log = []

        self._setup_traffic_lights()

    def _setup_traffic_lights(self):
        self.light_car_north = TrafficLight(LANE_CENTERS['N0C']['x'] + HALF_LANE, CROSS_BOTTOM - HALF_LANE, 'vertical',
                                            'N')
        self.light_car_south = TrafficLight(LANE_CENTERS['S0C']['x'] - HALF_LANE, CROSS_TOP + HALF_LANE, 'vertical',
                                            'S')
        self.lights_car_ns = [self.light_car_north, self.light_car_south]

        self.light_car_west = TrafficLight(CROSS_LEFT + HALF_LANE, LANE_CENTERS['W0C']['y'] + HALF_LANE, 'horizontal',
                                           'W')
        self.light_car_east = TrafficLight(CROSS_RIGHT - HALF_LANE, LANE_CENTERS['E0C']['y'] - HALF_LANE, 'horizontal',
                                           'E')
        self.lights_car_ew = [self.light_car_west, self.light_car_east]

        self.lights_tram_ns = []
        if self.intersection_type == 'tram_line':
            self.light_tram_north = TramTrafficLight(LANE_CENTERS['N0T']['x'] + HALF_TRAM_LANE,
                                                     CROSS_BOTTOM - HALF_TRAM_LANE, 'N')
            self.light_tram_south = TramTrafficLight(LANE_CENTERS['S0T']['x'] - HALF_TRAM_LANE,
                                                     CROSS_TOP + HALF_TRAM_LANE, 'S')
            self.lights_tram_ns = [self.light_tram_north, self.light_tram_south]

        green_time_car = FIXED_GREEN_TIME if self.mode == 'fixed' else MIN_GREEN_TIME_ADAPTIVE
        green_time_tram = green_time_car * TRAM_GREEN_TIME_MULTIPLIER

        for light in self.lights_car_ns:
            light.state = 'green'
            light.timer = green_time_car
            light.current_time_display = math.ceil(green_time_car)

        for light in self.lights_tram_ns:
            light.state = 'green'
            light.timer = green_time_tram
            light.current_time_display = math.ceil(green_time_tram)

        for light in self.lights_car_ew:
            light.state = 'red'
            light.timer = 0
            light.current_time_display = 0

    def _check_for_waiting_traffic(self, directions: List[str]) -> bool:
        has_waiting = False
        for d in directions:
            active_waiting = [v for v in self.vehicles
                              if v.direction == d and v.stopped_by_light and not v.entered_cross]
            if active_waiting:
                has_waiting = True

            pending_queue = [q for q in self.queues[d] if q['lane_key'].startswith(d)]
            if pending_queue:
                has_waiting = True

        return has_waiting

    def _lazy_switch_to_green(self, to_group: str, next_group: str):
        is_ns_phase = (to_group == 'ns_car')
        directions = ['N', 'S'] if is_ns_phase else ['E', 'W']

        traffic_waiting = self._check_for_waiting_traffic(directions)

        if is_ns_phase and self.lights_tram_ns:
            tram_directions = [d for d in directions if f'{d}0T' in LANE_CENTERS]
            if self._check_for_waiting_traffic(tram_directions):
                traffic_waiting = True

        if traffic_waiting or self.mode == 'fixed':
            self._switch_to_green(to_group)

            if is_ns_phase and self.lights_tram_ns:
                self._switch_to_green('ns_tram')
        else:
            self._switch_to_red(to_group)
            if to_group == 'ns_car' and self.lights_tram_ns:
                self._switch_to_red('ns_tram')

            self._switch_to_green(next_group)

            if next_group == 'ns_car' and self.lights_tram_ns:
                self._switch_to_green('ns_tram')

    def update(self, dt):
        self.dt_accumulator += dt
        self.process_queues()
        self.spawn_vehicles()

        all_lights = self.lights_car_ns + self.lights_car_ew + self.lights_tram_ns
        for light in all_lights:
            light.update(dt)

        if self.lights_car_ns[0].state == 'green' and self.lights_car_ns[0].timer <= 0:
            self._switch_to_yellow('ns_car')
            if self.lights_tram_ns:
                self._switch_to_yellow('ns_tram')

        elif self.lights_car_ew[0].state == 'green' and self.lights_car_ew[0].timer <= 0:
            self._switch_to_yellow('ew_car')

        elif self.lights_car_ns[0].state == 'yellow' and self.lights_car_ns[0].timer <= 0:
            self._switch_to_red('ns_car')
            if self.lights_tram_ns:
                self._switch_to_red('ns_tram')
            self._lazy_switch_to_green('ew_car', next_group='ns_car')

        elif self.lights_car_ew[0].state == 'yellow' and self.lights_car_ew[0].timer <= 0:
            self._switch_to_red('ew_car')
            self._lazy_switch_to_green('ns_car', next_group='ew_car')

        self.update_vehicles()

        self.is_crossing_occupied = any(v.entered_cross for v in self.vehicles)

    def _switch_to_yellow(self, from_group):
        lights_to_switch = []
        if from_group == 'ns_car':
            lights_to_switch = self.lights_car_ns
        elif from_group == 'ew_car':
            lights_to_switch = self.lights_car_ew
        elif from_group == 'ns_tram':
            lights_to_switch = self.lights_tram_ns

        for light in lights_to_switch:
            if light.state == 'green':
                light.state = 'yellow'
                light.timer = YELLOW_TIME
                light.last_green_duration = light.timer

    def _switch_to_red(self, to_group):
        lights_to_red = []
        if to_group == 'ns_car':
            lights_to_red = self.lights_car_ns
        elif to_group == 'ew_car':
            lights_to_red = self.lights_car_ew
        elif to_group == 'ns_tram':
            lights_to_red = self.lights_tram_ns

        for light in lights_to_red:
            if light.state != 'red':
                light.state = 'red'
                light.timer = 0

    def _calculate_adaptive_time(self, directions: List[str], light_group_name: str) -> float:

        car_queue_size = 0
        tram_queue_size = 0
        bus_queue_size = 0

        for d in directions:
            active_standing = [v for v in self.vehicles
                               if v.direction == d and v.stopped_by_light and not v.entered_cross]
            pending_queue = [v for v in self.queues[d]]

            for v in active_standing:
                if v.vehicle_type == 'car':
                    car_queue_size += 1
                elif v.vehicle_type == 'tram':
                    tram_queue_size += 1
                elif v.vehicle_type == 'bus':
                    bus_queue_size += 1

            for q in pending_queue:
                if q['vehicle_type'] == 'car':
                    car_queue_size += 1
                elif q['vehicle_type'] == 'tram':
                    tram_queue_size += 1
                elif q['vehicle_type'] == 'bus':
                    bus_queue_size += 1

        total_wait_cost = car_queue_size + \
                          TRAM_PRIORITY_WEIGHT * tram_queue_size + \
                          BUS_PRIORITY_WEIGHT * bus_queue_size

        new_time = MIN_GREEN_TIME_ADAPTIVE + ADAPTIVE_QUEUE_COEFF * total_wait_cost

        if 'tram' in light_group_name:
            new_time *= TRAM_GREEN_TIME_MULTIPLIER

        return min(new_time, MAX_GREEN_TIME_ADAPTIVE)

    def _switch_to_green(self, to_group):
        lights_to_green = []
        directions = []
        if to_group == 'ns_car':
            lights_to_green = self.lights_car_ns
            directions = ['N', 'S']
            self.light_stats['ns_cycles_count'] += 1
        elif to_group == 'ew_car':
            lights_to_green = self.lights_car_ew
            directions = ['E', 'W']
            self.light_stats['ew_cycles_count'] += 1
        elif to_group == 'ns_tram':
            lights_to_green = self.lights_tram_ns
            directions = ['N', 'S']

        if self.mode == 'fixed':
            green_time = FIXED_GREEN_TIME * (TRAM_GREEN_TIME_MULTIPLIER if 'tram' in to_group else 1)
        else:
            green_time = self._calculate_adaptive_time(directions, to_group)

        for light in lights_to_green:
            light.state = 'green'
            light.timer = green_time
            light.current_time_display = math.ceil(green_time)

            if to_group == 'ns_car':
                self.light_stats['ns_total_green_time'] += green_time
            elif to_group == 'ew_car':
                self.light_stats['ew_total_green_time'] += green_time
            elif to_group == 'ns_tram':
                self.light_stats['ns_tram_total_green_time'] += green_time

    def get_first_vehicle_in_lane(self, lane_key: str) -> Tuple[Vehicle | None, float]:
        direction = lane_key[0]
        vehicles_in_lane = [v for v in self.vehicles if v.lane_key == lane_key]

        first_vehicle = None
        min_distance_to_stop = float('inf')

        for vehicle in vehicles_in_lane:
            dist = float('inf')

            if direction == 'N':
                dist = vehicle.rect.top - CROSS_BOTTOM
            elif direction == 'S':
                dist = CROSS_TOP - vehicle.rect.bottom
            elif direction == 'W':
                dist = CROSS_LEFT - vehicle.rect.right
            elif direction == 'E':
                dist = vehicle.rect.left - CROSS_RIGHT
            else:
                continue

            if dist > 0 and dist < min_distance_to_stop:
                min_distance_to_stop = dist
                first_vehicle = vehicle

        return first_vehicle, min_distance_to_stop

    def update_vehicles(self):
        all_vehicles_list = list(self.vehicles)

        first_vehicles_data: Dict[str, Tuple[Vehicle | None, float]] = {}
        for lane_key in LANE_CENTERS.keys():
            first_vehicles_data[lane_key] = self.get_first_vehicle_in_lane(lane_key)

        for vehicle in all_vehicles_list:
            vehicle.check_entered_cross()

            current_light: TrafficLight
            if vehicle.direction in ['N', 'S']:
                if vehicle.lane_type == 'T' and self.intersection_type == 'tram_line':
                    current_light = self.light_tram_north if vehicle.direction == 'N' else self.light_tram_south
                else:
                    current_light = self.light_car_north if vehicle.direction == 'N' else self.light_car_south
            else:
                current_light = self.light_car_west if vehicle.direction == 'W' else self.light_car_east

            vehicles_in_same_lane = [v for v in all_vehicles_list
                                     if v != vehicle and v.lane_key == vehicle.lane_key]

            first_vehicle_in_lane, distance_to_cross_stop = first_vehicles_data.get(vehicle.lane_key,
                                                                                    (None, float('inf')))
            is_first_in_lane = first_vehicle_in_lane == vehicle

            should_stop = False

            if (current_light.state in ['red', 'yellow']) and not vehicle.entered_cross:

                is_ns_vehicle = vehicle.direction in ['N', 'S']
                is_crossing_blocked_for_me = any(
                    other_vehicle.entered_cross
                    for other_vehicle in all_vehicles_list
                    if (other_vehicle.direction in ['N', 'S']) != is_ns_vehicle
                )

                is_in_stop_zone = distance_to_cross_stop > 0 and distance_to_cross_stop < LIGHT_STOP_DISTANCE

                if is_first_in_lane and is_in_stop_zone:
                    should_stop = True

                elif is_crossing_blocked_for_me and is_first_in_lane and distance_to_cross_stop < CAR_LENGTH * 2:
                    should_stop = True

            vehicle.set_stop_by_light(should_stop)

            vehicle_id = id(vehicle)
            if vehicle.stopped_by_light and vehicle_id not in self.waiting_vehicles:
                self.waiting_vehicles[vehicle_id] = time.time()
            elif not vehicle.stopped_by_light and vehicle_id in self.waiting_vehicles:
                wait_time = time.time() - self.waiting_vehicles.pop(vehicle_id)
                self.stats['total_wait_time'] += wait_time
                if vehicle.vehicle_type == 'tram':
                    self.stats['total_tram_wait_time'] += wait_time
                elif vehicle.vehicle_type == 'bus':
                    self.stats['total_bus_wait_time'] += wait_time

            vehicle.update(vehicles_in_same_lane)

            if not (0 - 100 <= vehicle.rect.x < SCREEN_WIDTH + 100 and 0 - 100 <= vehicle.rect.y < SCREEN_HEIGHT + 100):
                self.stats['total_vehicles'] += 1
                if id(vehicle) in self.waiting_vehicles:
                    wait_time = time.time() - self.waiting_vehicles.pop(id(vehicle))
                    self.stats['total_wait_time'] += wait_time
                    if vehicle.vehicle_type == 'tram':
                        self.stats['total_tram_wait_time'] += wait_time
                    elif vehicle.vehicle_type == 'bus':
                        self.stats['total_bus_wait_time'] += wait_time
                vehicle.kill()

    def spawn_vehicles(self):

        if self.spawn_playback:
            while self.spawn_index < len(self.spawn_playback) and \
                    self.spawn_playback[self.spawn_index]['time'] <= self.dt_accumulator:
                spawn_data = self.spawn_playback[self.spawn_index]
                self.queues[spawn_data['direction']].append({
                    'vehicle_type': spawn_data['vehicle_type'],
                    'lane_key': spawn_data['lane_key']
                })
                self.spawn_index += 1
            return

        if random.random() < VEHICLE_SPAWN_CHANCE:

            direction = random.choice(['N', 'S', 'E', 'W'])
            lane_key = None
            vehicle_type = None

            if direction in ['N', 'S']:
                if self.intersection_type == 'tram_line':
                    lane_type_choice = random.choices(['car', 'bus', 'tram'], weights=[0.6, 0.2, 0.2], k=1)[0]
                else:
                    lane_type_choice = random.choices(['car', 'bus'], weights=[0.8, 0.2], k=1)[0]

                if lane_type_choice == 'tram':
                    vehicle_type = 'tram'
                    lane_key = f'{direction}0T'
                elif lane_type_choice == 'bus':
                    vehicle_type = 'bus'
                    lane_key = f'{direction}0C'
                else:
                    vehicle_type = 'car'
                    lane_key = f'{direction}0C'
            elif direction in ['E', 'W']:
                lane_type_choice = random.choices(['car', 'bus'], weights=[0.8, 0.2], k=1)[0]
                vehicle_type = lane_type_choice
                lane_key = f'{direction}0C'

            if lane_key in LANE_CENTERS:
                spawn_record = {
                    'time': self.dt_accumulator,
                    'direction': direction,
                    'vehicle_type': vehicle_type,
                    'lane_key': lane_key
                }

                self.spawn_log.append(spawn_record)

                self.queues[direction].append({
                    'vehicle_type': vehicle_type,
                    'lane_key': lane_key
                })

    def process_queues(self):

        for direction in ['N', 'S', 'E', 'W']:
            for lane_key in [k for k in LANE_CENTERS.keys() if k.startswith(direction)]:

                next_car_in_queue = next((car for car in self.queues[direction] if car['lane_key'] == lane_key), None)
                if next_car_in_queue is None:
                    continue

                vehicle_type = next_car_in_queue['vehicle_type']

                if vehicle_type == 'tram':
                    next_vehicle_length = TRAM_LENGTH
                elif vehicle_type == 'bus':
                    next_vehicle_length = BUS_LENGTH
                else:
                    next_vehicle_length = CAR_LENGTH

                active_vehicles = sorted([v for v in self.vehicles if v.lane_key == lane_key],
                                         key=lambda v: (
                                             v.rect.bottom if v.direction == 'N' else v.rect.top if v.direction == 'S' else v.rect.left if v.direction == 'W' else v.rect.right),
                                         reverse=(direction in ['N', 'E']))

                can_spawn = False
                if active_vehicles:
                    last_vehicle = active_vehicles[0]

                    if direction == 'N':
                        distance_to_edge = SCREEN_HEIGHT - last_vehicle.rect.bottom
                    elif direction == 'S':
                        distance_to_edge = last_vehicle.rect.top
                    elif direction == 'W':
                        distance_to_edge = last_vehicle.rect.left
                    elif direction == 'E':
                        distance_to_edge = SCREEN_WIDTH - last_vehicle.rect.right

                    if distance_to_edge > (next_vehicle_length + COLLISION_BUFFER * 2):
                        can_spawn = True
                else:
                    can_spawn = True

                if can_spawn:
                    self.queues[direction].remove(next_car_in_queue)

                    center_coords = LANE_CENTERS[lane_key]

                    if direction in ['N', 'S']:
                        y_offset = (next_vehicle_length // 2) * (1 if direction == 'S' else -1)
                        start_x = center_coords['x']
                        start_y = center_coords['y_start'] + y_offset
                    else:
                        x_offset = (next_vehicle_length // 2) * (1 if direction == 'W' else -1)
                        start_y = center_coords['y']
                        start_x = center_coords['x_start'] + x_offset

                    if vehicle_type == 'tram':
                        self.vehicles.add(TramVehicle(start_x, start_y, direction, lane_key))
                    elif vehicle_type == 'bus':
                        self.vehicles.add(BusVehicle(start_x, start_y, direction, lane_key))
                    else:
                        self.vehicles.add(Vehicle(start_x, start_y, direction, lane_key, vehicle_type))

                    break

    def draw(self, screen, current_sim_time: float):
        screen.fill(COLOR_BLACK)

        pygame.draw.rect(screen, COLOR_GREY, (CROSS_LEFT, 0, TOTAL_ROAD_WIDTH, SCREEN_HEIGHT))
        pygame.draw.rect(screen, COLOR_GREY, (0, CROSS_TOP, SCREEN_WIDTH, TOTAL_ROAD_WIDTH))

        if self.intersection_type == 'tram_line':
            tram_lane_left = CENTER_X - TRAM_LANE_WIDTH
            tram_lane_right = CENTER_X + TRAM_LANE_WIDTH
            pygame.draw.rect(screen, COLOR_DARK_TRAM_ROAD, (tram_lane_left, 0, TRAM_LANE_WIDTH * 2, SCREEN_HEIGHT))

            pygame.draw.line(screen, COLOR_BLACK, (CENTER_X - 5, 0), (CENTER_X - 5, SCREEN_HEIGHT), 2)
            pygame.draw.line(screen, COLOR_BLACK, (CENTER_X + 5, 0), (CENTER_X + 5, SCREEN_HEIGHT), 2)

        for x in [CENTER_X - TRAM_LANE_WIDTH, CENTER_X + TRAM_LANE_WIDTH,
                  LANE_CENTERS['N0C']['x'] + HALF_LANE, LANE_CENTERS['S0C']['x'] - HALF_LANE]:
            pygame.draw.line(screen, COLOR_LANE_MARKER, (x, 0), (x, SCREEN_HEIGHT), 2)

        pygame.draw.line(screen, COLOR_LANE_MARKER,
                         (0, CENTER_Y), (SCREEN_WIDTH, CENTER_Y), 4)

        pygame.draw.line(screen, COLOR_LANE_MARKER, (0, CENTER_Y - LANE_WIDTH), (SCREEN_WIDTH, CENTER_Y - LANE_WIDTH),
                         2)
        pygame.draw.line(screen, COLOR_LANE_MARKER, (0, CENTER_Y + LANE_WIDTH), (SCREEN_WIDTH, CENTER_Y + LANE_WIDTH),
                         2)

        pygame.draw.line(screen, COLOR_RED, (CROSS_LEFT, CROSS_BOTTOM), (CROSS_RIGHT, CROSS_BOTTOM), 4)
        pygame.draw.line(screen, COLOR_RED, (CROSS_LEFT, CROSS_TOP), (CROSS_RIGHT, CROSS_TOP), 4)
        pygame.draw.line(screen, COLOR_RED, (CROSS_LEFT, CROSS_TOP), (CROSS_LEFT, CROSS_BOTTOM), 4)
        pygame.draw.line(screen, COLOR_RED, (CROSS_RIGHT, CROSS_TOP), (CROSS_RIGHT, CROSS_BOTTOM), 4)

        self.vehicles.draw(screen)

        for light in self.lights_car_ns + self.lights_car_ew + self.lights_tram_ns:
            light.draw(screen)

        font = pygame.font.SysFont(None, 26)

        time_text = font.render(
            f"Время: {current_sim_time:.2f} с / {SIMULATION_DURATION} с | Режим: {self.mode.capitalize()}",
            True, COLOR_WHITE)
        screen.blit(time_text, (10, 10))

        occupancy_text = font.render(
            f"Перекресток: {self.intersection_type.capitalize()} | Занят: {'ДА' if self.is_crossing_occupied else 'НЕТ'}",
            True, COLOR_WHITE)
        screen.blit(occupancy_text, (SCREEN_WIDTH - 320, 10))


def run_simulation(mode, intersection_type, spawn_data=None):
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption(f"Симуляция светофора ({intersection_type.capitalize()} / {mode.capitalize()} режим)")
    clock = pygame.time.Clock()

    intersection = Intersection(mode=mode, intersection_type=intersection_type, spawn_data=spawn_data)

    start_time = time.time()
    running = True

    while running and (time.time() - start_time) < SIMULATION_DURATION:
        dt = clock.tick(FPS) / 1000.0
        current_sim_time = time.time() - start_time

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        intersection.update(dt)

        intersection.draw(screen, current_sim_time)
        pygame.display.flip()

    stats = intersection.stats
    stats['ns_avg_green_time'] = intersection.light_stats['ns_total_green_time'] / intersection.light_stats[
        'ns_cycles_count'] if intersection.light_stats['ns_cycles_count'] > 0 else 0
    stats['ew_avg_green_time'] = intersection.light_stats['ew_total_green_time'] / intersection.light_stats[
        'ew_cycles_count'] if intersection.light_stats['ew_cycles_count'] > 0 else 0
    stats['ns_tram_avg_green_time'] = intersection.light_stats['ns_tram_total_green_time'] / intersection.light_stats[
        'ns_cycles_count'] if intersection.light_stats['ns_cycles_count'] > 0 else 0

    spawn_log = intersection.spawn_log if mode == 'fixed' else None

    pygame.quit()
    return stats, spawn_log


if __name__ == '__main__':
    INTERSECTION_TYPE = 'tram_line'
    FIXED_SPAWN_LOG = None

    print(f"Запускаем симуляцию для перекрестка '{INTERSECTION_TYPE}'...")

    random.seed(42)

    print("\n--- 1. ФИКСИРОВАННЫЙ режим (СЛУЧАЙНЫЙ, но записанный трафик) ---")
    fixed_stats, FIXED_SPAWN_LOG = run_simulation('fixed', INTERSECTION_TYPE)
    print("Симуляция в фиксированном режиме завершена.")
    print(f"Записано {len(FIXED_SPAWN_LOG)} событий спавна для второго запуска.")

    random.seed(None)

    print("\n--- 2. АДАПТИВНЫЙ режим (Воспроизведение трафика из первого запуска) ---")
    adaptive_stats, _ = run_simulation('adaptive', INTERSECTION_TYPE, spawn_data=FIXED_SPAWN_LOG)
    print("Симуляция в адаптивном режиме завершена.")

    print(
        f"\n\n--- РЕЗУЛЬТАТЫ СРАВНЕНИЯ (Длительность: {SIMULATION_DURATION} сек | Кол-во ТС: {fixed_stats['total_vehicles']}) ---")


    def format_stats(stats, mode, total_time):
        total_vehicles = stats.get('total_vehicles', 0)
        total_wait = stats.get('total_wait_time', 0.0)
        total_tram_wait = stats.get('total_tram_wait_time', 0.0)
        total_bus_wait = stats.get('total_bus_wait_time', 0.0)

        num_tram_finished = len([v for v in FIXED_SPAWN_LOG if
                                 v['vehicle_type'] == 'tram'])
        num_bus_finished = len([v for v in FIXED_SPAWN_LOG if v['vehicle_type'] == 'bus'])

        avg_wait = total_wait / total_vehicles if total_vehicles > 0 else 0
        avg_tram_wait = total_tram_wait / num_tram_finished if num_tram_finished > 0 else 0
        avg_bus_wait = total_bus_wait / num_bus_finished if num_bus_finished > 0 else 0

        print(f"\n[{mode.capitalize()} режим | Перекресток: {INTERSECTION_TYPE.capitalize()}]")
        print(f"  - Всего проехало ТС: {total_vehicles}")
        print(f"  - СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ (ВСЕ ТС): {avg_wait:.2f} сек/ТС")
        print(f"  - СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ ТРАМВАЕВ: {avg_tram_wait:.2f} сек/ТС")
        print(f"  - СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ АВТОБУСОВ: {avg_bus_wait:.2f} сек/ТС")
        print(f"  - СРЕДНИЙ ЦИКЛ ЗЕЛЕНОГО СВЕТА (NS Car): {stats.get('ns_avg_green_time', 0):.2f} сек")
        print(f"  - СРЕДНИЙ ЦИКЛ ЗЕЛЕНОГО СВЕТА (EW Car): {stats.get('ew_avg_green_time', 0):.2f} сек")
        if INTERSECTION_TYPE == 'tram_line':
            print(f"  - СРЕДНИЙ ЦИКЛ ЗЕЛЕНОГО СВЕТА (NS Tram): {stats.get('ns_tram_avg_green_time', 0):.2f} сек")

        return total_vehicles, avg_wait, avg_tram_wait, avg_bus_wait


    total_fixed, avg_wait_fixed, avg_tram_fixed, avg_bus_fixed = format_stats(fixed_stats, 'фиксированный',
                                                                              SIMULATION_DURATION)
    total_adaptive, avg_wait_adaptive, avg_tram_adaptive, avg_bus_adaptive = format_stats(adaptive_stats, 'адаптивный',
                                                                                          SIMULATION_DURATION)

    if total_fixed > 0:
        improvement_wait = ((avg_wait_fixed - avg_wait_adaptive) / avg_wait_fixed) * 100 if avg_wait_fixed > 0 else 0
        improvement_tram = ((avg_tram_fixed - avg_tram_adaptive) / avg_tram_fixed) * 100 if avg_tram_fixed > 0 else 0
        improvement_bus = ((avg_bus_fixed - avg_bus_adaptive) / avg_bus_fixed) * 100 if avg_bus_fixed > 0 else 0

        print("\n--- ВЫВОД (ЭФФЕКТИВНОСТЬ АДАПТИВНОЙ СИСТЕМЫ при ОДИНАКОВОМ ТРАФИКЕ) ---")
        print(f"1. Адаптивная система позволила сократить СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ (ВСЕ ТС) на {improvement_wait:.2f}%.")
        if improvement_tram != 0:
            print(
                f"2. Адаптивная система позволила сократить СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ (ТРАМВАЕВ) на {improvement_tram:.2f}%.")
        if improvement_bus != 0:
            print(
                f"3. Адаптивная система позволила сократить СРЕДНЕЕ ВРЕМЯ ОЖИДАНИЯ (АВТОБУСОВ) на {improvement_bus:.2f}%.")
    else:
        print("\nНе удалось сравнить результаты, так как в одной из симуляций не было движения.")
    print("\n\n--- ИДЕЯ МАСШТАБИРОВАНИЯ СИМУЛЯЦИИ НА СЛОЖНЫЕ РОССИЙСКИЕ ПЕРЕКРЕСТКИ ---")
    print(
        "Для распространения этой симуляции на большинство перекрестков в России (с несколькими полосами, поворотными фазами и сложной логикой) необходимо перейти от простой двухфазной системы (NS/EW) к МОДУЛЬНОЙ МНОГОФАЗНОЙ СИСТЕМЕ УПРАВЛЕНИЯ:")
    print(
        "1. **Введение 'Фазы' как Объекта:** Вместо жестких 'ns_car'/'ew_car' определить объект TrafficPhase (например, 'N_прямо', 'E_поворот_налево'). Каждая фаза должна знать:")
    print("   - Какие полосы (lane_key) она активирует (зеленый).")
    print("   - Какие полосы (lane_key) она блокирует (красный/конфликт).")
    print("   - Минимальное и максимальное время работы.")
    print("2. **Адаптивный Выбор Фазы (Скоррестированное Адаптивное Управление):**")
    print(
        "   - Калькуляция 'Суммарного спроса' для каждой ПОТЕНЦИАЛЬНОЙ фазы (например, сумма очереди на полосах, которые она активирует, с учетом приоритета ОТ).")
    print(
        "   - Использование алгоритма, выбирающего СЛЕДУЮЩУЮ фазу, которая имеет наибольший спрос, но не нарушает минимальный цикл предыдущей фазы (VAP - Vehicle Actuated Programming).")
    print(
        "   - Добавление 'Защищенных Поворотов': фазы поворота налево должны иметь отдельный подсчет спроса и запускаться только при наличии машин на поворотной полосе.")
    print(
        "3. **Расширенная Логика 'Занятости Перекрестка':** Ввести зонирование для конфликтов. Вместо одного флага is_crossing_occupied использовать матрицу конфликтов (Conflict Matrix) между полосами. Это позволит ТС двигаться, если их траектория не пересекается с фазой, идущей в данный момент, что критически важно для многополосных перекрестков и пересечений трамвайных путей.")
    print(
        "4. **Моделирование Разметки и Траектории:** У каждой машины должна быть не только direction (N, S, E, W), но и turn_intent (straight, left, right), что влияет на то, в какой полосе она должна стоять и с какой фазой светофора она взаимодействует.")

    print("\n\n*** ПЕРСПЕКТИВЫ РАЗВИТИЯ И ОЦЕНКА СЛОЖНОСТИ ***")
    print(
        "Текущая симуляция демонстрирует высокий потенциал адаптивного управления, но ее архитектура, основанная на жестких двухфазных циклах (NS/EW), не позволяет гибко управлять сложными поворотными фазами.")
    print(
        "Главная трудность для масштабирования - это отсутствие модульной системы фаз и детального моделирования траекторий поворота (пункт 1 и 4). Реализация этих элементов требует значительного времени на переработку базовой логики перекрестка и класса Vehicle.")
    print(
        "Мы считаем, что данный уровень сложности, а именно создание гибкого многофазного контроллера и полная переработка системы коллизий и траекторий, является задачей, требующей знаний и опыта, выходящих за рамки учебной программы 2-го курса.")
    print(
        "Это можно будет пофиксить в будущем, выделив ресурсы на полный рефакторинг с целью внедрения Модульной Многофазной Системы.")
