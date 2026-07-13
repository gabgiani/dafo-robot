from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import threading
import time
from typing import Any
import zlib

import msgpack
import numpy as np
import zmq


@dataclass(frozen=True)
class DepthFrame:
    received_at: float
    depth: np.ndarray
    fx: float
    fy: float
    cx: float
    cy: float
    camera_position: np.ndarray
    camera_rotation: np.ndarray


@dataclass(frozen=True)
class OccupancyGrid:
    occupied: np.ndarray
    origin: np.ndarray
    resolution: float

    def world_to_cell(self, point: np.ndarray) -> tuple[int, int]:
        cell_xy = np.floor((point[:2] - self.origin) / self.resolution).astype(int)
        return int(cell_xy[1]), int(cell_xy[0])

    def cell_to_world(self, cell: tuple[int, int]) -> np.ndarray:
        row, column = cell
        return self.origin + (np.array([column, row], dtype=float) + 0.5) * self.resolution


def decode_depth(payload: bytes) -> DepthFrame:
    message: dict[str, Any] = msgpack.unpackb(payload, raw=False)
    if message.get("topic") != "sonic_depth" or message.get("dtype") != "float32":
        raise ValueError("Paquete de profundidad SONIC no valido")
    shape = tuple(int(value) for value in message["shape"])
    if len(shape) != 2 or min(shape) <= 0:
        raise ValueError(f"Forma de profundidad no valida: {shape}")
    depth = np.frombuffer(zlib.decompress(message["depth_zlib"]), dtype=np.float32)
    if depth.size != shape[0] * shape[1]:
        raise ValueError("El payload depth no coincide con su forma")
    camera_position = np.asarray(message["camera_position"], dtype=float)
    camera_rotation = np.asarray(message["camera_rotation"], dtype=float)
    if camera_position.shape != (3,) or camera_rotation.shape != (3, 3):
        raise ValueError("Calibracion extrinseca de camera no valida")
    return DepthFrame(
        received_at=time.monotonic(),
        depth=depth.reshape(shape),
        fx=float(message["fx"]),
        fy=float(message["fy"]),
        cx=float(message["cx"]),
        cy=float(message["cy"]),
        camera_position=camera_position,
        camera_rotation=camera_rotation,
    )


class DepthSubscriber:
    def __init__(self, url: str):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.RCVHWM, 1)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.connect(url)
        self._lock = threading.Lock()
        self._latest: DepthFrame | None = None
        self._error: Exception | None = None
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._receive, name="sonic-depth", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def latest(self) -> DepthFrame | None:
        with self._lock:
            return self._latest

    def error(self) -> Exception | None:
        with self._lock:
            return self._error

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=1.0)
        self._socket.close(linger=0)
        self._context.term()

    def _receive(self) -> None:
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)
        try:
            while not self._stopping.is_set():
                if self._socket not in dict(poller.poll(100)):
                    continue
                frame = decode_depth(self._socket.recv())
                with self._lock:
                    self._latest = frame
        except Exception as exc:
            with self._lock:
                self._error = exc


def depth_to_world_points(
    frame: DepthFrame,
    *,
    max_depth: float,
    min_height: float = 0.12,
    max_height: float = 1.6,
    stride: int = 4,
) -> np.ndarray:
    rows, columns = np.indices(frame.depth.shape)
    rows = rows[::stride, ::stride].reshape(-1)
    columns = columns[::stride, ::stride].reshape(-1)
    depth = frame.depth[::stride, ::stride].reshape(-1).astype(float)
    valid = np.isfinite(depth) & (depth > 0.1) & (depth <= max_depth)
    depth = depth[valid]
    rows = rows[valid]
    columns = columns[valid]
    camera_points = np.column_stack(
        (
            (columns - frame.cx) * depth / frame.fx,
            -(rows - frame.cy) * depth / frame.fy,
            -depth,
        )
    )
    world_points = camera_points @ frame.camera_rotation.T + frame.camera_position
    height_mask = (world_points[:, 2] >= min_height) & (world_points[:, 2] <= max_height)
    return world_points[height_mask]


def build_occupancy_grid(
    points: np.ndarray,
    start: np.ndarray,
    goal: np.ndarray,
    *,
    resolution: float,
    inflation_radius: float,
    margin: float = 1.5,
) -> OccupancyGrid:
    lower = np.minimum(start[:2], goal[:2]) - margin
    upper = np.maximum(start[:2], goal[:2]) + margin
    shape_xy = np.ceil((upper - lower) / resolution).astype(int) + 1
    occupied = np.zeros((int(shape_xy[1]), int(shape_xy[0])), dtype=bool)
    if points.size:
        cells_xy = np.floor((points[:, :2] - lower) / resolution).astype(int)
        valid = (
            (cells_xy[:, 0] >= 0)
            & (cells_xy[:, 0] < occupied.shape[1])
            & (cells_xy[:, 1] >= 0)
            & (cells_xy[:, 1] < occupied.shape[0])
        )
        occupied[cells_xy[valid, 1], cells_xy[valid, 0]] = True

    inflation_cells = int(math.ceil(inflation_radius / resolution))
    source = occupied.copy()
    for row_offset in range(-inflation_cells, inflation_cells + 1):
        for column_offset in range(-inflation_cells, inflation_cells + 1):
            if math.hypot(row_offset, column_offset) > inflation_cells:
                continue
            source_rows, source_columns = np.nonzero(source)
            target_rows = source_rows + row_offset
            target_columns = source_columns + column_offset
            valid = (
                (target_rows >= 0)
                & (target_rows < occupied.shape[0])
                & (target_columns >= 0)
                & (target_columns < occupied.shape[1])
            )
            occupied[target_rows[valid], target_columns[valid]] = True

    grid = OccupancyGrid(occupied=occupied, origin=lower, resolution=resolution)
    start_row, start_column = grid.world_to_cell(start)
    for row_offset in range(-inflation_cells, inflation_cells + 1):
        for column_offset in range(-inflation_cells, inflation_cells + 1):
            if math.hypot(row_offset, column_offset) > inflation_cells:
                continue
            row = start_row + row_offset
            column = start_column + column_offset
            if 0 <= row < occupied.shape[0] and 0 <= column < occupied.shape[1]:
                occupied[row, column] = False
    goal_row, goal_column = grid.world_to_cell(goal)
    if 0 <= goal_row < occupied.shape[0] and 0 <= goal_column < occupied.shape[1]:
        occupied[goal_row, goal_column] = False
    return grid


def astar(grid: OccupancyGrid, start: np.ndarray, goal: np.ndarray) -> list[np.ndarray]:
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    height, width = grid.occupied.shape
    for cell, name in ((start_cell, "inicio"), (goal_cell, "objetivo")):
        if not (0 <= cell[0] < height and 0 <= cell[1] < width):
            raise ValueError(f"El {name} queda fuera del mapa")

    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
    cost_so_far = {start_cell: 0.0}
    neighbors = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal_cell:
            break
        for row_offset, column_offset, movement_cost in neighbors:
            neighbor = (current[0] + row_offset, current[1] + column_offset)
            if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                continue
            if grid.occupied[neighbor]:
                continue
            if row_offset and column_offset:
                if grid.occupied[current[0] + row_offset, current[1]] or grid.occupied[
                    current[0], current[1] + column_offset
                ]:
                    continue
            new_cost = cost_so_far[current] + movement_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heuristic = math.hypot(goal_cell[0] - neighbor[0], goal_cell[1] - neighbor[1])
                heapq.heappush(frontier, (new_cost + heuristic, neighbor))
                came_from[neighbor] = current

    if goal_cell not in came_from:
        return []
    cells = []
    current: tuple[int, int] | None = goal_cell
    while current is not None:
        cells.append(current)
        current = came_from[current]
    cells.reverse()
    return [grid.cell_to_world(cell) for cell in cells]


def select_waypoint(path: list[np.ndarray], position: np.ndarray, lookahead: float) -> np.ndarray:
    if not path:
        raise ValueError("No existe una ruta para seleccionar waypoint")
    waypoint = path[-1]
    for candidate in path:
        if np.linalg.norm(candidate - position[:2]) >= lookahead:
            waypoint = candidate
            break
    return waypoint