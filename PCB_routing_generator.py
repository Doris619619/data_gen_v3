"""
PCB Router - A* 基于网格的布线器
支持 8 方向走线（0°, 45°, 90° 等）、双层布线、过孔
正确处理焊盘尺寸：线连接到焊盘边缘,避开其他焊盘
过孔必须与焊盘保持最小距离
包含UCG导出功能，真实反映布线顺序
"""

import math
import heapq
from typing import Any, List, Dict, Tuple, Set, Optional
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict


class Direction(Enum):
    """8 个走线方向"""
    E = (1, 0)      # 0°
    NE = (1, 1)     # 45°
    N = (0, 1)      # 90°
    NW = (-1, 1)    # 135°
    W = (-1, 0)     # 180°
    SW = (-1, -1)   # 225°
    S = (0, -1)     # 270°
    SE = (1, -1)    # 315°


@dataclass
class GridPoint:
    """网格点"""
    x: int
    y: int
    layer: int
    
    def __hash__(self):
        return hash((self.x, self.y, self.layer))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.layer == other.layer
    
    def __lt__(self, other):
        return (self.x, self.y, self.layer) < (other.x, other.y, other.layer)


@dataclass
class Segment:
    """走线段"""
    start: Tuple[float, float]
    end: Tuple[float, float]
    layer: int
    width: float


@dataclass
class Via:
    """过孔"""
    x: float
    y: float
    layers: Tuple[int, int]  # (from_layer, to_layer)
    diameter: float


@dataclass
class PadInfo:
    """焊盘信息"""
    center_x: float
    center_y: float
    width: float
    height: float
    layer: int
    component_id: str
    pin_id: str
    net: str = ""  # 网络名称
    micro_id: str = ""  # 所属Micro
    rotation: float = 0.0  # 焊盘旋转角度
    
    @property
    def pad_key(self) -> str:
        return f"{self.component_id}_{self.pin_id}"
    
    def get_nearest_edge_point(self, from_x: float, from_y: float) -> Tuple[float, float]:
        half_w = self.width / 2
        half_h = self.height / 2
        left = self.center_x - half_w
        right = self.center_x + half_w
        bottom = self.center_y - half_h
        top = self.center_y + half_h
        
        if left <= from_x <= right and bottom <= from_y <= top:
            return (self.center_x, self.center_y)
        
        nearest_x = max(left, min(right, from_x))
        nearest_y = max(bottom, min(top, from_y))
        return (nearest_x, nearest_y)
    
    def get_connection_point_towards(self, target_x: float, target_y: float) -> Tuple[float, float]:
        return self.get_nearest_edge_point(target_x, target_y)
    
    def contains_point(self, x: float, y: float, margin: float = 0) -> bool:
        half_w = self.width / 2 + margin
        half_h = self.height / 2 + margin
        return (abs(x - self.center_x) <= half_w and 
                abs(y - self.center_y) <= half_h)
    
    def distance_to_point(self, x: float, y: float) -> float:
        half_w = self.width / 2
        half_h = self.height / 2
        dx = abs(x - self.center_x)
        dy = abs(y - self.center_y)
        
        if dx <= half_w and dy <= half_h:
            return -min(half_w - dx, half_h - dy)
        if dx <= half_w:
            return dy - half_h
        if dy <= half_h:
            return dx - half_w
        return math.sqrt((dx - half_w)**2 + (dy - half_h)**2)


@dataclass
class RoutedConnection:
    """记录单条布线连接的详细信息（用于UCG导出）"""
    net_id: str
    source_component: str
    source_pin: str
    source_micro: str
    target_component: str
    target_pin: str
    target_micro: str
    segments: List[Segment]
    vias: List[Via]
    routing_order: int  # 布线顺序（第几条布的线）


@dataclass
class RoutingResult:
    """单条边的布线结果"""
    success: bool
    segments: List[Segment] = field(default_factory=list)
    vias: List[Via] = field(default_factory=list)
    total_length: float = 0.0
    num_vias: int = 0


@dataclass
class RouterConfig:
    """布线器配置"""
    grid_size: float = 0.2
    num_layers: int = 2
    trace_width: float = 0.15
    via_diameter: float = 0.6
    via_drill: float = 0.3
    
    clearance_trace_trace: float = 0.1
    clearance_trace_pad: float = 0.1
    clearance_trace_via: float = 0.1
    clearance_via_via: float = 0.1
    clearance_trace_boundary: float = 0.1
    clearance_via_pad: float = 0.5
    
    via_cost: float = 5
    layer_change_cost: float = 5.0
    direction_change_cost: float = 0.1
    
    max_iterations: int = 200000
    pad_layer: int = 0


class ObstacleMap:
    """障碍物地图管理"""
    
    def __init__(self, config: RouterConfig, canvas_width: float, canvas_height: float):
        self.config = config
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.grid_size = config.grid_size
        
        self.grid_width = int(math.ceil(canvas_width / self.grid_size)) + 1
        self.grid_height = int(math.ceil(canvas_height / self.grid_size)) + 1
        
        self.blocked: Dict[int, Set[Tuple[int, int]]] = defaultdict(set)
        self.via_blocked: Set[Tuple[int, int]] = set()
        self.pads: Dict[str, PadInfo] = {}
        self.pad_grid_points: Dict[str, List[Tuple[int, int, int]]] = {}
        self.routed_segments: List[Segment] = []
        self.routed_vias: List[Via] = []
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round(x / self.grid_size))
        gy = int(round(y / self.grid_size))
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return gx * self.grid_size, gy * self.grid_size
    
    def is_valid_grid(self, gx: int, gy: int) -> bool:
        margin = int(math.ceil(self.config.clearance_trace_boundary / self.grid_size))
        return (margin <= gx < self.grid_width - margin and 
                margin <= gy < self.grid_height - margin)
    
    def add_rectangle_obstacle(self, x1: float, y1: float, x2: float, y2: float, 
                                layer: int, clearance: float = 0):
        x1 -= clearance
        y1 -= clearance
        x2 += clearance
        y2 += clearance
        
        gx1, gy1 = self.world_to_grid(x1, y1)
        gx2, gy2 = self.world_to_grid(x2, y2)
        
        for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
            for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
                self.blocked[layer].add((gx, gy))
    
    def add_circle_obstacle(self, cx: float, cy: float, radius: float, 
                            layer: int, clearance: float = 0):
        total_radius = radius + clearance
        gcx, gcy = self.world_to_grid(cx, cy)
        grid_radius = int(math.ceil(total_radius / self.grid_size))
        
        for gx in range(gcx - grid_radius, gcx + grid_radius + 1):
            for gy in range(gcy - grid_radius, gcy + grid_radius + 1):
                wx, wy = self.grid_to_world(gx, gy)
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= total_radius ** 2:
                    self.blocked[layer].add((gx, gy))
    
    def add_line_obstacle(self, x1: float, y1: float, x2: float, y2: float,
                          width: float, layer: int, clearance: float = 0):
        total_width = width / 2 + clearance
        
        gx1, gy1 = self.world_to_grid(x1, y1)
        gx2, gy2 = self.world_to_grid(x2, y2)
        
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        
        if dx == 0 and dy == 0:
            self._add_point_with_width(gx1, gy1, total_width, layer)
            return
        
        if dx >= dy:
            err = dx / 2
            gy = gy1
            for gx in range(gx1, gx2 + sx, sx):
                self._add_point_with_width(gx, gy, total_width, layer)
                err -= dy
                if err < 0:
                    gy += sy
                    err += dx
        else:
            err = dy / 2
            gx = gx1
            for gy in range(gy1, gy2 + sy, sy):
                self._add_point_with_width(gx, gy, total_width, layer)
                err -= dx
                if err < 0:
                    gx += sx
                    err += dy
    
    def _add_point_with_width(self, gx: int, gy: int, width: float, layer: int):
        grid_width = int(math.ceil(width / self.grid_size))
        for dx in range(-grid_width, grid_width + 1):
            for dy in range(-grid_width, grid_width + 1):
                if dx * dx + dy * dy <= grid_width * grid_width:
                    self.blocked[layer].add((gx + dx, gy + dy))
    
    def add_via_exclusion_zone(self, cx: float, cy: float, radius: float):
        gcx, gcy = self.world_to_grid(cx, cy)
        grid_radius = int(math.ceil(radius / self.grid_size))
        
        for gx in range(gcx - grid_radius, gcx + grid_radius + 1):
            for gy in range(gcy - grid_radius, gcy + grid_radius + 1):
                wx, wy = self.grid_to_world(gx, gy)
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= radius ** 2:
                    self.via_blocked.add((gx, gy))
    
    def add_rectangular_via_exclusion_zone(self, cx: float, cy: float, 
                                            width: float, height: float, 
                                            margin: float):
        half_w = width / 2 + margin
        half_h = height / 2 + margin
        
        gx1, gy1 = self.world_to_grid(cx - half_w, cy - half_h)
        gx2, gy2 = self.world_to_grid(cx + half_w, cy + half_h)
        
        for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
            for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
                self.via_blocked.add((gx, gy))
    
    def add_pad(self, pad_info: PadInfo, clearance: float = 0):
        pad_key = pad_info.pad_key
        self.pads[pad_key] = pad_info
        
        half_w = pad_info.width / 2 + clearance
        half_h = pad_info.height / 2 + clearance
        
        gx1, gy1 = self.world_to_grid(pad_info.center_x - half_w, pad_info.center_y - half_h)
        gx2, gy2 = self.world_to_grid(pad_info.center_x + half_w, pad_info.center_y + half_h)
        
        pad_grids = []
        layer = pad_info.layer
        for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
            for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
                self.blocked[layer].add((gx, gy))
                pad_grids.append((gx, gy, layer))
        
        self.pad_grid_points[pad_key] = pad_grids
        
        self.add_rectangular_via_exclusion_zone(
            pad_info.center_x, pad_info.center_y,
            pad_info.width, pad_info.height,
            self.config.clearance_via_pad
        )
    
    def add_via_obstacle(self, via: Via, clearance: float = 0):
        radius = via.diameter / 2
        for layer in range(self.config.num_layers):
            self.add_circle_obstacle(via.x, via.y, radius, layer, clearance)
        self.add_via_exclusion_zone(via.x, via.y, 
                                    via.diameter / 2 + self.config.clearance_via_via)
    
    def temporarily_remove_pads(self, pad_keys: List[str]) -> Dict[str, List[Tuple[int, int, int]]]:
        removed = {}
        for key in pad_keys:
            if key in self.pad_grid_points:
                removed[key] = self.pad_grid_points[key]
                for gx, gy, layer in self.pad_grid_points[key]:
                    self.blocked[layer].discard((gx, gy))
        return removed
    
    def restore_pads(self, removed: Dict[str, List[Tuple[int, int, int]]]):
        for key, grids in removed.items():
            for gx, gy, layer in grids:
                self.blocked[layer].add((gx, gy))
    
    def is_blocked(self, gx: int, gy: int, layer: int) -> bool:
        return (gx, gy) in self.blocked[layer]
    
    def is_via_blocked(self, gx: int, gy: int) -> bool:
        return (gx, gy) in self.via_blocked
    
    def can_place_via(self, gx: int, gy: int) -> bool:
        if not self.is_valid_grid(gx, gy):
            return False
        if self.is_via_blocked(gx, gy):
            return False
        for layer in range(self.config.num_layers):
            if self.is_blocked(gx, gy, layer):
                return False
        return True
    
    def get_pad_info(self, pad_key: str) -> Optional[PadInfo]:
        return self.pads.get(pad_key)
    
    def add_routed_segment(self, segment: Segment):
        self.routed_segments.append(segment)
        self.add_line_obstacle(
            segment.start[0], segment.start[1],
            segment.end[0], segment.end[1],
            segment.width, segment.layer,
            self.config.clearance_trace_trace
        )
    
    def add_routed_via(self, via: Via):
        self.routed_vias.append(via)
        self.add_via_obstacle(via, self.config.clearance_via_via)


class AStarRouter:
    """A* 布线算法"""
    
    def __init__(self, obstacle_map: ObstacleMap, config: RouterConfig):
        self.obstacle_map = obstacle_map
        self.config = config
        
        self.directions = [
            (1, 0), (1, 1), (0, 1), (-1, 1),
            (-1, 0), (-1, -1), (0, -1), (1, -1),
        ]
    
    def route(self, source_pad: PadInfo, target_pad: PadInfo,
              pad_keys_to_exclude: List[str] = None) -> RoutingResult:
        removed_pads = {}
        if pad_keys_to_exclude:
            removed_pads = self.obstacle_map.temporarily_remove_pads(pad_keys_to_exclude)
        
        try:
            result = self._astar_search(source_pad, target_pad)
            return result
        finally:
            if removed_pads:
                self.obstacle_map.restore_pads(removed_pads)
    
    def _astar_search(self, source_pad: PadInfo, target_pad: PadInfo) -> RoutingResult:
        om = self.obstacle_map
        cfg = self.config
        pad_layer = cfg.pad_layer
        
        source_conn = source_pad.get_connection_point_towards(
            target_pad.center_x, target_pad.center_y
        )
        target_conn = target_pad.get_connection_point_towards(
            source_pad.center_x, source_pad.center_y
        )
        
        start_gx, start_gy = om.world_to_grid(source_conn[0], source_conn[1])
        end_gx, end_gy = om.world_to_grid(target_conn[0], target_conn[1])
        
        start = GridPoint(start_gx, start_gy, pad_layer)
        end = GridPoint(end_gx, end_gy, pad_layer)
        
        # if not om.is_valid_grid(start_gx, start_gy):
        #     return RoutingResult(success=False)
        # if not om.is_valid_grid(end_gx, end_gy):
        #     return RoutingResult(success=False)
        
        open_set = []
        heapq.heappush(open_set, (0, 0, start, None, None))
        
        visited: Dict[GridPoint, Tuple[float, Optional[GridPoint], Optional[Tuple[int, int]]]] = {}
        
        iterations = 0
        
        while open_set and iterations < cfg.max_iterations:
            iterations += 1
            
            f_score, g_score, current, came_from, last_dir = heapq.heappop(open_set)
            
            if current in visited:
                existing_g, _, _ = visited[current]
                if existing_g <= g_score:
                    continue
            
            visited[current] = (g_score, came_from, last_dir)
            
            if (current.x == end.x and current.y == end.y and 
                current.layer == pad_layer):
                return self._reconstruct_path(
                    visited, current, 
                    source_pad, target_pad,
                    source_conn, target_conn
                )
            
            neighbors = self._get_neighbors(current, end, last_dir)
            
            for neighbor, move_cost, new_dir in neighbors:
                if neighbor in visited:
                    continue
                
                # if not om.is_valid_grid(neighbor.x, neighbor.y):
                #     continue
                
                if om.is_blocked(neighbor.x, neighbor.y, neighbor.layer):
                    continue
                
                if neighbor.layer != current.layer:
                    if not om.can_place_via(current.x, current.y):
                        continue
                
                new_g = g_score + move_cost
                h = self._heuristic(neighbor, end)
                new_f = new_g + h
                
                heapq.heappush(open_set, (new_f, new_g, neighbor, current, new_dir))
        
        return RoutingResult(success=False)
    
    def _get_neighbors(self, current: GridPoint, end: GridPoint, 
                       last_direction: Optional[Tuple[int, int]]) -> List[Tuple[GridPoint, float, Tuple[int, int]]]:
        neighbors = []
        gs = self.config.grid_size
        cfg = self.config
        om = self.obstacle_map
        
        for dx, dy in self.directions:
            nx, ny = current.x + dx, current.y + dy
            direction = (dx, dy)
            
            if dx != 0 and dy != 0:
                cost = math.sqrt(2) * gs
            else:
                cost = gs
            
            if last_direction is not None and direction != last_direction:
                cost += cfg.direction_change_cost
            
            neighbors.append((GridPoint(nx, ny, current.layer), cost, direction))
        
        if cfg.num_layers > 1:
            if om.can_place_via(current.x, current.y):
                for other_layer in range(cfg.num_layers):
                    if other_layer != current.layer:
                        cost = cfg.via_cost
                        neighbors.append((
                            GridPoint(current.x, current.y, other_layer), 
                            cost, 
                            last_direction or (0, 0)
                        ))
        
        return neighbors
    
    def _heuristic(self, point: GridPoint, end: GridPoint) -> float:
        gs = self.config.grid_size
        cfg = self.config
        
        dx = abs(point.x - end.x)
        dy = abs(point.y - end.y)
        
        diag = min(dx, dy)
        straight = abs(dx - dy)
        plane_dist = (diag * math.sqrt(2) + straight) * gs
        
        layer_cost = 0
        if point.layer != cfg.pad_layer:
            layer_cost = cfg.via_cost
        
        return plane_dist + layer_cost
    
    def _reconstruct_path(self, 
                          visited: Dict[GridPoint, Tuple[float, Optional[GridPoint], Optional[Tuple[int, int]]]],
                          end_point: GridPoint,
                          source_pad: PadInfo, target_pad: PadInfo,
                          source_conn: Tuple[float, float],
                          target_conn: Tuple[float, float]) -> RoutingResult:
        path = []
        current = end_point
        
        while current is not None:
            path.append(current)
            _, came_from, _ = visited[current]
            current = came_from
        
        path.reverse()
        
        if len(path) < 2:
            return RoutingResult(success=False)
        
        simplified = self._simplify_path(path)
        
        segments = []
        vias = []
        total_length = 0.0
        om = self.obstacle_map
        
        for i in range(len(simplified) - 1):
            p1 = simplified[i]
            p2 = simplified[i + 1]
            
            wx1, wy1 = om.grid_to_world(p1.x, p1.y)
            wx2, wy2 = om.grid_to_world(p2.x, p2.y)
            
            if i == 0:
                wx1, wy1 = source_conn
            if i == len(simplified) - 2:
                wx2, wy2 = target_conn
            
            if p1.layer == p2.layer:
                seg_length = math.sqrt((wx2 - wx1)**2 + (wy2 - wy1)**2)
                if seg_length > 0.001:
                    segments.append(Segment(
                        start=(wx1, wy1),
                        end=(wx2, wy2),
                        layer=p1.layer,
                        width=self.config.trace_width
                    ))
                    total_length += seg_length
            else:
                vias.append(Via(
                    x=wx1,
                    y=wy1,
                    layers=(min(p1.layer, p2.layer), max(p1.layer, p2.layer)),
                    diameter=self.config.via_diameter
                ))

        return RoutingResult(
            success=True,
            segments=segments,
            vias=vias,
            total_length=total_length,
            num_vias=len(vias)
        )

    def _simplify_path(self, path: List[GridPoint]) -> List[GridPoint]:
        if len(path) <= 2:
            return path

        simplified = [path[0]]

        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_p = path[i + 1]

            if prev.layer != curr.layer or curr.layer != next_p.layer:
                simplified.append(curr)
                continue

            dir1 = (curr.x - prev.x, curr.y - prev.y)
            dir2 = (next_p.x - curr.x, next_p.y - curr.y)

            if dir1 != dir2:
                simplified.append(curr)

        simplified.append(path[-1])
        return simplified
        
class UCGExporter:
    """UCG格式导出器（对齐 prompt 规则）"""

    SMALL_WIDTH = 3.0
    MEDIUM_WIDTH = 6.0
    LARGE_WIDTH = 8.0

    def __init__(self):
        self.space_node_counter = 0
        self.generation_logs: List[str] = []

    def _next_space_node(self) -> str:
        self.space_node_counter += 1
        return f"SN_{self.space_node_counter}"

    def _reset_space_node_counter(self):
        self.space_node_counter = 0

    def _extract_numeric_suffix(self, text: str, default: int = 0) -> int:
        if not text:
            return default
        buf = ""
        for ch in reversed(text):
            if ch.isdigit():
                buf = ch + buf
            elif buf:
                break
        return int(buf) if buf else default

    def _micro_sort_key(self, micro_id: str) -> Tuple[int, str]:
        return (self._extract_numeric_suffix(micro_id, 10**9), micro_id)

    def _component_sort_key(self, comp_id: str) -> Tuple[int, str]:
        return (self._extract_numeric_suffix(comp_id, 10**9), comp_id)

    def _normalize_rotation(self, value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _normalize_pad_id(self, pin_id: str) -> str:
        if not pin_id:
            return "Pad_1"
        if pin_id.startswith("Pad_"):
            return pin_id
        return f"Pad_{pin_id}"

    def _channel_width_for_net(self, net_id: str) -> float:
        name = (net_id or "").upper()
        medium_tokens = ["VCC_5V"]
        large_tokens = ["GND", "VCC_12V", "VCC_48V", "/EL+", "/EL-", "IND", "L", "MOSFET", "TO-252"]
        if any(tok in name for tok in large_tokens):
            return self.LARGE_WIDTH
        if any(tok in name for tok in medium_tokens):
            return self.MEDIUM_WIDTH
        return self.SMALL_WIDTH

    def _channel_size_label(self, width: float) -> str:
        if width >= self.LARGE_WIDTH:
            return "Large"
        if width >= self.MEDIUM_WIDTH:
            return "Medium"
        return "Small"

    def _effective_size(self, w: float, h: float, rot: float) -> Tuple[float, float]:
        if rot in [90.0, 270.0, -90.0, -270.0]:
            return h, w
        return w, h

    def _build_chain_spatial_topology(self, ordered_ids: List[str], dimensions: Dict[str, Tuple[float, float]],
                                      rotations: Dict[str, float], shared_width_lookup: Dict[Tuple[str, str], float]) -> List[Dict]:
        if len(ordered_ids) < 2:
            return []
        edges = []
        for idx in range(len(ordered_ids) - 1):
            source_id = ordered_ids[idx]
            target_id = ordered_ids[idx + 1]
            source_w, source_h = self._effective_size(*dimensions.get(source_id, (1.0, 1.0)), rotations.get(source_id, 0.0))
            target_w, target_h = self._effective_size(*dimensions.get(target_id, (1.0, 1.0)), rotations.get(target_id, 0.0))
            channel_width = shared_width_lookup.get((source_id, target_id), self.SMALL_WIDTH)
            dx = float((source_w / 2.0) + channel_width + (target_w / 2.0))
            dy = float(0.0)
            edges.append({"type": "SPATIAL", "source": source_id, "target": target_id, "dx": float(round(dx, 3)), "dy": float(round(dy, 3))})
        return edges

    def export(self, placement: Dict, routed_connections: List[RoutedConnection],
               pad_infos: Dict[str, PadInfo], components: Dict, metadata: Dict) -> Dict:
        del placement, pad_infos
        micro_info = metadata.get("micro_info", {})
        micro_partition = metadata.get("micro_partition", {})
        if not micro_info:
            raise ValueError("micro_info is missing in metadata")

        comp_to_micro: Dict[str, str] = {comp_id: comp.get("micro_id", "Micro_1") for comp_id, comp in components.items()}
        for micro_id, comp_ids in micro_partition.items():
            for comp_id in comp_ids:
                comp_to_micro[comp_id] = micro_id

        ordered_micros = sorted(set(micro_info.keys()) | set(comp_to_micro.values()), key=self._micro_sort_key)
        micro_to_nets: Dict[str, Set[str]] = defaultdict(set)
        for comp_id, comp in components.items():
            micro_id = comp_to_micro.get(comp_id, comp.get("micro_id", "Micro_1"))
            for pin in comp.get("pins", []):
                net_id = pin.get("net", "")
                if net_id:
                    micro_to_nets[micro_id].add(net_id)

        real_inter_micro_pairs = self._collect_real_inter_micro_pairs(routed_connections)
        self._reset_space_node_counter()
        level_0 = self._build_level_0(ordered_micros, micro_info, micro_to_nets, real_inter_micro_pairs)
        level_1 = self._build_level_1(ordered_micros, components, comp_to_micro)
        return {"UCG_Graph": {"level_0_global": level_0, "level_1_details": level_1}}

    def _build_level_0(self, ordered_micros: List[str], micro_info: Dict,
                       micro_to_nets: Dict[str, Set[str]], real_inter_micro_pairs: Dict[str, Set[Tuple[str, str]]]) -> Dict:
        nodes: List[Dict] = []
        dimensions: Dict[str, Tuple[float, float]] = {}
        rotations: Dict[str, float] = {}
        for micro_id in ordered_micros:
            info = micro_info.get(micro_id, {})
            bbox = info.get("bbox", [0.0, 0.0, 10.0, 10.0])
            min_x, min_y, max_x, max_y = [float(v) for v in bbox]
            w = float(max_x - min_x)
            h = float(max_y - min_y)
            nodes.append({
                "id": micro_id,
                "type": "Micro",
                "rot": 0.0,
                "w": float(round(w, 3)),
                "h": float(round(h, 3)),
                "center_x": float(round((min_x + max_x) / 2.0, 3)),
                "center_y": float(round((min_y + max_y) / 2.0, 3)),
            })
            dimensions[micro_id] = (w, h)
            rotations[micro_id] = 0.0

        shared_width_lookup: Dict[Tuple[str, str], float] = {}
        for idx in range(len(ordered_micros) - 1):
            m1 = ordered_micros[idx]
            m2 = ordered_micros[idx + 1]
            shared = micro_to_nets.get(m1, set()) & micro_to_nets.get(m2, set())
            shared_width_lookup[(m1, m2)] = max(self._channel_width_for_net(net_id) for net_id in shared) if shared else self.SMALL_WIDTH

        return {
            "nodes": nodes,
            "spatial_topology": self._build_chain_spatial_topology(ordered_micros, dimensions, rotations, shared_width_lookup),
            "routing_resources": self._build_inter_micro_routing_resources(ordered_micros, micro_to_nets, real_inter_micro_pairs),
        }

    def _collect_real_inter_micro_pairs(self, routed_connections: List[RoutedConnection]) -> Dict[str, Set[Tuple[str, str]]]:
        net_to_pairs: Dict[str, Set[Tuple[str, str]]] = defaultdict(set)
        for conn in routed_connections:
            if conn.source_micro and conn.target_micro and conn.net_id and conn.source_micro != conn.target_micro:
                net_to_pairs[conn.net_id].add(tuple(sorted((conn.source_micro, conn.target_micro))))
        return net_to_pairs

    def _build_inter_micro_routing_resources(self, ordered_micros: List[str], micro_to_nets: Dict[str, Set[str]],
                                             real_inter_micro_pairs: Dict[str, Set[Tuple[str, str]]]) -> List[Dict]:
        index_map = {m_id: idx for idx, m_id in enumerate(ordered_micros)}
        resources: List[Dict] = []
        for net_id in sorted(real_inter_micro_pairs.keys()):
            pair_list = sorted(real_inter_micro_pairs[net_id], key=lambda p: (index_map.get(p[0], 10**9), index_map.get(p[1], 10**9)))
            for source_micro, target_micro in pair_list:
                if net_id not in micro_to_nets.get(source_micro, set()) or net_id not in micro_to_nets.get(target_micro, set()):
                    continue
                if index_map.get(source_micro, 10**9) > index_map.get(target_micro, 10**9):
                    source_micro, target_micro = target_micro, source_micro
                sn_id = self._next_space_node()
                resources.append({
                    "net_id": net_id,
                    "path_sequence": [f"{source_micro}.{net_id}", sn_id, f"{target_micro}.{net_id}"],
                    "space_node": {"id": sn_id, "size": self._channel_size_label(self._channel_width_for_net(net_id))},
                })
        return resources

    def _build_level_1(self, ordered_micros: List[str], components: Dict, comp_to_micro: Dict[str, str]) -> Dict:
        level_1: Dict[str, Dict] = {}
        for micro_id in ordered_micros:
            micro_components = [(comp_id, comp) for comp_id, comp in components.items() if comp_to_micro.get(comp_id, comp.get("micro_id", "Micro_1")) == micro_id]
            micro_components.sort(key=lambda item: self._component_sort_key(item[0]))
            level_1[micro_id] = self._build_micro_detail(micro_id, micro_components)
        return level_1

    def _build_micro_detail(self, micro_id: str, micro_components: List[Tuple[str, Dict]]) -> Dict:
        micro_num = self._extract_numeric_suffix(micro_id, 1)
        nodes: List[Dict] = []
        dimensions: Dict[str, Tuple[float, float]] = {}
        rotations: Dict[str, float] = {}
        comp_nets: Dict[str, Set[str]] = {}
        pad_net_lookup: Dict[str, str] = {}
        net_to_endpoints: Dict[str, List[str]] = defaultdict(list)

        for idx, (_, comp) in enumerate(micro_components, start=1):
            alias_id = f"M{micro_num}_Comp_{idx}"
            comp_rot = self._normalize_rotation(comp.get("rotation", 0.0))
            comp_size = comp.get("size", [1.0, 1.0])
            comp_w = float(comp_size[0]) if len(comp_size) >= 1 else 1.0
            comp_h = float(comp_size[1]) if len(comp_size) >= 2 else 1.0
            pads: Dict[str, Dict] = {}
            comp_net_set: Set[str] = set()
            for pin in comp.get("pins", []):
                pad_id = self._normalize_pad_id(str(pin.get("pin_id", "Pad_1")))
                rel_pos = pin.get("rel_pos", [0.0, 0.0])
                if not isinstance(rel_pos, list) or len(rel_pos) != 2:
                    rel_pos = [0.0, 0.0]
                pin_size = pin.get("size", [0.6, 0.6])
                if not isinstance(pin_size, list) or len(pin_size) != 2:
                    pin_size = [0.6, 0.6]
                net_id = str(pin.get("net", ""))
                pad_rot = self._normalize_rotation(pin.get("rotation", comp_rot))
                pads[pad_id] = {
                    "rel_pos": [float(rel_pos[0]), float(rel_pos[1])],
                    "rot": float(pad_rot),
                    "w": float(pin_size[0]),
                    "h": float(pin_size[1]),
                    "net": net_id,
                }
                endpoint = f"{alias_id}.{pad_id}({net_id})"
                pad_net_lookup[endpoint] = net_id
                if net_id:
                    net_to_endpoints[net_id].append(endpoint)
                    comp_net_set.add(net_id)

            nodes.append({
                "id": alias_id,
                "type": str(comp.get("type", comp.get("comp_type", "Component"))),
                "rot": float(comp_rot),
                "w": float(comp_w),
                "h": float(comp_h),
                "pads": pads,
            })
            dimensions[alias_id] = (comp_w, comp_h)
            rotations[alias_id] = comp_rot
            comp_nets[alias_id] = comp_net_set

        nodes.sort(key=lambda node: self._component_sort_key(node["id"]))
        ordered_aliases = [node["id"] for node in nodes]
        shared_width_lookup: Dict[Tuple[str, str], float] = {}
        for idx in range(len(ordered_aliases) - 1):
            c1 = ordered_aliases[idx]
            c2 = ordered_aliases[idx + 1]
            shared_nets = comp_nets.get(c1, set()) & comp_nets.get(c2, set())
            shared_width_lookup[(c1, c2)] = max(self._channel_width_for_net(net_id) for net_id in shared_nets) if shared_nets else self.SMALL_WIDTH

        return {
            "nodes": nodes,
            "spatial_topology": self._build_chain_spatial_topology(ordered_aliases, dimensions, rotations, shared_width_lookup),
            "routing_resources": self._build_micro_routing_resources(micro_id, net_to_endpoints, pad_net_lookup),
        }

    def _build_micro_routing_resources(self, micro_id: str, net_to_endpoints: Dict[str, List[str]],
                                       pad_net_lookup: Dict[str, str]) -> List[Dict]:
        resources: List[Dict] = []
        for net_id in sorted(net_to_endpoints.keys()):
            endpoints = sorted(net_to_endpoints[net_id])
            if len(endpoints) < 2:
                continue
            source = endpoints[0]
            for target in endpoints[1:]:
                if not self._is_valid_same_net_entry(source, target, net_id, pad_net_lookup, micro_id):
                    continue
                sn_id = self._next_space_node()
                resources.append({
                    "net_id": net_id,
                    "path_sequence": [source, sn_id, target],
                    "space_node": {"id": sn_id, "size": self._channel_size_label(self._channel_width_for_net(net_id))},
                })
        return resources

    def _is_valid_same_net_entry(self, source_endpoint: str, target_endpoint: str,
                                 net_id: str, pad_net_lookup: Dict[str, str],
                                 micro_id: str) -> bool:
        source_net = pad_net_lookup.get(source_endpoint)
        target_net = pad_net_lookup.get(target_endpoint)
        if source_net != net_id or target_net != net_id:
            self.generation_logs.append(f"Skip routing in {micro_id}: endpoint net mismatch ({source_endpoint}, {target_endpoint}, {net_id})")
            return False
        try:
            source_comp = source_endpoint.split(".", 1)[0]
            target_comp = target_endpoint.split(".", 1)[0]
        except Exception:
            self.generation_logs.append(f"Skip routing in {micro_id}: malformed endpoint ({source_endpoint}, {target_endpoint})")
            return False
        micro_num = self._extract_numeric_suffix(micro_id, 1)
        return source_comp.startswith(f"M{micro_num}_Comp_") and target_comp.startswith(f"M{micro_num}_Comp_")

class PCBRouter:
    """PCB 布线器主类"""
    
    def __init__(self, config: RouterConfig = None):
        self.config = config or RouterConfig()
    
    def route_pcb(self, placement: Dict, netlist: Dict, metadata: Dict = None) -> Tuple[Dict, Dict]:
        """
        对整个 PCB 进行布线
        
        Returns:
            routing_result: 布线结果
            ucg: UCG格式数据
        """
        if metadata is None:
            exit("Metadata is required for PCB routing.")
        
        canvas = placement.get("canvas", {})
        canvas_width = canvas.get("width", 100)
        canvas_height = canvas.get("height", 100)
        
        obstacle_map = ObstacleMap(self.config, canvas_width, canvas_height)
        components = {c["id"]: c for c in placement.get("components", [])}
        
        # 添加焊盘并收集信息（包含net和micro_id）
        pad_infos = self._add_pad_obstacles(obstacle_map, components)
        
        # 获取边并按距离排序
        edges = netlist.get("edges", [])
        edges_with_distance = []
        for edge in edges:
            source = edge["source"]
            target = edge["target"]
            source_key = f"{source['component']}_{source['pin']}"
            target_key = f"{target['component']}_{target['pin']}"
            
            if source_key in pad_infos and target_key in pad_infos:
                sp = pad_infos[source_key]
                tp = pad_infos[target_key]
                dist = math.sqrt((sp.center_x - tp.center_x)**2 + 
                               (sp.center_y - tp.center_y)**2)
                edges_with_distance.append((edge, dist))
        
        edges_sorted = sorted(edges_with_distance, key=lambda x: x[1])
        
        router = AStarRouter(obstacle_map, self.config)
        
        all_segments = []
        all_vias = []
        routed_connections = []  # 记录实际布线顺序
        routing_order = 0
        
        routing_stats = {
            "total_edges": len(edges_sorted),
            "success_count": 0,
            "failed_count": 0,
            "total_length": 0.0,
            "total_vias": 0,
            "failed_edges": []
        }
        
        # 详细的错误报告
        error_report = {
            "pad_not_found": [],
            "routing_failed": [],
            "exceptions": []
        }
        
        # 逐条布线
        for idx, (edge, dist) in enumerate(edges_sorted):
            source = edge["source"]
            target = edge["target"]
            
            source_pad_key = f"{source['component']}_{source['pin']}"
            target_pad_key = f"{target['component']}_{target['pin']}"
            
            source_pad = pad_infos.get(source_pad_key)
            target_pad = pad_infos.get(target_pad_key)
            
            # 错误1：焊盘信息缺失
            if not source_pad or not target_pad:
                routing_stats["failed_count"] += 1
                routing_stats["failed_edges"].append(edge)
                
                error_info = {
                    "edge_index": idx,
                    "net": edge.get("net", "Unknown"),
                    "source": f"{source['component']}.{source['pin']}",
                    "target": f"{target['component']}.{target['pin']}",
                    "distance": f"{dist:.2f} mm",
                    "reason": []
                }
                
                if not source_pad:
                    error_info["reason"].append(f"Source pad '{source_pad_key}' not found")
                if not target_pad:
                    error_info["reason"].append(f"Target pad '{target_pad_key}' not found")
                
                error_report["pad_not_found"].append(error_info)
                continue
            
            try:
                result = router.route(
                    source_pad, target_pad,
                    pad_keys_to_exclude=[source_pad_key, target_pad_key]
                )
                
                # 错误2：A*算法失败
                if result.success:
                    all_segments.extend(result.segments)
                    all_vias.extend(result.vias)
                    
                    for seg in result.segments:
                        obstacle_map.add_routed_segment(seg)
                    for via in result.vias:
                        obstacle_map.add_routed_via(via)
                    
                    # 记录布线连接（用于UCG导出）
                    routed_connections.append(RoutedConnection(
                        net_id=source_pad.net or edge.get("net", f"Net_{routing_order}"),
                        source_component=source["component"],
                        source_pin=source["pin"],
                        source_micro=source_pad.micro_id,
                        target_component=target["component"],
                        target_pin=target["pin"],
                        target_micro=target_pad.micro_id,
                        segments=result.segments,
                        vias=result.vias,
                        routing_order=routing_order
                    ))
                    
                    routing_order += 1
                    routing_stats["success_count"] += 1
                    routing_stats["total_length"] += result.total_length
                    routing_stats["total_vias"] += result.num_vias
                else:
                    routing_stats["failed_count"] += 1
                    routing_stats["failed_edges"].append(edge)
                    
                    error_report["routing_failed"].append({
                        "edge_index": idx,
                        "net": edge.get("net", "Unknown"),
                        "source": f"{source['component']}.{source['pin']}",
                        "target": f"{target['component']}.{target['pin']}",
                        "source_pos": f"({source_pad.center_x:.2f}, {source_pad.center_y:.2f})",
                        "target_pos": f"({target_pad.center_x:.2f}, {target_pad.center_y:.2f})",
                        "distance": f"{dist:.2f} mm",
                        "source_micro": source_pad.micro_id,
                        "target_micro": target_pad.micro_id,
                        "reason": "A* algorithm failed to find a valid path (likely blocked by obstacles or too complex)"
                    })
            
            except Exception as e:
                # 错误3：异常错误
                routing_stats["failed_count"] += 1
                routing_stats["failed_edges"].append(edge)
                
                import traceback
                error_report["exceptions"].append({
                    "edge_index": idx,
                    "net": edge.get("net", "Unknown"),
                    "source": f"{source['component']}.{source['pin']}",
                    "target": f"{target['component']}.{target['pin']}",
                    "distance": f"{dist:.2f} mm",
                    "exception": str(e),
                    "traceback": traceback.format_exc()
                })
        
        # 打印错误报告
        self._print_error_report(error_report, routing_stats)
        
        # 导出UCG
        ucg_exporter = UCGExporter()
        ucg = ucg_exporter.export(placement, routed_connections, pad_infos, components, metadata)
        
        routing_result = {
            "circuit_id": placement.get("circuit_id", "unknown"),
            "segments": [
                {
                    "start": list(seg.start),
                    "end": list(seg.end),
                    "layer": seg.layer,
                    "width": seg.width
                }
                for seg in all_segments
            ],
            "vias": [
                {
                    "x": via.x,
                    "y": via.y,
                    "layers": list(via.layers),
                    "diameter": via.diameter
                }
                for via in all_vias
            ],
            "statistics": routing_stats,
            "error_report": error_report  # 添加到输出中
        }
        
        return routing_result, ucg
    
    def _print_error_report(self, error_report: Dict, routing_stats: Dict):
        """打印详细的错误报告"""
        total_errors = (len(error_report["pad_not_found"]) + 
                       len(error_report["routing_failed"]) + 
                       len(error_report["exceptions"]))
        
        if total_errors == 0:
            return
        
        print("\n" + "=" * 60)
        print("ROUTING ERROR REPORT")
        print("=" * 60)
        
        # 1. 焊盘未找到错误
        if error_report["pad_not_found"]:
            print(f"\n[1] PAD NOT FOUND ERRORS ({len(error_report['pad_not_found'])} errors)")
            print("-" * 60)
            for err in error_report["pad_not_found"]:
                print(f"  Edge #{err['edge_index']} | Net: {err['net']}")
                print(f"    {err['source']} -> {err['target']}")
                print(f"    Distance: {err['distance']}")
                for reason in err['reason']:
                    print(f"    ✗ {reason}")
                print()
        
        # 2. 布线失败错误
        if error_report["routing_failed"]:
            print(f"\n[2] ROUTING FAILED ERRORS ({len(error_report['routing_failed'])} errors)")
            print("-" * 60)
            for err in error_report["routing_failed"]:
                print(f"  Edge #{err['edge_index']} | Net: {err['net']}")
                print(f"    {err['source']} -> {err['target']}")
                print(f"    Source: {err['source_pos']} [{err['source_micro']}]")
                print(f"    Target: {err['target_pos']} [{err['target_micro']}]")
                print(f"    Distance: {err['distance']}")
                print(f"    ✗ {err['reason']}")
                print()
        
        # 3. 异常错误
        if error_report["exceptions"]:
            print(f"\n[3] EXCEPTION ERRORS ({len(error_report['exceptions'])} errors)")
            print("-" * 60)
            for err in error_report["exceptions"]:
                print(f"  Edge #{err['edge_index']} | Net: {err['net']}")
                print(f"    {err['source']} -> {err['target']}")
                print(f"    Distance: {err['distance']}")
                print(f"    ✗ Exception: {err['exception']}")
                print(f"    Traceback:")
                for line in err['traceback'].split('\n'):
                    if line.strip():
                        print(f"      {line}")
                print()
        
        print("=" * 60)
        print(f"Total Errors: {total_errors} / {routing_stats['total_edges']} edges")
        print(f"Success Rate: {(routing_stats['success_count']/routing_stats['total_edges']*100):.1f}%")
        print("=" * 60)
    
    def _add_pad_obstacles(self, obstacle_map: ObstacleMap, 
                           components: Dict) -> Dict[str, PadInfo]:
        """将所有焊盘添加为障碍物，并记录net和micro_id"""
        pad_infos = {}
        pad_layer = self.config.pad_layer
        
        for comp_id, comp in components.items():
            comp_x, comp_y = comp["position"]
            rotation = comp.get("rotation", 0)
            micro_id = comp.get("micro_id", "Micro_1")
            
            for pin in comp.get("pins", []):
                rel_x, rel_y = pin["rel_pos"]
                
                abs_x, abs_y = self._rotate_point(rel_x, rel_y, rotation)
                abs_x += comp_x
                abs_y += comp_y
                
                pad_w, pad_h = pin["size"]
                if rotation in [90, 270, -90, -270]:
                    pad_w, pad_h = pad_h, pad_w
                
                pad_info = PadInfo(
                    center_x=abs_x,
                    center_y=abs_y,
                    width=pad_w,
                    height=pad_h,
                    layer=pad_layer,
                    component_id=comp_id,
                    pin_id=pin['pin_id'],
                    net=pin.get("net", ""),
                    micro_id=micro_id,
                    rotation=pin.get("rotation", 0)
                )
                
                pad_key = pad_info.pad_key
                pad_infos[pad_key] = pad_info
                
                obstacle_map.add_pad(pad_info, self.config.clearance_trace_pad)
        
        return pad_infos
    
    def _rotate_point(self, x: float, y: float, rotation: float) -> Tuple[float, float]:
        if rotation == 0:
            return x, y
        
        rad = math.radians(rotation)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        
        new_x = x * cos_r - y * sin_r
        new_y = x * sin_r + y * cos_r
        
        return new_x, new_y


def route_pcb(placement: Dict, netlist: Dict, config: RouterConfig = None, 
              metadata: Dict = None) -> Tuple[Dict, Dict]:
    """
    便捷的布线函数
    
    Returns:
        routing_result: 布线结果
        ucg: UCG格式数据
    """
    router = PCBRouter(config)
    return router.route_pcb(placement, netlist, metadata)