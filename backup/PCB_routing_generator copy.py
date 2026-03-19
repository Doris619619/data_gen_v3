"""
PCB Router - A* 基于网格的布线器
支持 8 方向走线（0°, 45°, 90° 等）、双层布线、过孔
"""

import math
import heapq
from typing import List, Dict, Tuple, Set, Optional
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
    grid_size: float = 0.5
    num_layers: int = 2
    trace_width: float = 0.2
    via_diameter: float = 0.4
    via_drill: float = 0.2
    
    # 间距要求
    clearance_trace_trace: float = 0.1
    clearance_trace_pad: float = 0.1
    clearance_trace_via: float = 0.1
    clearance_via_via: float = 0.1
    clearance_trace_boundary: float = 0.1
    
    # 代价权重
    via_cost: float = 10.0  # 过孔的额外代价
    layer_change_cost: float = 5.0  # 换层代价
    
    # 搜索限制
    max_iterations: int = 100000


class ObstacleMap:
    """障碍物地图管理"""
    
    def __init__(self, config: RouterConfig, canvas_width: float, canvas_height: float):
        self.config = config
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.grid_size = config.grid_size
        
        # 网格尺寸
        self.grid_width = int(math.ceil(canvas_width / self.grid_size)) + 1
        self.grid_height = int(math.ceil(canvas_height / self.grid_size)) + 1
        
        # 每层的障碍物网格 (layer -> set of (gx, gy))
        self.blocked: Dict[int, Set[Tuple[int, int]]] = defaultdict(set)
        
        # 焊盘位置记录 (用于排除当前连接的焊盘)
        self.pad_positions: Dict[str, List[Tuple[int, int, int]]] = {}  # pad_key -> [(gx, gy, layer), ...]
        
        # 已布线的线段 (用于动态添加障碍)
        self.routed_segments: List[Segment] = []
        self.routed_vias: List[Via] = []
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转网格坐标"""
        gx = int(round(x / self.grid_size))
        gy = int(round(y / self.grid_size))
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """网格坐标转世界坐标"""
        return gx * self.grid_size, gy * self.grid_size
    
    def is_valid_grid(self, gx: int, gy: int) -> bool:
        """检查网格坐标是否在有效范围内"""
        margin = int(math.ceil(self.config.clearance_trace_boundary / self.grid_size))
        return (margin <= gx < self.grid_width - margin and 
                margin <= gy < self.grid_height - margin)
    
    def add_rectangle_obstacle(self, x1: float, y1: float, x2: float, y2: float, 
                                layer: int, clearance: float = 0):
        """添加矩形障碍物"""
        # 扩展 clearance
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
        """添加圆形障碍物"""
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
        """添加线段障碍物（走线）"""
        total_width = width / 2 + clearance
        
        # 使用 Bresenham 类似的方法栅格化线段
        gx1, gy1 = self.world_to_grid(x1, y1)
        gx2, gy2 = self.world_to_grid(x2, y2)
        
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        
        if dx == 0 and dy == 0:
            self._add_point_with_width(gx1, gy1, total_width, layer)
            return
        
        # 遍历线段上的所有点
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
        """在点周围添加一定宽度的障碍"""
        grid_width = int(math.ceil(width / self.grid_size))
        for dx in range(-grid_width, grid_width + 1):
            for dy in range(-grid_width, grid_width + 1):
                if dx * dx + dy * dy <= grid_width * grid_width:
                    self.blocked[layer].add((gx + dx, gy + dy))
    
    def add_pad(self, pad_key: str, cx: float, cy: float, 
                width: float, height: float, layer: int, clearance: float = 0):
        """添加焊盘障碍物"""
        half_w = width / 2 + clearance
        half_h = height / 2 + clearance
        
        gx1, gy1 = self.world_to_grid(cx - half_w, cy - half_h)
        gx2, gy2 = self.world_to_grid(cx + half_w, cy + half_h)
        
        pad_grids = []
        for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
            for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
                self.blocked[layer].add((gx, gy))
                pad_grids.append((gx, gy, layer))
        
        self.pad_positions[pad_key] = pad_grids
    
    def add_via_obstacle(self, via: Via, clearance: float = 0):
        """添加过孔障碍物"""
        radius = via.diameter / 2
        for layer in range(self.config.num_layers):
            self.add_circle_obstacle(via.x, via.y, radius, layer, clearance)
    
    def temporarily_remove_pads(self, pad_keys: List[str]) -> Dict[str, List[Tuple[int, int, int]]]:
        """临时移除指定焊盘的障碍（用于当前布线）"""
        removed = {}
        for key in pad_keys:
            if key in self.pad_positions:
                removed[key] = self.pad_positions[key]
                for gx, gy, layer in self.pad_positions[key]:
                    self.blocked[layer].discard((gx, gy))
        return removed
    
    def restore_pads(self, removed: Dict[str, List[Tuple[int, int, int]]]):
        """恢复之前移除的焊盘障碍"""
        for key, grids in removed.items():
            for gx, gy, layer in grids:
                self.blocked[layer].add((gx, gy))
    
    def is_blocked(self, gx: int, gy: int, layer: int) -> bool:
        """检查网格点是否被阻挡"""
        return (gx, gy) in self.blocked[layer]
    
    def add_routed_segment(self, segment: Segment):
        """添加已布线段作为障碍"""
        self.routed_segments.append(segment)
        self.add_line_obstacle(
            segment.start[0], segment.start[1],
            segment.end[0], segment.end[1],
            segment.width, segment.layer,
            self.config.clearance_trace_trace
        )
    
    def add_routed_via(self, via: Via):
        """添加已布过孔作为障碍"""
        self.routed_vias.append(via)
        self.add_via_obstacle(via, self.config.clearance_via_via)


class AStarRouter:
    """A* 布线算法"""
    
    def __init__(self, obstacle_map: ObstacleMap, config: RouterConfig):
        self.obstacle_map = obstacle_map
        self.config = config
        
        # 8 方向偏移量
        self.directions = [
            (1, 0),    # E
            (1, 1),    # NE
            (0, 1),    # N
            (-1, 1),   # NW
            (-1, 0),   # W
            (-1, -1),  # SW
            (0, -1),   # S
            (1, -1),   # SE
        ]
    
    def route(self, start_x: float, start_y: float, start_layer: int,
              end_x: float, end_y: float, end_layer: int,
              pad_keys_to_exclude: List[str] = None) -> RoutingResult:
        """
        执行 A* 布线
        
        Args:
            start_x, start_y: 起点世界坐标
            start_layer: 起点层
            end_x, end_y: 终点世界坐标
            end_layer: 终点层
            pad_keys_to_exclude: 要从障碍中排除的焊盘键
        
        Returns:
            RoutingResult
        """
        # 临时移除当前连接的焊盘
        removed_pads = {}
        if pad_keys_to_exclude:
            removed_pads = self.obstacle_map.temporarily_remove_pads(pad_keys_to_exclude)
        
        try:
            result = self._astar_search(start_x, start_y, start_layer,
                                        end_x, end_y, end_layer)
            return result
        finally:
            # 恢复焊盘障碍
            if removed_pads:
                self.obstacle_map.restore_pads(removed_pads)
    
    def _astar_search(self, start_x: float, start_y: float, start_layer: int,
                      end_x: float, end_y: float, end_layer: int) -> RoutingResult:
        """A* 搜索核心"""
        om = self.obstacle_map
        cfg = self.config
        
        # 转换为网格坐标
        start_gx, start_gy = om.world_to_grid(start_x, start_y)
        end_gx, end_gy = om.world_to_grid(end_x, end_y)
        
        start = GridPoint(start_gx, start_gy, start_layer)
        end = GridPoint(end_gx, end_gy, end_layer)
        
        # 检查起点终点有效性
        if not om.is_valid_grid(start_gx, start_gy):
            return RoutingResult(success=False)
        if not om.is_valid_grid(end_gx, end_gy):
            return RoutingResult(success=False)
        
        # 优先队列: (f_score, g_score, point, came_from_point)
        # 使用 g_score 作为第二排序键来打破平局
        open_set = []
        heapq.heappush(open_set, (0, 0, start, None))
        
        # 已访问点 -> (g_score, came_from)
        visited: Dict[GridPoint, Tuple[float, Optional[GridPoint]]] = {}
        
        iterations = 0
        
        while open_set and iterations < cfg.max_iterations:
            iterations += 1
            
            f_score, g_score, current, came_from = heapq.heappop(open_set)
            
            # 如果已经访问过且代价更低，跳过
            if current in visited:
                existing_g, _ = visited[current]
                if existing_g <= g_score:
                    continue
            
            visited[current] = (g_score, came_from)
            
            # 到达终点
            if current.x == end.x and current.y == end.y and current.layer == end.layer:
                return self._reconstruct_path(visited, current, start_x, start_y, end_x, end_y)
            
            # 扩展邻居
            neighbors = self._get_neighbors(current, end)
            
            for neighbor, move_cost in neighbors:
                if neighbor in visited:
                    continue
                
                if not om.is_valid_grid(neighbor.x, neighbor.y):
                    continue
                
                if om.is_blocked(neighbor.x, neighbor.y, neighbor.layer):
                    continue
                
                new_g = g_score + move_cost
                h = self._heuristic(neighbor, end)
                new_f = new_g + h
                
                heapq.heappush(open_set, (new_f, new_g, neighbor, current))
        
        # 搜索失败
        return RoutingResult(success=False)
    
    def _get_neighbors(self, current: GridPoint, end: GridPoint) -> List[Tuple[GridPoint, float]]:
        """获取当前点的所有邻居及移动代价"""
        neighbors = []
        gs = self.config.grid_size
        
        # 8 方向平面移动
        for dx, dy in self.directions:
            nx, ny = current.x + dx, current.y + dy
            
            # 对角移动代价为 √2，直线移动代价为 1
            if dx != 0 and dy != 0:
                cost = math.sqrt(2) * gs
            else:
                cost = gs
            
            neighbors.append((GridPoint(nx, ny, current.layer), cost))
        
        # 换层（放置过孔）
        if self.config.num_layers > 1:
            for other_layer in range(self.config.num_layers):
                if other_layer != current.layer:
                    # 换层代价
                    cost = self.config.via_cost
                    neighbors.append((GridPoint(current.x, current.y, other_layer), cost))
        
        return neighbors
    
    def _heuristic(self, point: GridPoint, end: GridPoint) -> float:
        """启发式函数：曼哈顿距离 + 层差代价"""
        gs = self.config.grid_size
        
        # 使用对角距离（Chebyshev-like）更准确
        dx = abs(point.x - end.x)
        dy = abs(point.y - end.y)
        
        # 对角距离
        diag = min(dx, dy)
        straight = abs(dx - dy)
        plane_dist = (diag * math.sqrt(2) + straight) * gs
        
        # 层差代价
        layer_cost = 0
        if point.layer != end.layer:
            layer_cost = self.config.via_cost
        
        return plane_dist + layer_cost
    
    def _reconstruct_path(self, visited: Dict[GridPoint, Tuple[float, Optional[GridPoint]]],
                          end_point: GridPoint,
                          start_wx: float, start_wy: float,
                          end_wx: float, end_wy: float) -> RoutingResult:
        """从 A* 结果重建路径并生成线段"""
        # 回溯路径
        path = []
        current = end_point
        
        while current is not None:
            path.append(current)
            _, came_from = visited[current]
            current = came_from
        
        path.reverse()
        
        if len(path) < 2:
            return RoutingResult(success=False)
        
        # 简化路径（合并同方向的连续点）
        simplified = self._simplify_path(path)
        
        # 生成线段和过孔
        segments = []
        vias = []
        total_length = 0.0
        om = self.obstacle_map
        
        for i in range(len(simplified) - 1):
            p1 = simplified[i]
            p2 = simplified[i + 1]
            
            wx1, wy1 = om.grid_to_world(p1.x, p1.y)
            wx2, wy2 = om.grid_to_world(p2.x, p2.y)
            
            # 使用精确的起点/终点坐标
            if i == 0:
                wx1, wy1 = start_wx, start_wy
            if i == len(simplified) - 2:
                wx2, wy2 = end_wx, end_wy
            
            if p1.layer == p2.layer:
                # 同层走线
                seg_length = math.sqrt((wx2 - wx1)**2 + (wy2 - wy1)**2)
                segments.append(Segment(
                    start=(wx1, wy1),
                    end=(wx2, wy2),
                    layer=p1.layer,
                    width=self.config.trace_width
                ))
                total_length += seg_length
            else:
                # 换层 - 添加过孔
                vias.append(Via(
                    x=wx1,
                    y=wy1,
                    layers=(p1.layer, p2.layer),
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
        """简化路径：合并同方向的连续点"""
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_p = path[i + 1]
            
            # 检查是否换层
            if prev.layer != curr.layer or curr.layer != next_p.layer:
                simplified.append(curr)
                continue
            
            # 检查方向是否改变
            dir1 = (curr.x - prev.x, curr.y - prev.y)
            dir2 = (next_p.x - curr.x, next_p.y - curr.y)
            
            if dir1 != dir2:
                simplified.append(curr)
        
        simplified.append(path[-1])
        return simplified


class PCBRouter:
    """PCB 布线器主类"""
    
    def __init__(self, config: RouterConfig = None):
        self.config = config or RouterConfig()
    
    def route_pcb(self, placement: Dict, netlist: Dict) -> Dict:
        """
        对整个 PCB 进行布线
        
        Args:
            placement: 放置数据（包含 components）
            netlist: 网表数据（包含 edges）
        
        Returns:
            布线结果字典
        """
        canvas = placement.get("canvas", {})
        canvas_width = canvas.get("width", 100)
        canvas_height = canvas.get("height", 100)
        
        # 创建障碍物地图
        obstacle_map = ObstacleMap(self.config, canvas_width, canvas_height)
        
        # 构建组件映射
        components = {c["id"]: c for c in placement.get("components", [])}
        
        # 添加焊盘作为障碍物
        self._add_pad_obstacles(obstacle_map, components)
        
        # 获取边并按距离排序
        edges = netlist.get("edges", [])
        edges_sorted = sorted(edges, key=lambda e: e.get("distance", float('inf')))
        
        # 创建路由器
        router = AStarRouter(obstacle_map, self.config)
        
        # 布线结果
        all_segments = []
        all_vias = []
        routing_stats = {
            "total_edges": len(edges_sorted),
            "success_count": 0,
            "failed_count": 0,
            "total_length": 0.0,
            "total_vias": 0,
            "failed_edges": []
        }
        
        # 逐条布线
        for edge in edges_sorted:
            source = edge["source"]
            target = edge["target"]
            
            # 获取源和目标焊盘位置
            source_comp = components.get(source["component"])
            target_comp = components.get(target["component"])
            
            if not source_comp or not target_comp:
                routing_stats["failed_count"] += 1
                routing_stats["failed_edges"].append(edge)
                continue
            
            source_pos = self._get_pad_position(source_comp, source["pin"])
            target_pos = self._get_pad_position(target_comp, target["pin"])
            
            if not source_pos or not target_pos:
                routing_stats["failed_count"] += 1
                routing_stats["failed_edges"].append(edge)
                continue
            
            # 生成焊盘键
            source_pad_key = f"{source['component']}_{source['pin']}"
            target_pad_key = f"{target['component']}_{target['pin']}"
            
            # 默认两个焊盘都在 top 层
            source_layer = 0
            target_layer = 0
            
            # 执行布线
            result = router.route(
                source_pos[0], source_pos[1], source_layer,
                target_pos[0], target_pos[1], target_layer,
                pad_keys_to_exclude=[source_pad_key, target_pad_key]
            )
            
            if result.success:
                # 添加到结果
                all_segments.extend(result.segments)
                all_vias.extend(result.vias)
                
                # 将布线添加为障碍
                for seg in result.segments:
                    obstacle_map.add_routed_segment(seg)
                for via in result.vias:
                    obstacle_map.add_routed_via(via)
                
                routing_stats["success_count"] += 1
                routing_stats["total_length"] += result.total_length
                routing_stats["total_vias"] += result.num_vias
            else:
                routing_stats["failed_count"] += 1
                routing_stats["failed_edges"].append(edge)
        
        # 构建输出
        return {
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
            "statistics": routing_stats
        }
    
    def _add_pad_obstacles(self, obstacle_map: ObstacleMap, components: Dict):
        """将所有焊盘添加为障碍物"""
        for comp_id, comp in components.items():
            comp_x, comp_y = comp["position"]
            rotation = comp.get("rotation", 0)
            
            for pin in comp.get("pins", []):
                # 计算焊盘绝对位置
                rel_x, rel_y = pin["rel_pos"]
                
                # 应用旋转
                abs_x, abs_y = self._rotate_point(rel_x, rel_y, rotation)
                abs_x += comp_x
                abs_y += comp_y
                
                # 焊盘尺寸
                pad_w, pad_h = pin["size"]
                if rotation in [90, 270]:
                    pad_w, pad_h = pad_h, pad_w
                
                pad_key = f"{comp_id}_{pin['pin_id']}"
                
                # 添加焊盘障碍（默认在 top 层）
                obstacle_map.add_pad(
                    pad_key, abs_x, abs_y, pad_w, pad_h, 
                    layer=0, 
                    clearance=self.config.clearance_trace_pad
                )
    
    def _get_pad_position(self, component: Dict, pin_id: str) -> Optional[Tuple[float, float]]:
        """获取焊盘的绝对位置"""
        comp_x, comp_y = component["position"]
        rotation = component.get("rotation", 0)
        
        for pin in component.get("pins", []):
            if pin["pin_id"] == pin_id:
                rel_x, rel_y = pin["rel_pos"]
                abs_x, abs_y = self._rotate_point(rel_x, rel_y, rotation)
                return (comp_x + abs_x, comp_y + abs_y)
        
        return None
    
    def _rotate_point(self, x: float, y: float, rotation: float) -> Tuple[float, float]:
        """旋转点坐标"""
        if rotation == 0:
            return x, y
        
        rad = math.radians(rotation)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        
        new_x = x * cos_r - y * sin_r
        new_y = x * sin_r + y * cos_r
        
        return new_x, new_y


# 便捷函数
def route_pcb(placement: Dict, netlist: Dict, config: RouterConfig = None) -> Dict:
    """便捷的布线函数"""
    router = PCBRouter(config)
    return router.route_pcb(placement, netlist)