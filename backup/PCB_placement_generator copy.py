"""
PCB Placement 核心生成器
"""

import math
import random
import json
import uuid
import numpy as np
from typing import List, Dict, Tuple
from collections import defaultdict

from data_structures import (
    NetType, Pin, Component, MicroRegion, NetConnection, Edge
)
from footprint_library import FOOTPRINT_LIBRARY
from config import GeneratorConfig, CONFIG_DEFAULT
from micro_layout import MicroLayoutPlanner


class PCBPlacementGenerator:
    """
    PCB Placement 合成数据生成器 (v3)
    """
    
    def __init__(self, config: GeneratorConfig = None, seed: int = None):
        self.config = config or CONFIG_DEFAULT
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)
        
        self._net_counter = defaultdict(int)
        self._component_counter = 0
    
    def generate_dataset(self, num_circuits: int, output_path: str = None) -> List[Dict]:
        """生成完整数据集"""
        dataset = []
        
        for i in range(num_circuits):
            circuit_data = self.generate_single_circuit(circuit_id=f"synth_{i:06d}")
            dataset.append(circuit_data)
            
            if (i + 1) % 100 == 0:
                print(f"Generated {i + 1}/{num_circuits} circuits")
        
        if output_path:
            with open(output_path, 'w') as f:
                json.dump(dataset, f, indent=2)
            print(f"Dataset saved to {output_path}")
        
        return dataset
    
    def generate_single_circuit(self, circuit_id: str = None) -> Dict:
        """
        生成单个电路的 placement 和 netlist
        
        流程:
        1. 规划 Micro 区域（不重叠）
        2. 在各 Micro 区域内放置元件
        3. 生成 Micro 内部连接
        4. 生成跨 Micro 连接
        """
        if circuit_id is None:
            circuit_id = f"synth_{uuid.uuid4().hex[:8]}"
        
        # 重置计数器
        self._net_counter.clear()
        self._component_counter = 0
        
        # Step 1: 规划 Micro 区域
        num_micros = np.random.randint(*self.config.num_micros_range)
        layout_planner = MicroLayoutPlanner(
            self.config.canvas_width,
            self.config.canvas_height,
            self.config.micro_margin
        )
        micro_regions = layout_planner.plan_micro_regions(num_micros, layout_type="auto")
        
        # Step 2: 在各 Micro 内放置元件
        all_components = []
        for micro in micro_regions:
            components = self._place_components_in_micro(micro)
            micro.components = components
            all_components.extend(components)
        
        # Step 3: 为引脚分配网络
        self._assign_nets_to_pins(all_components, micro_regions)
        
        # Step 4: 生成边（Micro 内部 + 跨 Micro）
        edges, nets = self._generate_all_edges(micro_regions)
        
        # 构建输出
        placement = self._build_placement_dict(circuit_id, all_components)
        netlist = self._build_netlist_dict(circuit_id, nets, edges)
        
        # 构建 Micro 分区信息
        micro_partition = {
            micro.id: [c.id for c in micro.components]
            for micro in micro_regions
        }
        
        metadata = {
            "num_micros": len(micro_regions),
            "num_components": len(all_components),
            "num_edges": len(edges),
            "num_nets": len(nets),
            "micro_partition": micro_partition,
            "micro_info": {
                micro.id: {
                    "bbox": micro.bbox,
                    "num_components": len(micro.components),
                    "area": micro.area
                }
                for micro in micro_regions
            }
        }
        
        return {
            "placement": placement,
            "netlist": netlist,
            "metadata": metadata
        }
    
    def _place_components_in_micro(self, micro: MicroRegion) -> List[Component]:
        """在单个 Micro 区域内放置元件"""
        components = []
        max_retries = 500
        
        # 计算可用区域
        padding = self.config.micro_padding
        usable_bbox = (
            micro.bbox[0] + padding,
            micro.bbox[1] + padding,
            micro.bbox[2] - padding,
            micro.bbox[3] - padding
        )
        
        usable_width = usable_bbox[2] - usable_bbox[0]
        usable_height = usable_bbox[3] - usable_bbox[1]
        
        if usable_width <= 0 or usable_height <= 0:
            return components
        
        usable_area = usable_width * usable_height
        
        # 根据面积计算元件数量
        num_components = self._calculate_num_components_by_area(usable_area)
        
        target_density = np.random.uniform(*self.config.density_range)
        
        footprint_types = list(self.config.footprint_weights.keys())
        footprint_probs = np.array([
            self.config.footprint_weights.get(ft, 0.1) 
            for ft in footprint_types
        ])
        footprint_probs = footprint_probs / footprint_probs.sum()
        
        for _ in range(num_components):
            fp_type = np.random.choice(footprint_types, p=footprint_probs)
            component = self._create_component(fp_type, micro.id)
            
            placed = False
            for _ in range(max_retries):
                x = np.random.uniform(
                    usable_bbox[0] + component.width / 2,
                    usable_bbox[2] - component.width / 2
                )
                y = np.random.uniform(
                    usable_bbox[1] + component.height / 2,
                    usable_bbox[3] - component.height / 2
                )
                
                x = max(usable_bbox[0] + component.width / 2, 
                    min(x, usable_bbox[2] - component.width / 2))
                y = max(usable_bbox[1] + component.height / 2, 
                    min(y, usable_bbox[3] - component.height / 2))
                
                component.position = [x, y]
                
                if not self._check_component_overlap(component, components):
                    placed = True
                    break
            
            if placed:
                components.append(component)
            
            current_density = self._calculate_micro_density(components, micro)
            if current_density >= target_density:
                break
        
        return components


    def _calculate_num_components_by_area(self, usable_area: float) -> int:
        """
        根据可用面积计算元件数量
        
        策略：
        1. 计算参考面积（基于 canvas 尺寸和平均 micro 数量）
        2. 根据实际面积与参考面积的比例缩放元件数量
        3. 添加随机扰动
        """
        # 参考面积：假设 canvas 被平均分成 num_micros 个区域
        avg_num_micros = (self.config.num_micros_range[0] + self.config.num_micros_range[1]) / 2
        canvas_area = self.config.canvas_width * self.config.canvas_height
        reference_area = canvas_area / avg_num_micros
        
        # 面积比例
        area_ratio = usable_area / reference_area
        
        # 基准元件数量范围
        base_min, base_max = self.config.components_per_micro_range
        base_avg = (base_min + base_max) / 2
        
        # 根据面积比例缩放
        scaled_avg = base_avg * area_ratio
        
        # 计算缩放后的范围（保持相对波动比例）
        range_half_width = (base_max - base_min) / 2
        scaled_half_width = range_half_width * math.sqrt(area_ratio)  # 使用 sqrt 避免波动过大
        
        scaled_min = max(1, int(scaled_avg - scaled_half_width))
        scaled_max = max(scaled_min + 1, int(scaled_avg + scaled_half_width))
        
        # 在缩放后的范围内随机采样
        num_components = np.random.randint(scaled_min, scaled_max + 1)
        
        # 确保至少有 1 个元件
        return max(1, num_components)

    
    def _create_component(self, footprint_type: str, micro_id: str) -> Component:
        """创建单个元件"""
        fp_info = FOOTPRINT_LIBRARY.get(footprint_type, FOOTPRINT_LIBRARY["0603_R"])
        
        size_factor = self._sample_clipped_exponential()
        rotation = np.random.choice([0.0, 90.0, 180.0, 270.0])
        
        pins = []
        for j, pin_info in enumerate(fp_info["pins"]):
            pin = Pin(
                pin_id=f"Pad_{j+1}",
                rel_pos=pin_info["rel_pos"].copy(),
                size=pin_info["size"].copy(),
                net="",
                rotation=rotation
            )
            pins.append(pin)
        
        self._component_counter += 1
        
        return Component(
            id=f"Comp_{self._component_counter}",
            comp_type=footprint_type,
            position=[0.0, 0.0],
            size=[fp_info["size"][0] * size_factor, fp_info["size"][1] * size_factor],
            rotation=rotation,
            pins=pins,
            micro_id=micro_id
        )
    
    def _sample_clipped_exponential(self) -> float:
        """采样裁剪指数分布的尺寸因子"""
        while True:
            val = np.random.exponential(self.config.size_scale)
            if self.config.size_min <= val <= self.config.size_max:
                return 0.8 + val
    
    def _check_component_overlap(self, new_comp: Component, 
                                  existing: List[Component]) -> bool:
        """检查新元件是否与现有元件重叠"""
        new_bbox = new_comp.get_bbox()
        margin = 0.5
        
        for comp in existing:
            comp_bbox = comp.get_bbox()
            
            if not (new_bbox[2] + margin < comp_bbox[0] or
                   comp_bbox[2] + margin < new_bbox[0] or
                   new_bbox[3] + margin < comp_bbox[1] or
                   comp_bbox[3] + margin < new_bbox[1]):
                return True
        
        return False
    
    def _calculate_micro_density(self, components: List[Component], 
                                  micro: MicroRegion) -> float:
        """计算 Micro 区域的元件密度"""
        total_area = sum(c.width * c.height for c in components)
        return total_area / micro.area
    
    def _assign_nets_to_pins(self, components: List[Component], 
                             micro_regions: List[MicroRegion]):
        """为引脚分配网络"""
        power_nets = ["VCC_5V", "VCC_3V3", "VCC_12V"]
        ground_nets = ["GND"]
        
        micro_local_nets = {}
        for micro in micro_regions:
            num_local_nets = max(3, len(micro.components) // 3)
            micro_local_nets[micro.id] = [
                f"{micro.id}_SIG_{i}" for i in range(num_local_nets)
            ]
        
        num_global_nets = max(5, len(components) // 10)
        global_signal_nets = [f"GLOBAL_SIG_{i}" for i in range(num_global_nets)]
        
        for comp in components:
            micro_id = comp.micro_id
            local_nets = micro_local_nets.get(micro_id, [])
            
            for i, pin in enumerate(comp.pins):
                rand = np.random.random()
                
                if self._is_power_pin(comp, i):
                    if rand < self.config.ground_net_ratio:
                        pin.net = np.random.choice(ground_nets)
                    else:
                        pin.net = np.random.choice(power_nets)
                else:
                    if rand < self.config.ground_net_ratio:
                        pin.net = "GND"
                    elif rand < self.config.ground_net_ratio + self.config.power_net_ratio:
                        pin.net = np.random.choice(power_nets)
                    elif rand < 0.7 and local_nets:
                        pin.net = np.random.choice(local_nets)
                    elif rand < 0.9:
                        pin.net = np.random.choice(global_signal_nets)
                    else:
                        pin.net = self._generate_signal_net_name()
    
    def _is_power_pin(self, comp: Component, pin_index: int) -> bool:
        """判断是否为电源类引脚"""
        power_pin_map = {
            "SOT23-3": [1, 2],
            "SOT89-3": [0, 1, 2],
            "SOIC-8": [3, 5],
        }
        
        if comp.comp_type in power_pin_map:
            return pin_index in power_pin_map[comp.comp_type]
        
        if len(comp.pins) == 2:
            return np.random.random() < 0.5
        
        return False
    
    def _generate_signal_net_name(self) -> str:
        """生成信号网络名称"""
        prefixes = ["Net", "SIG", "DATA", "CTRL", "CLK"]
        prefix = np.random.choice(prefixes)
        self._net_counter[prefix] += 1
        return f"{prefix}_{self._net_counter[prefix]}"
    
    def _generate_all_edges(self, micro_regions: List[MicroRegion]) -> Tuple[List[Edge], List[NetConnection]]:
        """生成所有边（Micro 内部 + 跨 Micro）"""
        all_edges = []
        
        for micro in micro_regions:
            internal_edges = self._generate_internal_edges(micro.components)
            all_edges.extend(internal_edges)
        
        cross_edges = self._generate_cross_micro_edges(micro_regions)
        all_edges.extend(cross_edges)
        
        all_components = []
        for micro in micro_regions:
            all_components.extend(micro.components)
        nets = self._build_net_connections(all_components)
        
        return all_edges, nets





    def _generate_internal_edges(self, components: List[Component]) -> List[Edge]:
        """生成 Micro 内部的边，确保所有组件连通"""
        edges = []
        
        if len(components) < 2:
            return edges
        
        # 跟踪每个引脚的使用次数（用于优先选择未使用的引脚）
        pin_usage_count = defaultdict(lambda: defaultdict(int))  # comp_id -> pin_id -> count
        
        comp_map = {c.id: c for c in components}
        
        gamma = self.config.gamma
        scale_param = self.config.scale_parameter
        
        # Step 1: 基于概率采样边，优先使用未使用的引脚
        # 构建所有可能的组件对
        component_pairs = []
        for i, comp1 in enumerate(components):
            for j, comp2 in enumerate(components):
                if i >= j:
                    continue
                component_pairs.append((comp1, comp2))
        
        # 打乱顺序以增加随机性
        np.random.shuffle(component_pairs)
        
        for comp1, comp2 in component_pairs:
            # 找出最佳引脚对（优先选择未使用的引脚）
            best_pair = self._find_best_pin_pair(comp1, comp2, pin_usage_count)
            
            if best_pair is None:
                continue
            
            pin1, pin2, distance = best_pair
            same_net = pin1.net == pin2.net and pin1.net != ""
            
            norm_distance = distance / 50.0
            
            prob = min(gamma * math.exp(-norm_distance / scale_param), 
                    self.config.edge_max_prob)
            
            if same_net:
                prob = min(prob * 3, 0.9)
            
            if np.random.random() < prob:
                if same_net:
                    net_name = pin1.net
                else:
                    net_name = np.random.choice([pin1.net, pin2.net]) \
                            if pin1.net and pin2.net else \
                            (pin1.net or pin2.net or self._generate_signal_net_name())
                    pin1.net = net_name
                    pin2.net = net_name
                
                edges.append(Edge(
                    source_comp=comp1.id,
                    source_pin=pin1.pin_id,
                    target_comp=comp2.id,
                    target_pin=pin2.pin_id,
                    net=net_name,
                    distance=distance
                ))
                
                # 更新引脚使用计数
                pin_usage_count[comp1.id][pin1.pin_id] += 1
                pin_usage_count[comp2.id][pin2.pin_id] += 1
        
        # Step 2: 确保 Micro 内部所有组件连通（消除孤岛）
        edges = self._ensure_internal_connectivity(components, edges, pin_usage_count)
        
        # Step 3: 确保每个组件至少有 min_edges_per_component 个引脚被使用
        edges = self._ensure_min_edges_per_component(components, edges, pin_usage_count)
        
        return edges


    def _find_best_pin_pair(self, comp1: Component, comp2: Component,
                            pin_usage_count: defaultdict) -> tuple:
        """
        找出两个组件之间的最佳引脚对
        优先级：
        1. 两边都是未使用的引脚
        2. 一边未使用
        3. 使用次数最少的引脚
        在同优先级内，选择距离最近的
        """
        candidates = []
        
        for pin1 in comp1.pins:
            usage1 = pin_usage_count[comp1.id][pin1.pin_id]
            pos1 = comp1.get_pin_absolute_position(pin1)
            
            for pin2 in comp2.pins:
                usage2 = pin_usage_count[comp2.id][pin2.pin_id]
                pos2 = comp2.get_pin_absolute_position(pin2)
                
                distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                
                # 优先级分数：未使用的引脚数量（0, 1, 或 2）
                unused_count = (1 if usage1 == 0 else 0) + (1 if usage2 == 0 else 0)
                # 总使用次数（越少越好）
                total_usage = usage1 + usage2
                
                candidates.append({
                    'pin1': pin1,
                    'pin2': pin2,
                    'distance': distance,
                    'unused_count': unused_count,
                    'total_usage': total_usage
                })
        
        if not candidates:
            return None
        
        # 排序：优先未使用数量多的，然后总使用次数少的，最后距离近的
        candidates.sort(key=lambda x: (-x['unused_count'], x['total_usage'], x['distance']))
        
        best = candidates[0]
        return (best['pin1'], best['pin2'], best['distance'])


    def _ensure_internal_connectivity(self, components: List[Component], 
                                    edges: List[Edge],
                                    pin_usage_count: defaultdict = None) -> List[Edge]:
        """
        确保单个 Micro 内部所有组件连通（消除孤岛）
        """
        if len(components) <= 1:
            return edges
        
        comp_ids = [c.id for c in components]
        comp_map = {c.id: c for c in components}
        parent = {cid: cid for cid in comp_ids}
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
        
        # 如果没有传入 pin_usage_count，从 edges 统计
        if pin_usage_count is None:
            pin_usage_count = defaultdict(lambda: defaultdict(int))
            for edge in edges:
                pin_usage_count[edge.source_comp][edge.source_pin] += 1
                pin_usage_count[edge.target_comp][edge.target_pin] += 1
        
        for edge in edges:
            if edge.source_comp in comp_map and edge.target_comp in comp_map:
                union(edge.source_comp, edge.target_comp)
        
        roots = set(find(cid) for cid in comp_ids)
        
        if len(roots) == 1:
            return edges
        
        # 预计算所有组件对之间的最佳引脚对
        component_pairs = []
        for i, comp1 in enumerate(components):
            for j, comp2 in enumerate(components):
                if i >= j:
                    continue
                
                best_pair = self._find_best_pin_pair(comp1, comp2, pin_usage_count)
                if best_pair:
                    pin1, pin2, distance = best_pair
                    component_pairs.append({
                        'comp1': comp1,
                        'comp2': comp2,
                        'pin1': pin1,
                        'pin2': pin2,
                        'distance': distance
                    })
        
        # 按距离排序
        component_pairs.sort(key=lambda x: x['distance'])
        
        for pair in component_pairs:
            comp1, comp2 = pair['comp1'], pair['comp2']
            
            if find(comp1.id) == find(comp2.id):
                continue
            
            # 重新找最佳引脚对（因为之前的迭代可能改变了使用情况）
            best_pair = self._find_best_pin_pair(comp1, comp2, pin_usage_count)
            if best_pair is None:
                continue
            
            pin1, pin2, distance = best_pair
            
            net_name = f"BRIDGE_{comp1.id}_{comp2.id}"
            
            if pin1.net and pin1.net != "":
                net_name = pin1.net
            elif pin2.net and pin2.net != "":
                net_name = pin2.net
            
            if pin1.net == "":
                pin1.net = net_name
            if pin2.net == "":
                pin2.net = net_name
            
            edges.append(Edge(
                source_comp=comp1.id,
                source_pin=pin1.pin_id,
                target_comp=comp2.id,
                target_pin=pin2.pin_id,
                net=net_name,
                distance=distance
            ))
            
            pin_usage_count[comp1.id][pin1.pin_id] += 1
            pin_usage_count[comp2.id][pin2.pin_id] += 1
            
            union(comp1.id, comp2.id)
            
            roots = set(find(cid) for cid in comp_ids)
            if len(roots) == 1:
                break
        
        return edges


    def _ensure_min_edges_per_component(self, components: List[Component],
                                        edges: List[Edge],
                                        pin_usage_count: defaultdict = None) -> List[Edge]:
        """
        确保每个组件至少有 min_edges_per_component 个引脚被使用
        注意：一个器件的最大连接数受限于其引脚数量（每个引脚至少被使用一次才算一个有效连接）
        """
        min_edges = self.config.min_edges_per_component
        
        if min_edges <= 0:
            return edges
        
        # 如果没有传入 pin_usage_count，从 edges 统计
        if pin_usage_count is None:
            pin_usage_count = defaultdict(lambda: defaultdict(int))
            for edge in edges:
                pin_usage_count[edge.source_comp][edge.source_pin] += 1
                pin_usage_count[edge.target_comp][edge.target_pin] += 1
        
        comp_map = {c.id: c for c in components}
        
        # 找出"已使用引脚数"不足的组件
        deficient_comps = []
        for comp in components:
            # 计算已使用的引脚数量
            used_pin_count = sum(1 for pin in comp.pins 
                            if pin_usage_count[comp.id][pin.pin_id] > 0)
            max_possible = len(comp.pins)
            target = min(min_edges, max_possible)
            
            if used_pin_count < target:
                deficient_comps.append((comp, target - used_pin_count))
        
        if not deficient_comps:
            return edges
        
        for comp, needed in deficient_comps:
            if needed <= 0:
                continue
            
            # 获取该组件的未使用引脚
            unused_pins = [p for p in comp.pins 
                        if pin_usage_count[comp.id][p.pin_id] == 0]
            
            if not unused_pins:
                continue
            
            # 找候选组件（任何其他组件都可以）
            candidates = []
            
            for other in components:
                if other.id == comp.id:
                    continue
                
                # 对于每个未使用的引脚，找到 other 上的最佳引脚
                for pin1 in unused_pins:
                    pos1 = comp.get_pin_absolute_position(pin1)
                    
                    # 优先选择 other 上未使用的引脚
                    best_pin2 = None
                    best_dist = float('inf')
                    best_usage = float('inf')
                    
                    for pin2 in other.pins:
                        usage2 = pin_usage_count[other.id][pin2.pin_id]
                        pos2 = other.get_pin_absolute_position(pin2)
                        dist = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                        
                        # 优先未使用的，然后距离近的
                        if usage2 < best_usage or (usage2 == best_usage and dist < best_dist):
                            best_usage = usage2
                            best_dist = dist
                            best_pin2 = pin2
                    
                    if best_pin2:
                        candidates.append({
                            'other': other,
                            'pin1': pin1,
                            'pin2': best_pin2,
                            'distance': best_dist,
                            'other_usage': best_usage
                        })
            
            # 排序：优先 other 引脚未使用的，然后距离近的
            candidates.sort(key=lambda x: (x['other_usage'], x['distance']))
            
            # 添加连接
            added = 0
            used_local_pins = set()  # 本轮已使用的 comp 的引脚
            
            for cand in candidates:
                if added >= needed:
                    break
                
                pin1 = cand['pin1']
                
                # 确保这个引脚在本轮还没被使用
                if pin1.pin_id in used_local_pins:
                    continue
                
                pin2 = cand['pin2']
                other = cand['other']
                
                # 生成网络名称
                if pin1.net and pin1.net != "":
                    net_name = pin1.net
                elif pin2.net and pin2.net != "":
                    net_name = pin2.net
                else:
                    net_name = f"MIN_CONN_{comp.id}_{other.id}"
                
                if pin1.net == "":
                    pin1.net = net_name
                if pin2.net == "":
                    pin2.net = net_name
                
                edges.append(Edge(
                    source_comp=comp.id,
                    source_pin=pin1.pin_id,
                    target_comp=other.id,
                    target_pin=pin2.pin_id,
                    net=net_name,
                    distance=cand['distance']
                ))
                
                # 更新状态
                pin_usage_count[comp.id][pin1.pin_id] += 1
                pin_usage_count[other.id][pin2.pin_id] += 1
                used_local_pins.add(pin1.pin_id)
                added += 1
        
        return edges






    def _generate_cross_micro_edges(self, micro_regions: List[MicroRegion]) -> List[Edge]:
        """生成跨 Micro 的边，确保每对相邻 Micro 之间至少有连接"""
        edges = []
        
        if len(micro_regions) == 1:
            return edges
        
        gamma = self.config.cross_micro_gamma
        
        # 记录每对 Micro 之间是否已有连接
        micro_pairs_connected = set()
        
        for i, micro1 in enumerate(micro_regions):
            for j, micro2 in enumerate(micro_regions):
                if i >= j:
                    continue
                
                pair_key = (micro1.id, micro2.id)
                pair_edges = []  # 该对 Micro 之间的候选边
                
                micro_distance = math.sqrt(
                    (micro1.center[0] - micro2.center[0])**2 +
                    (micro1.center[1] - micro2.center[1])**2
                )
                
                boundary_pins_1 = self._get_boundary_pins(micro1, micro2.center)
                boundary_pins_2 = self._get_boundary_pins(micro2, micro1.center)
                
                # 如果边界引脚为空，使用所有引脚
                if not boundary_pins_1:
                    boundary_pins_1 = [
                        (comp, pin) 
                        for comp in micro1.components 
                        for pin in comp.pins
                    ]
                if not boundary_pins_2:
                    boundary_pins_2 = [
                        (comp, pin) 
                        for comp in micro2.components 
                        for pin in comp.pins
                    ]
                
                # 收集所有候选边及其权重
                candidate_edges = []
                
                for comp1, pin1 in boundary_pins_1:
                    for comp2, pin2 in boundary_pins_2:
                        same_net = (pin1.net == pin2.net and pin1.net != "" and
                                "GLOBAL" in pin1.net)
                        
                        pos1 = comp1.get_pin_absolute_position(pin1)
                        pos2 = comp2.get_pin_absolute_position(pin2)
                        distance = math.sqrt(
                            (pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2
                        )
                        
                        canvas_diag = math.sqrt(
                            self.config.canvas_width**2 + self.config.canvas_height**2
                        )
                        norm_distance = distance / canvas_diag
                        
                        prob = min(
                            gamma * math.exp(-norm_distance / self.config.scale_parameter),
                            self.config.cross_micro_connection_prob
                        )
                        
                        if same_net:
                            prob = min(prob * 5, 0.8)
                        
                        candidate_edges.append({
                            'comp1': comp1,
                            'pin1': pin1,
                            'comp2': comp2,
                            'pin2': pin2,
                            'distance': distance,
                            'prob': prob,
                            'same_net': same_net
                        })
                
                # 按概率采样边
                for candidate in candidate_edges:
                    if np.random.random() < candidate['prob']:
                        comp1, pin1 = candidate['comp1'], candidate['pin1']
                        comp2, pin2 = candidate['comp2'], candidate['pin2']
                        
                        if candidate['same_net']:
                            net_name = pin1.net
                        else:
                            net_name = f"CROSS_{micro1.id}_{micro2.id}_{len(edges) + len(pair_edges)}"
                            pin1.net = net_name
                            pin2.net = net_name
                        
                        pair_edges.append(Edge(
                            source_comp=comp1.id,
                            source_pin=pin1.pin_id,
                            target_comp=comp2.id,
                            target_pin=pin2.pin_id,
                            net=net_name,
                            distance=candidate['distance']
                        ))
                
                # 确保每对 Micro 之间至少有 min_cross_edges 条边
                min_cross_edges = getattr(self.config, 'min_cross_micro_edges', 1)
                
                if len(pair_edges) < min_cross_edges and candidate_edges:
                    # 按距离排序，选择最近的边
                    candidate_edges.sort(key=lambda x: x['distance'])
                    
                    for candidate in candidate_edges:
                        if len(pair_edges) >= min_cross_edges:
                            break
                        
                        comp1, pin1 = candidate['comp1'], candidate['pin1']
                        comp2, pin2 = candidate['comp2'], candidate['pin2']
                        
                        # 检查是否已经添加了这对引脚的边
                        already_added = any(
                            e.source_comp == comp1.id and e.source_pin == pin1.pin_id and
                            e.target_comp == comp2.id and e.target_pin == pin2.pin_id
                            for e in pair_edges
                        )
                        
                        if already_added:
                            continue
                        
                        if candidate['same_net']:
                            net_name = pin1.net
                        else:
                            net_name = f"CROSS_{micro1.id}_{micro2.id}_{len(edges) + len(pair_edges)}"
                            pin1.net = net_name
                            pin2.net = net_name
                        
                        pair_edges.append(Edge(
                            source_comp=comp1.id,
                            source_pin=pin1.pin_id,
                            target_comp=comp2.id,
                            target_pin=pin2.pin_id,
                            net=net_name,
                            distance=candidate['distance']
                        ))
                
                edges.extend(pair_edges)
                if pair_edges:
                    micro_pairs_connected.add(pair_key)
        
        # 最终检查：确保图连通（使用最小生成树的思想）
        edges = self._ensure_micro_connectivity(micro_regions, edges)
        
        return edges


    def _ensure_micro_connectivity(self, micro_regions: List[MicroRegion], 
                                edges: List[Edge]) -> List[Edge]:
        """确保所有 Micro 区域通过边连通"""
        if len(micro_regions) <= 1:
            return edges
        
        # 构建 Micro 连通图
        micro_ids = [m.id for m in micro_regions]
        micro_map = {m.id: m for m in micro_regions}
        
        # 找出每条边连接的 Micro
        def get_component_micro(comp_id: str) -> str:
            for micro in micro_regions:
                for comp in micro.components:
                    if comp.id == comp_id:
                        return micro.id
            return None
        
        # 构建已连接的 Micro 对
        connected_pairs = set()
        for edge in edges:
            micro1 = get_component_micro(edge.source_comp)
            micro2 = get_component_micro(edge.target_comp)
            if micro1 and micro2 and micro1 != micro2:
                pair = tuple(sorted([micro1, micro2]))
                connected_pairs.add(pair)
        
        # 使用并查集检查连通性
        parent = {mid: mid for mid in micro_ids}
        
        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            px, py = find(x), find(y)
            if px != py:
                parent[px] = py
                return True
            return False
        
        for m1, m2 in connected_pairs:
            union(m1, m2)
        
        # 检查是否所有 Micro 都连通
        roots = set(find(mid) for mid in micro_ids)
        
        if len(roots) == 1:
            return edges  # 已经连通
        
        # 需要添加边来连接不同的连通分量
        print(f"  Warning: {len(roots)} disconnected Micro groups, adding bridge edges...")
        
        # 按 Micro 中心距离排序所有 Micro 对
        micro_pairs_by_distance = []
        for i, m1 in enumerate(micro_regions):
            for j, m2 in enumerate(micro_regions):
                if i >= j:
                    continue
                dist = math.sqrt(
                    (m1.center[0] - m2.center[0])**2 +
                    (m1.center[1] - m2.center[1])**2
                )
                micro_pairs_by_distance.append((dist, m1, m2))
        
        micro_pairs_by_distance.sort(key=lambda x: x[0])
        
        # 添加边直到所有 Micro 连通
        for dist, m1, m2 in micro_pairs_by_distance:
            if find(m1.id) == find(m2.id):
                continue  # 已经在同一连通分量
            
            # 找到两个 Micro 之间最近的引脚对
            best_edge = None
            best_distance = float('inf')
            
            for comp1 in m1.components:
                for pin1 in comp1.pins:
                    pos1 = comp1.get_pin_absolute_position(pin1)
                    for comp2 in m2.components:
                        for pin2 in comp2.pins:
                            pos2 = comp2.get_pin_absolute_position(pin2)
                            d = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                            if d < best_distance:
                                best_distance = d
                                best_edge = (comp1, pin1, comp2, pin2)
            
            if best_edge:
                comp1, pin1, comp2, pin2 = best_edge
                net_name = f"BRIDGE_{m1.id}_{m2.id}"
                
                # 更新引脚的 net
                if pin1.net == "":
                    pin1.net = net_name
                if pin2.net == "":
                    pin2.net = net_name
                
                edges.append(Edge(
                    source_comp=comp1.id,
                    source_pin=pin1.pin_id,
                    target_comp=comp2.id,
                    target_pin=pin2.pin_id,
                    net=net_name,
                    distance=best_distance
                ))
                
                union(m1.id, m2.id)
                print(f"    Added bridge edge: {m1.id} <-> {m2.id}")
            
            # 检查是否已全部连通
            roots = set(find(mid) for mid in micro_ids)
            if len(roots) == 1:
                break
        
        return edges



    def _get_boundary_pins(self, micro: MicroRegion, 
                          target_center: List[float]) -> List[Tuple[Component, Pin]]:
        """获取靠近目标中心的边界引脚"""
        boundary_pins = []
        
        direction = [
            target_center[0] - micro.center[0],
            target_center[1] - micro.center[1]
        ]
        
        for comp in micro.components:
            for pin in comp.pins:
                pos = comp.get_pin_absolute_position(pin)
                
                rel_pos = [
                    pos[0] - micro.center[0],
                    pos[1] - micro.center[1]
                ]
                
                dot_product = rel_pos[0] * direction[0] + rel_pos[1] * direction[1]
                
                if dot_product > 0:
                    boundary_pins.append((comp, pin))
        
        if len(boundary_pins) > 10:
            boundary_pins = random.sample(boundary_pins, 10)
        
        return boundary_pins
    
    def _build_net_connections(self, components: List[Component]) -> List[NetConnection]:
        """构建网络连接信息"""
        net_pins = defaultdict(list)
        
        for comp in components:
            for pin in comp.pins:
                if pin.net:
                    net_pins[pin.net].append({
                        "component": comp.id,
                        "pin": pin.pin_id
                    })
        
        nets = []
        for net_name, pins in net_pins.items():
            if len(pins) >= 2:
                net_type = self._classify_net_type(net_name)
                nets.append(NetConnection(
                    net_id=net_name,
                    net_type=net_type,
                    pins=pins
                ))
        
        return nets
    
    def _classify_net_type(self, net_name: str) -> NetType:
        """分类网络类型"""
        name_upper = net_name.upper()
        if "GND" in name_upper or "VSS" in name_upper:
            return NetType.GROUND
        elif "VCC" in name_upper or "VDD" in name_upper or "PWR" in name_upper:
            return NetType.POWER
        else:
            return NetType.SIGNAL
    
    def _build_placement_dict(self, circuit_id: str, 
                              components: List[Component]) -> Dict:
        """构建 Placement 字典"""
        return {
            "circuit_id": circuit_id,
            "canvas": {
                "width": self.config.canvas_width,
                "height": self.config.canvas_height
            },
            "components": [
                {
                    "id": comp.id,
                    "type": comp.comp_type,
                    "position": comp.position,
                    "size": comp.size,
                    "rotation": comp.rotation,
                    "micro_id": comp.micro_id,
                    "pins": [
                        {
                            "pin_id": pin.pin_id,
                            "rel_pos": pin.rel_pos,
                            "size": pin.size,
                            "net": pin.net,
                            "rotation": pin.rotation
                        }
                        for pin in comp.pins
                    ]
                }
                for comp in components
            ]
        }
    
    def _build_netlist_dict(self, circuit_id: str, 
                           nets: List[NetConnection], 
                           edges: List[Edge]) -> Dict:
        """构建 Netlist 字典"""
        return {
            "circuit_id": circuit_id,
            "nets": [
                {
                    "net_id": net.net_id,
                    "net_type": net.net_type.value,
                    "pins": net.pins
                }
                for net in nets
            ],
            "edges": [
                {
                    "source": {"component": edge.source_comp, "pin": edge.source_pin},
                    "target": {"component": edge.target_comp, "pin": edge.target_pin},
                    "net": edge.net,
                    "distance": edge.distance
                }
                for edge in edges
            ]
        }