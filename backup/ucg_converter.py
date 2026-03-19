"""
UCG 格式转换器
"""

import numpy as np
from typing import List, Dict


class UCGConverter:
    """将放置结果转换为UCG格式"""
    
    def convert_to_ucg(self, circuit_data: Dict) -> Dict:
        """转换为UCG格式"""
        placement = circuit_data["placement"]
        netlist = circuit_data["netlist"]
        
        micro_info = circuit_data["metadata"].get("micro_info", {})
        micro_components = self._group_by_micro(placement["components"])
        
        level_0 = self._build_level_0(micro_components, micro_info, netlist)
        level_1 = self._build_level_1(micro_components, netlist)
        
        return {
            "UCG_Graph": {
                "level_0_global": level_0,
                "level_1_details": level_1
            }
        }
    
    def _group_by_micro(self, components: List[Dict]) -> Dict[str, List[Dict]]:
        """按Micro ID分组元件"""
        groups = {}
        for comp in components:
            micro_id = comp.get("micro_id", "Micro_0")
            if micro_id not in groups:
                groups[micro_id] = []
            groups[micro_id].append(comp)
        return groups
    
    def _build_level_0(self, micro_components: Dict, micro_info: Dict, 
                       netlist: Dict) -> Dict:
        """构建Level 0全局视图"""
        nodes = []
        
        for micro_id, components in micro_components.items():
            if micro_id in micro_info:
                bbox = micro_info[micro_id]["bbox"]
                w = round(bbox[2] - bbox[0], 2)
                h = round(bbox[3] - bbox[1], 2)
                center_x = (bbox[0] + bbox[2]) / 2
                center_y = (bbox[1] + bbox[3]) / 2
            else:
                bbox = self._calculate_bbox(components)
                w = round(bbox["max_x"] - bbox["min_x"], 2)
                h = round(bbox["max_y"] - bbox["min_y"], 2)
                center_x = bbox["center_x"]
                center_y = bbox["center_y"]
            
            nodes.append({
                "id": micro_id,
                "type": "Micro",
                "w": w,
                "h": h,
                "center_x": round(center_x, 2),
                "center_y": round(center_y, 2),
                "num_components": len(components)
            })
        
        spatial_topology = []
        node_list = list(nodes)
        for i, n1 in enumerate(node_list):
            for j, n2 in enumerate(node_list):
                if i >= j:
                    continue
                dx = round(n2["center_x"] - n1["center_x"], 2)
                dy = round(n2["center_y"] - n1["center_y"], 2)
                spatial_topology.append({
                    "type": "SPATIAL",
                    "source": n1["id"],
                    "target": n2["id"],
                    "dx": dx,
                    "dy": dy
                })
        
        routing_resources = self._extract_cross_micro_routing(
            micro_components, netlist
        )
        
        return {
            "nodes": nodes,
            "spatial_topology": spatial_topology,
            "routing_resources": routing_resources
        }
    
    def _build_level_1(self, micro_components: Dict, netlist: Dict) -> Dict:
        """构建Level 1详细视图"""
        level_1 = {}
        
        for micro_id, components in micro_components.items():
            nodes = []
            for comp in components:
                w, h = comp["size"]
                if comp["rotation"] in [90, 270, -90]:
                    w, h = h, w
                nodes.append({
                    "id": comp["id"],
                    "type": comp["type"],
                    "w": round(w, 2),
                    "h": round(h, 2),
                    "x": round(comp["position"][0], 2),
                    "y": round(comp["position"][1], 2),
                    "rotation": comp["rotation"]
                })
            
            spatial_topology = self._build_internal_spatial(components)
            routing_resources = self._extract_internal_routing(components, netlist)
            
            level_1[micro_id] = {
                "nodes": nodes,
                "spatial_topology": spatial_topology,
                "routing_resources": routing_resources
            }
        
        return level_1
    
    def _calculate_bbox(self, components: List[Dict]) -> Dict:
        """计算组件的实际包围盒"""
        if not components:
            return {"min_x": 0, "max_x": 0, "min_y": 0, "max_y": 0,
                    "center_x": 0, "center_y": 0}
        
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        
        for comp in components:
            x, y = comp["position"]
            w, h = comp["size"]
            if comp["rotation"] in [90, 270, -90]:
                w, h = h, w
            
            min_x = min(min_x, x - w/2)
            max_x = max(max_x, x + w/2)
            min_y = min(min_y, y - h/2)
            max_y = max(max_y, y + h/2)
        
        return {
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "center_x": (min_x + max_x) / 2,
            "center_y": (min_y + max_y) / 2
        }
    
    def _build_internal_spatial(self, components: List[Dict], 
                                 max_neighbors: int = 4) -> List[Dict]:
        """构建Micro内部空间拓扑（KNN方式）"""
        if len(components) < 2:
            return []
        
        positions = np.array([c["position"] for c in components])
        topology = []
        
        for i, comp in enumerate(components):
            distances = np.sqrt(np.sum((positions - positions[i])**2, axis=1))
            distances[i] = float('inf')
            
            k = min(max_neighbors, len(components) - 1)
            nearest_indices = np.argsort(distances)[:k]
            
            for j in nearest_indices:
                if i < j:
                    topology.append({
                        "type": "SPATIAL",
                        "source": comp["id"],
                        "target": components[j]["id"],
                        "distance": round(float(distances[j]), 2)
                    })
        
        seen = set()
        unique_topology = []
        for edge in topology:
            key = (edge["source"], edge["target"])
            if key not in seen:
                seen.add(key)
                unique_topology.append(edge)
        
        return unique_topology
    
    def _extract_cross_micro_routing(self, micro_components: Dict, 
                                     netlist: Dict) -> List[Dict]:
        """提取跨Micro的路由连接"""
        comp_to_micro = {}
        for micro_id, components in micro_components.items():
            for comp in components:
                comp_to_micro[comp["id"]] = micro_id
        
        cross_routes = []
        for edge in netlist["edges"]:
            src_micro = comp_to_micro.get(edge["source"]["component"])
            tgt_micro = comp_to_micro.get(edge["target"]["component"])
            
            if src_micro and tgt_micro and src_micro != tgt_micro:
                cross_routes.append({
                    "net": edge["net"],
                    "source_micro": src_micro,
                    "target_micro": tgt_micro,
                    "source_comp": edge["source"]["component"],
                    "target_comp": edge["target"]["component"]
                })
        
        return cross_routes
    
    def _extract_internal_routing(self, components: List[Dict], 
                                  netlist: Dict) -> List[Dict]:
        """提取Micro内部的路由连接"""
        comp_ids = {c["id"] for c in components}
        
        internal_routes = []
        for edge in netlist["edges"]:
            src = edge["source"]["component"]
            tgt = edge["target"]["component"]
            
            if src in comp_ids and tgt in comp_ids:
                internal_routes.append({
                    "net": edge["net"],
                    "source": edge["source"],
                    "target": edge["target"]
                })
        
        return internal_routes