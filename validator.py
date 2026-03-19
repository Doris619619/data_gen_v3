"""
数据集验证和统计工具
"""

import json
from collections import defaultdict, deque
from typing import Any, List, Dict, Set, Tuple
import numpy as np


class DatasetValidator:
    """数据集验证器"""
    
    def validate_circuit(self, circuit_data: Dict) -> Dict:
        """验证单个电路数据"""
        issues = []
        warnings = []
        
        placement = circuit_data["placement"]
        netlist = circuit_data["netlist"]
        
        # 检查元件重叠
        overlaps = self._check_overlaps(placement["components"])
        if overlaps:
            issues.extend([f"Overlap detected: {o}" for o in overlaps[:5]])
        
        # 检查边界
        out_of_bounds = self._check_bounds(placement)
        if out_of_bounds:
            warnings.extend([f"Out of bounds: {o}" for o in out_of_bounds[:5]])
        
        # 检查孤立网络
        isolated = self._check_isolated_nets(netlist)
        if isolated:
            warnings.extend([f"Isolated net: {n}" for n in isolated[:5]])
        
        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "warnings": warnings
        }
    
    def _check_overlaps(self, components: List[Dict]) -> List[str]:
        """检查元件重叠"""
        overlaps = []
        for i, c1 in enumerate(components):
            for j, c2 in enumerate(components):
                if i >= j:
                    continue
                
                if self._components_overlap(c1, c2):
                    overlaps.append(f"{c1['id']} <-> {c2['id']}")
        
        return overlaps
    
    def _components_overlap(self, c1: Dict, c2: Dict) -> bool:
        """检查两个元件是否重叠"""
        x1, y1 = c1["position"]
        w1, h1 = c1["size"]
        x2, y2 = c2["position"]
        w2, h2 = c2["size"]
        
        return not (x1 + w1/2 < x2 - w2/2 or x2 + w2/2 < x1 - w1/2 or
                   y1 + h1/2 < y2 - h2/2 or y2 + h2/2 < y1 - h1/2)
    
    def _check_bounds(self, placement: Dict) -> List[str]:
        """检查边界"""
        out = []
        canvas_w = placement["canvas"]["width"]
        canvas_h = placement["canvas"]["height"]
        
        for comp in placement["components"]:
            x, y = comp["position"]
            w, h = comp["size"]
            
            if x - w/2 < 0 or x + w/2 > canvas_w or \
               y - h/2 < 0 or y + h/2 > canvas_h:
                out.append(comp["id"])
        
        return out
    
    def _check_isolated_nets(self, netlist: Dict) -> List[str]:
        """检查孤立网络（只有一个引脚）"""
        return [net["net_id"] for net in netlist["nets"] if len(net["pins"]) < 2]


class DatasetStatistics:
    """数据集统计分析"""
    
    def analyze_dataset(self, dataset: List[Dict]) -> Dict:
        """分析整个数据集"""
        stats = {
            "num_circuits": len(dataset),
            "components": [],
            "edges": [],
            "nets": [],
            "densities": [],
            "scale_params": []
        }
        
        for circuit in dataset:
            meta = circuit["metadata"]
            stats["components"].append(meta["num_components"])
            stats["edges"].append(meta["num_edges"])
            stats["nets"].append(meta["num_nets"])
            # 这些字段可能不存在于新版本的 metadata 中
            if "actual_density" in meta:
                stats["densities"].append(meta["actual_density"])
            if "scale_parameter" in meta:
                stats["scale_params"].append(meta["scale_parameter"])
        
        # 计算统计量
        summary = {
            "num_circuits": len(dataset),
            "components": self._compute_stats(stats["components"]),
            "edges": self._compute_stats(stats["edges"]),
            "nets": self._compute_stats(stats["nets"]),
        }
        
        if stats["densities"]:
            summary["densities"] = self._compute_stats(stats["densities"])
        if stats["scale_params"]:
            summary["scale_params"] = self._compute_stats(stats["scale_params"])
        
        return summary
    
    def _compute_stats(self, values: List) -> Dict:
        """计算统计量"""
        if not values:
            return {"mean": 0, "std": 0, "min": 0, "max": 0, "median": 0}
        
        arr = np.array(values)
        return {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "median": float(np.median(arr))
        }
    
    def print_summary(self, summary: Dict):
        """打印统计摘要"""
        print("\n" + "=" * 50)
        print("Dataset Statistics Summary")
        print("=" * 50)
        print(f"Total circuits: {summary['num_circuits']}")
        
        for key in ["components", "edges", "nets", "densities", "scale_params"]:
            if key in summary:
                s = summary[key]
                print(f"\n{key.upper()}:")
                print(f"  Mean: {s['mean']:.2f} ± {s['std']:.2f}")
                print(f"  Range: [{s['min']:.2f}, {s['max']:.2f}]")
                print(f"  Median: {s['median']:.2f}")


class PromptUCGValidator:
    """按 prompt 约束验证 UCG 输出"""

    VALID_SPACE_SIZES = {"Small", "Medium", "Large"}

    def __init__(self):
        self.small_width = 3.0
        self.medium_width = 6.0
        self.large_width = 8.0

    def validate(self, ucg_data: Dict, metadata: Dict = None) -> Dict:
        errors: List[str] = []
        warnings: List[str] = []
        metadata = metadata or {}

        # 1) JSON 合法性
        try:
            json.dumps(ucg_data)
        except Exception as ex:
            errors.append(f"JSON serialization failed: {ex}")
            return {"valid": False, "errors": errors, "warnings": warnings}

        root = self._get_ucg_root(ucg_data)
        level_0 = root.get("level_0_global", {})
        level_1 = root.get("level_1_details", {})

        self._check_rot_and_pads(level_0, level_1, errors)
        self._check_spatial_float_strict(level_0, level_1, errors)
        self._check_spatial_dag(level_0, level_1, errors)

        level0_node_ids = [n.get("id") for n in level_0.get("nodes", []) if n.get("id")]
        self._check_spatial_anchor_generic(
            level="level_0_global",
            node_ids=level0_node_ids,
            edges=level_0.get("spatial_topology", []),
            errors=errors,
        )

        for micro_id, detail in level_1.items():
            node_ids = [n.get("id") for n in detail.get("nodes", []) if n.get("id")]
            self._check_spatial_anchor_generic(
                level=micro_id,
                node_ids=node_ids,
                edges=detail.get("spatial_topology", []),
                errors=errors,
            )

        self._check_space_node_consistency(level_0, level_1, errors)
        self._check_spacing_formula(level_0, level_1, errors)

        self._check_level0_routing_syntax(level_0, errors)
        self._check_level1_routing_constraints(level_1, errors)
        self._check_level1_same_net_coverage(level_1, errors)

        self._check_micros_coverage(level_1, metadata, errors, warnings)

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
        }

    def _get_ucg_root(self, ucg_data: Dict) -> Dict:
        if "UCG_Graph" in ucg_data and isinstance(ucg_data["UCG_Graph"], dict):
            return ucg_data["UCG_Graph"]
        return ucg_data

    def _check_rot_and_pads(self, level_0: Dict, level_1: Dict, errors: List[str]):
        for node in level_0.get("nodes", []):
            if "rot" not in node:
                errors.append(f"level_0 node missing rot: {node.get('id', 'UNKNOWN')}")

        for micro_id, detail in level_1.items():
            for node in detail.get("nodes", []):
                node_id = node.get("id", "UNKNOWN")
                if "rot" not in node:
                    errors.append(f"{micro_id}.{node_id} missing rot")
                pads = node.get("pads")
                if not isinstance(pads, dict):
                    errors.append(f"{micro_id}.{node_id} missing pads object")
                    continue
                for pad_id, pad in pads.items():
                    for field in ["rel_pos", "rot", "w", "h", "net"]:
                        if field not in pad:
                            errors.append(f"{micro_id}.{node_id}.{pad_id} missing field: {field}")

    def _check_spatial_float_strict(self, level_0: Dict, level_1: Dict, errors: List[str]):
        for edge in level_0.get("spatial_topology", []):
            if not self._is_python_float(edge.get("dx")):
                errors.append(f"level_0 spatial edge dx is not float: {edge}")
            if not self._is_python_float(edge.get("dy")):
                errors.append(f"level_0 spatial edge dy is not float: {edge}")

        for micro_id, detail in level_1.items():
            for edge in detail.get("spatial_topology", []):
                if not self._is_python_float(edge.get("dx")):
                    errors.append(f"{micro_id} spatial edge dx is not float: {edge}")
                if not self._is_python_float(edge.get("dy")):
                    errors.append(f"{micro_id} spatial edge dy is not float: {edge}")

    def _check_spatial_dag(self, level_0: Dict, level_1: Dict, errors: List[str]):
        level0_nodes = [n.get("id") for n in level_0.get("nodes", []) if n.get("id")]
        if self._has_cycle(level0_nodes, level_0.get("spatial_topology", [])):
            errors.append("level_0 spatial_topology is not DAG")

        for micro_id, detail in level_1.items():
            node_ids = [n.get("id") for n in detail.get("nodes", []) if n.get("id")]
            if self._has_cycle(node_ids, detail.get("spatial_topology", [])):
                errors.append(f"{micro_id} spatial_topology is not DAG")

    def _check_spatial_anchor_generic(self, level: str, node_ids: List[str],
                                      edges: List[Dict], errors: List[str]):
        n = len(node_ids)
        if n <= 1:
            return
        if len(edges) < n - 1:
            errors.append(f"{level} spatial_topology has fewer than n-1 edges")

        if not self._is_connected_undirected(node_ids, edges):
            errors.append(f"{level} nodes are not fully anchored in spatial_topology")

    def _check_space_node_consistency(self, level_0: Dict, level_1: Dict, errors: List[str]):
        self._check_space_node_consistency_for_resources(
            level_name="level_0_global",
            resources=level_0.get("routing_resources", []),
            errors=errors,
        )

        for micro_id, detail in level_1.items():
            self._check_space_node_consistency_for_resources(
                level_name=micro_id,
                resources=detail.get("routing_resources", []),
                errors=errors,
            )

    def _check_space_node_consistency_for_resources(self, level_name: str,
                                                    resources: List[Dict],
                                                    errors: List[str]):
        for idx, rr in enumerate(resources):
            seq = rr.get("path_sequence")
            if not isinstance(seq, list) or len(seq) < 3:
                errors.append(f"{level_name} routing_resources[{idx}] path_sequence malformed")
                continue

            space_node = rr.get("space_node")
            if not isinstance(space_node, dict):
                errors.append(f"{level_name} routing_resources[{idx}] missing space_node dict")
                continue

            sn_id = space_node.get("id")
            sn_size = space_node.get("size")
            if not isinstance(sn_id, str) or not sn_id:
                errors.append(f"{level_name} routing_resources[{idx}] space_node.id missing")
            if not isinstance(sn_size, str) or sn_size not in self.VALID_SPACE_SIZES:
                errors.append(f"{level_name} routing_resources[{idx}] invalid space_node.size={sn_size}")

            mid_token = seq[1]
            if not isinstance(mid_token, str) or not mid_token.startswith("SN_"):
                errors.append(f"{level_name} routing_resources[{idx}] path_sequence[1] is not SN token")
            if sn_id != mid_token:
                errors.append(f"{level_name} routing_resources[{idx}] SN mismatch: seq={mid_token} space_node.id={sn_id}")

    def _check_spacing_formula(self, level_0: Dict, level_1: Dict, errors: List[str]):
        level0_nodes = {n.get("id"): n for n in level_0.get("nodes", []) if n.get("id")}
        level0_net_map = self._build_level0_micro_net_map(level_1)
        self._check_spacing_for_level(
            level_name="level_0_global",
            edges=level_0.get("spatial_topology", []),
            node_map=level0_nodes,
            entity_net_map=level0_net_map,
            errors=errors,
        )

        for micro_id, detail in level_1.items():
            node_map = {n.get("id"): n for n in detail.get("nodes", []) if n.get("id")}
            entity_net_map = self._build_level1_component_net_map(detail)
            self._check_spacing_for_level(
                level_name=micro_id,
                edges=detail.get("spatial_topology", []),
                node_map=node_map,
                entity_net_map=entity_net_map,
                errors=errors,
            )

    def _check_spacing_for_level(self, level_name: str, edges: List[Dict],
                                 node_map: Dict[str, Dict],
                                 entity_net_map: Dict[str, Set[str]],
                                 errors: List[str]):
        tol = 1e-3
        for idx, edge in enumerate(edges):
            source_id = edge.get("source")
            target_id = edge.get("target")
            if source_id not in node_map or target_id not in node_map:
                errors.append(f"{level_name} spatial edge[{idx}] has unknown node")
                continue

            src = node_map[source_id]
            tgt = node_map[target_id]
            src_w, src_h = self._effective_dimensions(src)
            tgt_w, tgt_h = self._effective_dimensions(tgt)

            shared_nets = entity_net_map.get(source_id, set()) & entity_net_map.get(target_id, set())
            if shared_nets:
                channel_width = max(self._channel_width_for_net(net_id) for net_id in shared_nets)
            else:
                channel_width = self.small_width

            dx = edge.get("dx")
            dy = edge.get("dy")
            if not self._is_python_float(dx) or not self._is_python_float(dy):
                continue

            abs_dx = abs(dx)
            abs_dy = abs(dy)
            min_dx = (src_w / 2.0) + channel_width + (tgt_w / 2.0)
            min_dy = (src_h / 2.0) + channel_width + (tgt_h / 2.0)

            if dy == 0.0:
                if abs_dx + tol < min_dx:
                    errors.append(f"{level_name} spatial edge[{idx}] violates horizontal spacing: |dx|={abs_dx} < {min_dx}")
            elif dx == 0.0:
                if abs_dy + tol < min_dy:
                    errors.append(f"{level_name} spatial edge[{idx}] violates vertical spacing: |dy|={abs_dy} < {min_dy}")
            else:
                if abs_dx + tol < min_dx:
                    errors.append(f"{level_name} spatial edge[{idx}] violates mixed spacing on x: |dx|={abs_dx} < {min_dx}")
                if abs_dy + tol < min_dy:
                    errors.append(f"{level_name} spatial edge[{idx}] violates mixed spacing on y: |dy|={abs_dy} < {min_dy}")

    def _check_level0_routing_syntax(self, level_0: Dict, errors: List[str]):
        for idx, rr in enumerate(level_0.get("routing_resources", [])):
            net_id = rr.get("net_id", "")
            seq = rr.get("path_sequence", [])
            if not isinstance(seq, list) or len(seq) < 3:
                errors.append(f"level_0 routing_resources[{idx}] path_sequence malformed")
                continue

            for token in [seq[0], seq[-1]]:
                if "." not in token:
                    errors.append(f"level_0 endpoint malformed: {token}")
                    continue
                left, right = token.split(".", 1)
                if not left.startswith("Micro_"):
                    errors.append(f"level_0 endpoint must start with Micro_: {token}")
                if "Comp_" in token or "Pad_" in token or "(" in token:
                    errors.append(f"level_0 endpoint contains component/pad syntax: {token}")
                if right != net_id:
                    errors.append(f"level_0 endpoint net mismatch with net_id={net_id}: {token}")

    def _check_level1_routing_constraints(self, level_1: Dict, errors: List[str]):
        for micro_id, detail in level_1.items():
            micro_num = self._extract_numeric_suffix(micro_id)
            node_pad_net = self._build_node_pad_net_map(detail)

            for idx, rr in enumerate(detail.get("routing_resources", [])):
                net_id = rr.get("net_id", "")
                seq = rr.get("path_sequence", [])
                if not isinstance(seq, list) or len(seq) < 3:
                    errors.append(f"{micro_id} routing_resources[{idx}] path_sequence malformed")
                    continue

                endpoints = [seq[0], seq[-1]]
                for endpoint in endpoints:
                    comp_id, pad_id, net_label = self._parse_level1_endpoint(endpoint)
                    if not comp_id or not pad_id:
                        errors.append(f"{micro_id} malformed endpoint: {endpoint}")
                        continue

                    # 3) level_1 不得跨 micro
                    expected_prefix = f"M{micro_num}_Comp_"
                    if not comp_id.startswith(expected_prefix):
                        errors.append(f"{micro_id} has cross-micro endpoint: {endpoint}")

                    # 4) path_sequence net label == net_id
                    if net_label != net_id:
                        errors.append(f"{micro_id} net label mismatch: {endpoint} vs net_id={net_id}")

                    # 5) endpoint pad 真实存在
                    if comp_id not in node_pad_net or pad_id not in node_pad_net[comp_id]:
                        errors.append(f"{micro_id} endpoint pad not found: {endpoint}")
                        continue

                    # 6) endpoint pad 的真实 net == net_id
                    real_net = node_pad_net[comp_id][pad_id]
                    if real_net != net_id:
                        errors.append(
                            f"{micro_id} endpoint real net mismatch: {endpoint} real={real_net} net_id={net_id}"
                        )

    def _check_level1_same_net_coverage(self, level_1: Dict, errors: List[str]):
        for micro_id, detail in level_1.items():
            net_to_all_endpoints: Dict[str, Set[str]] = defaultdict(set)
            for node in detail.get("nodes", []):
                comp_id = node.get("id")
                pads = node.get("pads", {})
                if not comp_id or not isinstance(pads, dict):
                    continue
                for pad_id, pad in pads.items():
                    if not isinstance(pad, dict):
                        continue
                    net_id = str(pad.get("net", ""))
                    if not net_id:
                        continue
                    endpoint = f"{comp_id}.{pad_id}({net_id})"
                    net_to_all_endpoints[net_id].add(endpoint)

            net_to_covered_endpoints: Dict[str, Set[str]] = defaultdict(set)
            for rr in detail.get("routing_resources", []):
                net_id = rr.get("net_id", "")
                seq = rr.get("path_sequence", [])
                if not isinstance(seq, list) or len(seq) < 3:
                    continue
                for token in [seq[0], seq[-1]]:
                    if isinstance(token, str) and token.endswith(f"({net_id})"):
                        net_to_covered_endpoints[net_id].add(token)

            for net_id, expected_endpoints in net_to_all_endpoints.items():
                if len(expected_endpoints) < 2:
                    continue
                covered = net_to_covered_endpoints.get(net_id, set())
                missing = sorted(expected_endpoints - covered)
                if missing:
                    errors.append(
                        f"{micro_id} net={net_id} has uncovered same-net pads: {missing[:4]}"
                    )

    def _check_micros_coverage(self, level_1: Dict, metadata: Dict,
                               errors: List[str], warnings: List[str]):
        expected_from_info = set(metadata.get("micro_info", {}).keys())
        expected_count = metadata.get("num_micros")
        actual_set = set(level_1.keys())

        if expected_from_info and actual_set != expected_from_info:
            errors.append(
                f"level_1 micros mismatch: actual={sorted(actual_set)} expected={sorted(expected_from_info)}"
            )

        if isinstance(expected_count, int) and len(actual_set) != expected_count:
            errors.append(
                f"level_1 micro count mismatch: actual={len(actual_set)} expected={expected_count}"
            )

        if "Micro_N" in actual_set:
            warnings.append("Found placeholder Micro_N in output")

    def _has_cycle(self, node_ids: List[str], edges: List[Dict]) -> bool:
        graph = defaultdict(list)
        indeg = {n: 0 for n in node_ids}

        for edge in edges:
            s = edge.get("source")
            t = edge.get("target")
            if s in indeg and t in indeg:
                graph[s].append(t)
                indeg[t] += 1

        q = deque([n for n, deg in indeg.items() if deg == 0])
        visited = 0
        while q:
            u = q.popleft()
            visited += 1
            for v in graph[u]:
                indeg[v] -= 1
                if indeg[v] == 0:
                    q.append(v)
        return visited != len(node_ids)

    def _is_connected_undirected(self, node_ids: List[str], edges: List[Dict]) -> bool:
        if not node_ids:
            return True
        graph = defaultdict(set)
        for edge in edges:
            s = edge.get("source")
            t = edge.get("target")
            if s in node_ids and t in node_ids:
                graph[s].add(t)
                graph[t].add(s)

        start = node_ids[0]
        seen = set([start])
        q = deque([start])
        while q:
            u = q.popleft()
            for v in graph[u]:
                if v not in seen:
                    seen.add(v)
                    q.append(v)
        return len(seen) == len(node_ids)

    def _build_node_pad_net_map(self, micro_detail: Dict) -> Dict[str, Dict[str, str]]:
        result: Dict[str, Dict[str, str]] = {}
        for node in micro_detail.get("nodes", []):
            node_id = node.get("id")
            pads = node.get("pads", {})
            if not node_id or not isinstance(pads, dict):
                continue
            result[node_id] = {
                pad_id: str(pad.get("net", ""))
                for pad_id, pad in pads.items()
                if isinstance(pad, dict)
            }
        return result

    def _parse_level1_endpoint(self, endpoint: str):
        # M1_Comp_2.Pad_3(GND)
        if "." not in endpoint or "(" not in endpoint or not endpoint.endswith(")"):
            return None, None, None
        comp_id, right = endpoint.split(".", 1)
        pad_id, net_with_end = right.split("(", 1)
        net_label = net_with_end[:-1]
        return comp_id, pad_id, net_label

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

    def _is_python_float(self, value: Any) -> bool:
        return type(value) is float

    def _effective_dimensions(self, node: Dict) -> Tuple[float, float]:
        w = float(node.get("w", 1.0))
        h = float(node.get("h", 1.0))
        rot = float(node.get("rot", 0.0))
        if rot in [90.0, 270.0, -90.0, -270.0]:
            return h, w
        return w, h

    def _channel_width_for_net(self, net_id: str) -> float:
        name = (net_id or "").upper()
        medium_tokens = ["VCC_5V"]
        large_tokens = [
            "GND", "VCC_12V", "VCC_48V", "/EL+", "/EL-", "IND", "L", "MOSFET", "TO-252"
        ]
        if any(tok in name for tok in large_tokens):
            return self.large_width
        if any(tok in name for tok in medium_tokens):
            return self.medium_width
        return self.small_width

    def _build_level1_component_net_map(self, detail: Dict) -> Dict[str, Set[str]]:
        result: Dict[str, Set[str]] = {}
        for node in detail.get("nodes", []):
            comp_id = node.get("id")
            if not comp_id:
                continue
            net_set: Set[str] = set()
            pads = node.get("pads", {})
            if isinstance(pads, dict):
                for pad in pads.values():
                    if isinstance(pad, dict):
                        net_id = str(pad.get("net", ""))
                        if net_id:
                            net_set.add(net_id)
            result[comp_id] = net_set
        return result

    def _build_level0_micro_net_map(self, level_1: Dict) -> Dict[str, Set[str]]:
        result: Dict[str, Set[str]] = {}
        for micro_id, detail in level_1.items():
            net_set: Set[str] = set()
            for node in detail.get("nodes", []):
                pads = node.get("pads", {})
                if not isinstance(pads, dict):
                    continue
                for pad in pads.values():
                    if isinstance(pad, dict):
                        net_id = str(pad.get("net", ""))
                        if net_id:
                            net_set.add(net_id)
            result[micro_id] = net_set
        return result