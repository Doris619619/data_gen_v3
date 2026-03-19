"""
数据集验证和统计工具
"""

from typing import List, Dict
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