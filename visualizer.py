"""
放置结果和布线结果可视化工具
"""

from typing import List, Dict, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.collections as mc
import numpy as np


class PlacementVisualizer:
    """放置结果可视化（支持Micro分区显示）"""
    
    def __init__(self, figsize=(14, 12)):
        self.figsize = figsize
        
        # 颜色映射
        self.net_colors = {
            "GND": "#2E7D32",
            "VCC_5V": "#C62828",
            "VCC_3V3": "#F57C00",
            "VCC_12V": "#6A1B9A",
        }
        self.default_color = "#1976D2"
        
        # Micro分区颜色（柔和的背景色）
        self.micro_colors = [
            "#FFCDD2", "#C8E6C9", "#BBDEFB", "#FFF9C4",
            "#E1BEE7", "#B2DFDB", "#FFE0B2", "#D7CCC8",
        ]
        
        # Micro边框颜色（深色边框）
        self.micro_border_colors = [
            "#C62828", "#2E7D32", "#1565C0", "#F9A825",
            "#7B1FA2", "#00695C", "#EF6C00", "#5D4037",
        ]
        
        # 布线层颜色
        self.layer_colors = [
            "#E53935",  # Layer 0 - 红色 (Top)
            "#1E88E5",  # Layer 1 - 蓝色 (Bottom)
            "#43A047",  # Layer 2 - 绿色
            "#FB8C00",  # Layer 3 - 橙色
        ]
    
    def visualize_placement(self, circuit_data: Dict, 
                           show_nets: bool = True,
                           show_micros: bool = True,
                           show_pad_shape: bool = True,
                           ucg_data: Dict = None,
                           micro_partition: Dict = None,
                           micro_bboxes: Dict = None,
                           save_path: str = None):
        """
        可视化放置结果
        
        Args:
            circuit_data: 电路数据
            show_nets: 是否显示网络连接
            show_micros: 是否显示Micro分区
            show_pad_shape: 是否显示焊盘实际形状（True）或简化为圆点（False）
            ucg_data: UCG数据（可选）
            micro_partition: Micro分区信息（可选）
            micro_bboxes: Micro边界框（可选）
            save_path: 保存路径（可选）
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        placement = circuit_data["placement"]
        netlist = circuit_data["netlist"]
        
        # 绘制画布边界
        canvas_w = placement["canvas"]["width"]
        canvas_h = placement["canvas"]["height"]
        ax.set_xlim(-10, canvas_w + 10)
        ax.set_ylim(-10, canvas_h + 10)
        ax.set_aspect('equal')
        
        # 构建元件查找表
        comp_lookup = {c["id"]: c for c in placement["components"]}
        
        # 获取或生成Micro分区
        if micro_partition is None and ucg_data is not None:
            micro_partition = self._extract_partition_from_ucg(ucg_data)
        elif micro_partition is None:
            micro_partition = self._auto_partition(placement)
        
        # 尝试从 circuit_data 获取预分配的 bbox
        if micro_bboxes is None:
            micro_info = circuit_data.get("metadata", {}).get("micro_info", {})
            if micro_info:
                micro_bboxes = {mid: info["bbox"] for mid, info in micro_info.items()}
        
        # 绘制Micro分区
        micro_legend_handles = []
        if show_micros and micro_partition:
            micro_legend_handles = self._draw_micro_regions(
                ax, micro_partition, comp_lookup, micro_bboxes
            )
        
        # 绘制元件
        for comp in placement["components"]:
            self._draw_component(ax, comp, micro_partition, show_pad_shape=show_pad_shape)
        
        # 绘制连接
        if show_nets:
            self._draw_connections(ax, placement, netlist)
        
        # 设置标签和标题
        ax.set_xlabel("X (mm)", fontsize=12)
        ax.set_ylabel("Y (mm)", fontsize=12)
        
        title = f"PCB Placement: {placement['circuit_id']}\n"
        title += f"Components: {len(placement['components'])}, "
        title += f"Nets: {len(netlist['nets'])}"
        if micro_partition:
            title += f", Micros: {len(micro_partition)}"
        ax.set_title(title, fontsize=14)
        
        # 添加图例
        if micro_legend_handles:
            ax.legend(handles=micro_legend_handles, loc='upper right', 
                     fontsize=9, title="Micro Regions")
        
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"    Saved visualization to {save_path}")
        
        return fig, ax

    def visualize_routing(self, circuit_data: Dict, routing_data: Dict,
                        show_components: bool = True,
                        show_vias: bool = True,
                        show_pad_shape: bool = True,
                        layer_filter: List[int] = None,
                        save_path: str = None):
        """
        可视化布线结果（新格式适配）
        
        新的 routing_data 格式:
        {
            "circuit_id": str,
            "segments": [{start, end, layer, width}, ...],
            "vias": [{x, y, layers, diameter}, ...],
            "statistics": {...}
        }
        """
        # 获取层数（从统计信息推断或默认为2）
        num_layers = 2
        if routing_data.get("segments"):
            max_layer = max(seg["layer"] for seg in routing_data["segments"])
            num_layers = max(num_layers, max_layer + 1)
        
        if layer_filter is None:
            layer_filter = list(range(num_layers))
        
        # 创建多层视图
        if len(layer_filter) == 1:
            fig, axes = plt.subplots(1, 1, figsize=self.figsize)
            axes = [axes]
        else:
            cols = min(len(layer_filter), 2)
            rows = (len(layer_filter) + cols - 1) // cols
            fig, axes = plt.subplots(rows, cols, figsize=(self.figsize[0], self.figsize[1] * rows / 2))
            if rows == 1:
                axes = [axes] if cols == 1 else list(axes)
            else:
                axes = [ax for row in axes for ax in (row if hasattr(row, '__iter__') else [row])]
        
        # 从 circuit_data 获取画布尺寸
        placement = circuit_data["placement"]
        canvas_w = placement["canvas"]["width"]
        canvas_h = placement["canvas"]["height"]
        
        for idx, layer in enumerate(layer_filter):
            if idx >= len(axes):
                break
                
            ax = axes[idx]
            ax.set_xlim(-5, canvas_w + 5)
            ax.set_ylim(-5, canvas_h + 5)
            ax.set_aspect('equal')
            
            # 绘制元件（灰色背景）
            if show_components:
                self._draw_components_background(ax, placement, show_pad_shape=show_pad_shape)
            
            # 绘制该层的布线
            self._draw_routing_layer(ax, routing_data, layer)
            
            # 绘制过孔
            if show_vias:
                self._draw_vias(ax, routing_data, layer)
            
            layer_name = "Top" if layer == 0 else ("Bottom" if layer == 1 else f"Layer {layer}")
            ax.set_title(f"{layer_name} Layer", fontsize=12, fontweight='bold',
                        color=self.layer_colors[layer % len(self.layer_colors)])
            ax.set_xlabel("X (mm)", fontsize=10)
            ax.set_ylabel("Y (mm)", fontsize=10)
            ax.grid(True, alpha=0.2)
        
        # 隐藏多余的子图
        for idx in range(len(layer_filter), len(axes)):
            axes[idx].set_visible(False)
        
        # 总标题
        stats = routing_data.get("statistics", {})
        circuit_id = routing_data.get("circuit_id", "unknown")
        success_count = stats.get('success_count', 0)
        total_edges = stats.get('total_edges', 0)
        total_length = stats.get('total_length', 0)
        total_vias = stats.get('total_vias', 0)
        
        fig.suptitle(
            f"PCB Routing: {circuit_id}\n"
            f"Routes: {success_count}/{total_edges}, "
            f"Total Length: {total_length:.1f}mm, "
            f"Vias: {total_vias}",
            fontsize=14, fontweight='bold'
        )
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"    Saved routing visualization to {save_path}")
        
        return fig, axes

    def visualize_routing_combined(self, circuit_data: Dict, routing_data: Dict,
                                show_components: bool = True,
                                show_vias: bool = True,
                                show_pad_shape: bool = True,
                                save_path: str = None):
        """
        在单一视图中显示所有层的布线（重叠显示）- 新格式适配
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # 从 circuit_data 获取画布尺寸
        placement = circuit_data["placement"]
        canvas_w = placement["canvas"]["width"]
        canvas_h = placement["canvas"]["height"]
        
        ax.set_xlim(-5, canvas_w + 5)
        ax.set_ylim(-5, canvas_h + 5)
        ax.set_aspect('equal')
        
        # 绘制元件
        if show_components:
            for comp in placement["components"]:
                self._draw_component_simple(ax, comp, show_pad_shape=show_pad_shape)
        
        # 获取层数
        num_layers = 2
        if routing_data.get("segments"):
            max_layer = max(seg["layer"] for seg in routing_data["segments"])
            num_layers = max(num_layers, max_layer + 1)
        
        legend_handles = []
        
        for layer in range(num_layers):
            color = self.layer_colors[layer % len(self.layer_colors)]
            self._draw_routing_layer(ax, routing_data, layer, alpha=0.7)
            
            layer_name = "Top" if layer == 0 else ("Bottom" if layer == 1 else f"Layer {layer}")
            legend_handles.append(patches.Patch(color=color, label=f"{layer_name} Layer"))
        
        # 绘制过孔
        if show_vias:
            self._draw_all_vias(ax, routing_data)
            legend_handles.append(patches.Patch(
                facecolor='gold', edgecolor='black', label='Via'
            ))
        
        # 标题和图例
        stats = routing_data.get("statistics", {})
        circuit_id = routing_data.get("circuit_id", "unknown")
        success_count = stats.get('success_count', 0)
        total_edges = stats.get('total_edges', 0)
        total_length = stats.get('total_length', 0)
        total_vias = stats.get('total_vias', 0)
        
        ax.set_title(
            f"PCB Routing: {circuit_id}\n"
            f"Routes: {success_count}/{total_edges}, "
            f"Length: {total_length:.1f}mm, Vias: {total_vias}",
            fontsize=14
        )
        ax.set_xlabel("X (mm)", fontsize=12)
        ax.set_ylabel("Y (mm)", fontsize=12)
        
        ax.legend(handles=legend_handles, loc='upper right', fontsize=9)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"    Saved combined routing visualization to {save_path}")
        
        return fig, ax

    def _draw_routing_layer(self, ax, routing_data: Dict, layer: int, alpha: float = 0.8):
        """绘制单层布线 - 新格式适配"""
        color = self.layer_colors[layer % len(self.layer_colors)]
        
        # 新格式：直接从 segments 列表读取
        for seg in routing_data.get("segments", []):
            if seg["layer"] == layer:
                start = seg["start"]
                end = seg["end"]
                width = seg.get("width", 0.2)
                
                ax.plot(
                    [start[0], end[0]], 
                    [start[1], end[1]],
                    color=color,
                    linewidth=max(width * 3, 1.5),
                    alpha=alpha,
                    solid_capstyle='round',
                    zorder=5
                )

    def _draw_vias(self, ax, routing_data: Dict, layer: int):
        """绘制经过指定层的过孔 - 新格式适配"""
        # 从统计信息或默认值获取过孔尺寸
        stats = routing_data.get("statistics", {})
        via_diameter = 0.6  # 默认值
        via_drill = 0.3     # 默认值
        
        for via in routing_data.get("vias", []):
            # 检查过孔是否经过此层
            layers = via.get("layers", [0, 1])
            if layer < min(layers) or layer > max(layers):
                continue
            
            pos_x = via["x"]
            pos_y = via["y"]
            diameter = via.get("diameter", via_diameter)
            radius = diameter / 2
            
            # 外圈
            outer = patches.Circle(
                (pos_x, pos_y), radius,
                facecolor='gold',
                edgecolor='black',
                linewidth=1,
                zorder=10
            )
            ax.add_patch(outer)
            
            # 内圈（钻孔）
            drill_radius = via_drill / 2
            inner = patches.Circle(
                (pos_x, pos_y), drill_radius,
                facecolor='white',
                edgecolor='black',
                linewidth=0.5,
                zorder=11
            )
            ax.add_patch(inner)

    def _draw_all_vias(self, ax, routing_data: Dict):
        """绘制所有过孔 - 新格式适配"""
        via_diameter = 0.6  # 默认值
        via_drill = 0.3     # 默认值
        
        for via in routing_data.get("vias", []):
            pos_x = via["x"]
            pos_y = via["y"]
            diameter = via.get("diameter", via_diameter)
            radius = diameter / 2
            
            outer = patches.Circle(
                (pos_x, pos_y), radius,
                facecolor='gold',
                edgecolor='black',
                linewidth=1,
                zorder=10
            )
            ax.add_patch(outer)
            
            drill_radius = via_drill / 2
            inner = patches.Circle(
                (pos_x, pos_y), drill_radius,
                facecolor='white',
                edgecolor='black',
                linewidth=0.5,
                zorder=11
            )
            ax.add_patch(inner)

    def _draw_components_background(self, ax, placement: Dict, show_pad_shape: bool = True):
        """绘制元件作为背景（灰色）"""
        for comp in placement["components"]:
            x, y = comp["position"]
            w, h = comp["size"]
            rot = comp["rotation"]
            
            if rot in [90, 270, -90]:
                w, h = h, w
            
            rect = patches.Rectangle(
                (x - w/2, y - h/2), w, h,
                linewidth=1,
                edgecolor='#666666',
                facecolor='#EEEEEE',
                alpha=0.5,
                zorder=1
            )
            ax.add_patch(rect)
            
            # 绘制焊盘
            for pin in comp["pins"]:
                self._draw_pad(ax, comp, pin, color='#999999', 
                              show_shape=show_pad_shape, alpha=0.6, zorder=2)
            
            # 简化标签
            label = comp["id"].split("_")[-1]
            ax.text(x, y, label, ha='center', va='center', 
                   fontsize=6, color='#666666', zorder=2)
    
    def _draw_component_simple(self, ax, comp: Dict, show_pad_shape: bool = True):
        """简化的元件绘制"""
        x, y = comp["position"]
        w, h = comp["size"]
        rot = comp["rotation"]
        
        if rot in [90, 270, -90]:
            w, h = h, w
        
        rect = patches.Rectangle(
            (x - w/2, y - h/2), w, h,
            linewidth=1.5,
            edgecolor='#333333',
            facecolor='#FFFFFF',
            alpha=0.9,
            zorder=3
        )
        ax.add_patch(rect)
        
        # 绘制焊盘
        for pin in comp["pins"]:
            color = self.net_colors.get(pin["net"], "#666666")
            self._draw_pad(ax, comp, pin, color=color, 
                          show_shape=show_pad_shape, zorder=4)
        
        label = comp["id"].split("_")[-1]
        ax.text(x, y, label, ha='center', va='center', 
               fontsize=6, color='#333', fontweight='bold', zorder=5)
    
    def _draw_pad(self, ax, comp: Dict, pin: Dict, color: str,
                  show_shape: bool = True, alpha: float = 1.0, zorder: int = 3):
        """
        绘制单个焊盘
        
        Args:
            ax: matplotlib axes
            comp: 元件数据
            pin: 引脚数据
            color: 焊盘颜色
            show_shape: True显示实际矩形形状，False显示为圆点
            alpha: 透明度
            zorder: 绘制层级
        """
        x, y = comp["position"]
        rot = comp["rotation"]
        
        # 计算焊盘中心绝对位置
        pin_rel_x, pin_rel_y = pin["rel_pos"]
        angle_rad = np.radians(rot)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        
        rx = pin_rel_x * cos_a - pin_rel_y * sin_a
        ry = pin_rel_x * sin_a + pin_rel_y * cos_a
        
        abs_x = x + rx
        abs_y = y + ry
        
        if show_shape:
            # 绘制实际形状的焊盘（矩形）
            pad_w, pad_h = pin["size"]
            
            # 焊盘也需要随元件旋转
            if rot in [90, 270, -90, -270]:
                pad_w, pad_h = pad_h, pad_w
            
            # 创建旋转的矩形
            # 使用 FancyBboxPatch 以获得更好的视觉效果
            pad_rect = patches.FancyBboxPatch(
                (abs_x - pad_w/2, abs_y - pad_h/2),
                pad_w, pad_h,
                boxstyle=patches.BoxStyle("Round", pad=0, rounding_size=min(pad_w, pad_h) * 0.1),
                linewidth=0.5,
                edgecolor='white',
                facecolor=color,
                alpha=alpha,
                zorder=zorder
            )
            ax.add_patch(pad_rect)
        else:
            # 简化为圆点
            radius = max(pin["size"][0], pin["size"][1]) / 2 * 0.8
            pin_patch = patches.Circle(
                (abs_x, abs_y), 
                radius=max(radius, 0.3),
                facecolor=color,
                edgecolor='white',
                linewidth=0.5,
                alpha=alpha,
                zorder=zorder
            )
            ax.add_patch(pin_patch)
    
    def _draw_micro_regions(self, ax, micro_partition: Dict, 
                           comp_lookup: Dict,
                           micro_bboxes: Dict = None) -> List:
        """绘制Micro分区区域"""
        legend_handles = []
        
        for idx, (micro_id, comp_ids) in enumerate(micro_partition.items()):
            if not comp_ids:
                continue
            
            bg_color = self.micro_colors[idx % len(self.micro_colors)]
            border_color = self.micro_border_colors[idx % len(self.micro_border_colors)]
            
            if micro_bboxes and micro_id in micro_bboxes:
                bbox_raw = micro_bboxes[micro_id]
                bbox = {
                    "min_x": bbox_raw[0],
                    "min_y": bbox_raw[1],
                    "max_x": bbox_raw[2],
                    "max_y": bbox_raw[3],
                    "width": bbox_raw[2] - bbox_raw[0],
                    "height": bbox_raw[3] - bbox_raw[1]
                }
            else:
                comps = [comp_lookup[cid] for cid in comp_ids if cid in comp_lookup]
                if not comps:
                    continue
                bbox = self._calculate_bounding_box(comps, padding=5.0)
            
            rect = patches.FancyBboxPatch(
                (bbox["min_x"], bbox["min_y"]),
                bbox["width"], bbox["height"],
                boxstyle=patches.BoxStyle("Round", pad=0, rounding_size=2),
                linewidth=2.5,
                edgecolor=border_color,
                facecolor=bg_color,
                alpha=0.4,
                zorder=0
            )
            ax.add_patch(rect)
            
            label_x = bbox["min_x"] + bbox["width"] / 2
            label_y = bbox["max_y"] + 3
            ax.text(label_x, label_y, micro_id, 
                   ha='center', va='bottom', 
                   fontsize=11, fontweight='bold',
                   color=border_color,
                   bbox=dict(boxstyle='round,pad=0.3', 
                            facecolor='white', 
                            edgecolor=border_color,
                            alpha=0.9))
            
            legend_label = f"{micro_id} ({len(comp_ids)} comps, {bbox['width']:.1f}x{bbox['height']:.1f})"
            legend_patch = patches.Patch(
                facecolor=bg_color, 
                edgecolor=border_color,
                linewidth=2,
                label=legend_label
            )
            legend_handles.append(legend_patch)
        
        return legend_handles
    
    def _calculate_bounding_box(self, components: List[Dict], 
                                padding: float = 2.0) -> Dict:
        """
        计算元件组的包围盒（考虑焊盘尺寸）
        """
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        
        for comp in components:
            x, y = comp["position"]
            w, h = comp["size"]
            rot = comp["rotation"]
            
            if rot in [90, 270, -90]:
                w, h = h, w
            
            # 元件本体边界
            comp_min_x = x - w/2
            comp_max_x = x + w/2
            comp_min_y = y - h/2
            comp_max_y = y + h/2
            
            # 扩展以包含焊盘
            for pin in comp["pins"]:
                pin_rel_x, pin_rel_y = pin["rel_pos"]
                pad_w, pad_h = pin["size"]
                
                # 计算焊盘绝对位置
                angle_rad = np.radians(rot)
                cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
                rx = pin_rel_x * cos_a - pin_rel_y * sin_a
                ry = pin_rel_x * sin_a + pin_rel_y * cos_a
                
                pad_x = x + rx
                pad_y = y + ry
                
                # 焊盘尺寸也需要考虑旋转
                if rot in [90, 270, -90, -270]:
                    pad_w, pad_h = pad_h, pad_w
                
                comp_min_x = min(comp_min_x, pad_x - pad_w/2)
                comp_max_x = max(comp_max_x, pad_x + pad_w/2)
                comp_min_y = min(comp_min_y, pad_y - pad_h/2)
                comp_max_y = max(comp_max_y, pad_y + pad_h/2)
            
            min_x = min(min_x, comp_min_x)
            max_x = max(max_x, comp_max_x)
            min_y = min(min_y, comp_min_y)
            max_y = max(max_y, comp_max_y)
        
        # 添加 padding
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding
        
        return {
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "width": max_x - min_x,
            "height": max_y - min_y,
            "center": [(min_x + max_x) / 2, (min_y + max_y) / 2]
        }
    
    def _draw_component(self, ax, comp: Dict, micro_partition: Dict = None,
                       show_pad_shape: bool = True):
        """绘制单个元件"""
        x, y = comp["position"]
        w, h = comp["size"]
        rot = comp["rotation"]
        
        if rot in [90, 270, -90]:
            w, h = h, w
        
        border_color = '#333333'
        if micro_partition:
            for idx, (micro_id, comp_ids) in enumerate(micro_partition.items()):
                if comp["id"] in comp_ids:
                    border_color = self.micro_border_colors[idx % len(self.micro_border_colors)]
                    break
        
        rect = patches.Rectangle(
            (x - w/2, y - h/2), w, h,
            linewidth=1.5,
            edgecolor=border_color,
            facecolor='#FFFFFF',
            alpha=0.95,
            zorder=2
        )
        ax.add_patch(rect)
        
        # 绘制焊盘
        for pin in comp["pins"]:
            color = self.net_colors.get(pin["net"], self.default_color)
            self._draw_pad(ax, comp, pin, color=color, 
                          show_shape=show_pad_shape, zorder=3)
        
        ax.text(x, y, comp["id"].split("_")[-1], 
               ha='center', va='center', fontsize=7, 
               color='#333', fontweight='bold', zorder=4)
    
    def _draw_connections(self, ax, placement: Dict, netlist: Dict):
        """绘制网络连接"""
        comp_positions = {}
        for comp in placement["components"]:
            comp_positions[comp["id"]] = {
                "position": comp["position"],
                "rotation": comp["rotation"],
                "pins": {p["pin_id"]: p for p in comp["pins"]}
            }
        
        for edge in netlist["edges"]:
            src_comp = edge["source"]["component"]
            src_pin = edge["source"]["pin"]
            tgt_comp = edge["target"]["component"]
            tgt_pin = edge["target"]["pin"]
            
            if src_comp not in comp_positions or tgt_comp not in comp_positions:
                continue
            
            src_pos = self._get_pin_position(comp_positions[src_comp], src_pin)
            tgt_pos = self._get_pin_position(comp_positions[tgt_comp], tgt_pin)
            
            if src_pos and tgt_pos:
                color = self.net_colors.get(edge["net"], "#90A4AE")
                ax.plot([src_pos[0], tgt_pos[0]], [src_pos[1], tgt_pos[1]],
                       color=color, alpha=0.5, linewidth=0.8, zorder=1)
    
    def _get_pin_position(self, comp_data: Dict, pin_id: str) -> List[float]:
        """获取引脚绝对位置"""
        if pin_id not in comp_data["pins"]:
            return None
        
        pin = comp_data["pins"][pin_id]
        pos = comp_data["position"]
        rot = comp_data["rotation"]
        
        angle_rad = np.radians(rot)
        rx = pin["rel_pos"][0] * np.cos(angle_rad) - pin["rel_pos"][1] * np.sin(angle_rad)
        ry = pin["rel_pos"][0] * np.sin(angle_rad) + pin["rel_pos"][1] * np.cos(angle_rad)
        
        return [pos[0] + rx, pos[1] + ry]
    
    def _extract_partition_from_ucg(self, ucg_data: Dict) -> Dict:
        """从UCG数据提取Micro分区"""
        partition = {}
        level1 = ucg_data.get("UCG_Graph", {}).get("level_1_details", {})
        for micro_id, details in level1.items():
            partition[micro_id] = [node["id"] for node in details.get("nodes", [])]
        return partition
    
    def _auto_partition(self, placement: Dict) -> Dict:
        """自动生成Micro分区"""
        partition = {}
        for comp in placement["components"]:
            micro_id = comp.get("micro_id", "Micro_0")
            if micro_id not in partition:
                partition[micro_id] = []
            partition[micro_id].append(comp["id"])
        return partition
    
    def visualize_ucg_structure(self, ucg_data: Dict, circuit_data: Dict = None,
                                save_path: str = None):
        """可视化UCG结构（双层视图）"""
        fig, axes = plt.subplots(1, 2, figsize=(18, 8))
        
        self._draw_level0_graph(axes[0], ucg_data)
        self._draw_level1_graph(axes[1], ucg_data, circuit_data)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved UCG visualization to {save_path}")
        
        return fig, axes
    
    def _draw_level0_graph(self, ax, ucg_data: Dict):
        """绘制Level 0全局图"""
        ax.set_title("Level 0: Global Micro Topology", fontsize=14, fontweight='bold')
        
        level0 = ucg_data["UCG_Graph"]["level_0_global"]
        nodes = level0["nodes"]
        spatial_edges = level0["spatial_topology"]
        routing = level0["routing_resources"]
        
        if not nodes:
            ax.text(0.5, 0.5, "No Micro nodes", ha='center', va='center', 
                   transform=ax.transAxes, fontsize=12)
            return
        
        num_nodes = len(nodes)
        cols = int(np.ceil(np.sqrt(num_nodes)))
        
        node_positions = {}
        for i, node in enumerate(nodes):
            row = i // cols
            col = i % cols
            x = col * 60 + 30
            y = row * 60 + 30
            node_positions[node["id"]] = (x, y)
        
        for edge in spatial_edges:
            src_pos = node_positions.get(edge["source"])
            tgt_pos = node_positions.get(edge["target"])
            if src_pos and tgt_pos:
                ax.annotate("", xy=tgt_pos, xytext=src_pos,
                           arrowprops=dict(arrowstyle="->", color="#666666",
                                         lw=1.5, alpha=0.6))
        
        for i, node in enumerate(nodes):
            pos = node_positions[node["id"]]
            color = self.micro_colors[i % len(self.micro_colors)]
            border = self.micro_border_colors[i % len(self.micro_border_colors)]
            
            rect = patches.FancyBboxPatch(
                (pos[0] - node["w"]/2, pos[1] - node["h"]/2),
                node["w"], node["h"],
                boxstyle=patches.BoxStyle("Round", pad=0, rounding_size=2),
                linewidth=2,
                edgecolor=border,
                facecolor=color,
                alpha=0.8
            )
            ax.add_patch(rect)
            
            ax.text(pos[0], pos[1], f"{node['id']}\n({node['w']:.1f}x{node['h']:.1f})",
                   ha='center', va='center', fontsize=9, fontweight='bold')
        
        routing_text = f"Cross-Micro Routes: {len(routing)}"
        ax.text(0.02, 0.98, routing_text, transform=ax.transAxes,
               fontsize=10, va='top', 
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.set_xlim(-10, cols * 60 + 40)
        ax.set_ylim(-10, (num_nodes // cols + 1) * 60 + 20)
        ax.set_aspect('equal')
        ax.axis('off')
    
    def _draw_level1_graph(self, ax, ucg_data: Dict, circuit_data: Dict = None):
        """绘制Level 1详细图"""
        ax.set_title("Level 1: Micro Internal Details", fontsize=14, fontweight='bold')
        
        level1 = ucg_data["UCG_Graph"]["level_1_details"]
        
        if not level1:
            ax.text(0.5, 0.5, "No Level 1 details", ha='center', va='center',
                   transform=ax.transAxes, fontsize=12)
            return
        
        num_micros = len(level1)
        cols = min(num_micros, 3)
        rows = int(np.ceil(num_micros / cols))
        
        region_width = 80
        region_height = 60
        
        for idx, (micro_id, details) in enumerate(level1.items()):
            row = idx // cols
            col = idx % cols
            
            offset_x = col * region_width
            offset_y = row * region_height
            
            color = self.micro_colors[idx % len(self.micro_colors)]
            border = self.micro_border_colors[idx % len(self.micro_border_colors)]
            
            rect = patches.Rectangle(
                (offset_x + 2, offset_y + 2),
                region_width - 4, region_height - 4,
                linewidth=2,
                edgecolor=border,
                facecolor=color,
                alpha=0.3
            )
            ax.add_patch(rect)
            
            ax.text(offset_x + region_width/2, offset_y + region_height - 5,
                   f"{micro_id}", ha='center', va='top',
                   fontsize=11, fontweight='bold', color=border)
            
            nodes = details.get("nodes", [])
            num_nodes = len(nodes)
            
            if num_nodes > 0:
                node_cols = int(np.ceil(np.sqrt(num_nodes)))
                node_spacing_x = (region_width - 20) / max(node_cols, 1)
                node_spacing_y = (region_height - 25) / max((num_nodes // node_cols + 1), 1)
                
                for i, node in enumerate(nodes):
                    nx = offset_x + 10 + (i % node_cols) * node_spacing_x
                    ny = offset_y + 10 + (i // node_cols) * node_spacing_y
                    
                    node_rect = patches.Rectangle(
                        (nx, ny), 
                        min(node["w"] * 0.8, node_spacing_x - 2),
                        min(node["h"] * 0.8, node_spacing_y - 5),
                        linewidth=1,
                        edgecolor='#333',
                        facecolor='white',
                        alpha=0.9
                    )
                    ax.add_patch(node_rect)
                    
                    label = node["id"].split("_")[-1] if "_" in node["id"] else node["id"]
                    ax.text(nx + min(node["w"] * 0.4, node_spacing_x/2 - 1),
                           ny + min(node["h"] * 0.4, node_spacing_y/2 - 2.5),
                           label, ha='center', va='center', fontsize=6)
            
            num_spatial = len(details.get("spatial_topology", []))
            num_routing = len(details.get("routing_resources", []))
            stats_text = f"N:{num_nodes} S:{num_spatial} R:{num_routing}"
            ax.text(offset_x + region_width/2, offset_y + 5,
                   stats_text, ha='center', va='bottom', fontsize=7, color='#666')
        
        ax.set_xlim(-5, cols * region_width + 5)
        ax.set_ylim(-5, rows * region_height + 5)
        ax.set_aspect('equal')
        ax.axis('off')


# 便捷函数
def visualize_circuit_with_routing(circuit_path: str, routing_path: str = None,
                                   save_path: str = None,
                                   show_pad_shape: bool = True):
    """
    便捷函数：可视化电路放置和布线结果
    
    Args:
        circuit_path: 电路数据文件路径
        routing_path: 布线数据文件路径（可选）
        save_path: 保存路径（可选）
        show_pad_shape: 是否显示焊盘实际形状
    """
    import json
    from pathlib import Path
    
    with open(circuit_path, 'r') as f:
        circuit_data = json.load(f)
    
    vis = PlacementVisualizer()
    
    if routing_path:
        with open(routing_path, 'r') as f:
            routing_data = json.load(f)
        
        vis.visualize_routing_combined(circuit_data, routing_data, 
                                       show_pad_shape=show_pad_shape,
                                       save_path=save_path)
    else:
        vis.visualize_placement(circuit_data, show_pad_shape=show_pad_shape,
                               save_path=save_path)