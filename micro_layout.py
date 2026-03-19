"""
Micro 区域布局规划器
"""

import numpy as np
from typing import List, Tuple

from data_structures import MicroRegion


class MicroLayoutPlanner:
    """Micro 区域布局规划器"""
    
    def __init__(self, canvas_width: float, canvas_height: float, 
                 margin: float = 3.0):
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.margin = margin
    
    def plan_micro_regions(self, num_micros: int, 
                          layout_type: str = "auto") -> List[MicroRegion]:
        """
        规划 Micro 区域布局
        
        Args:
            num_micros: Micro 数量
            layout_type: 布局类型 ("grid", "treemap", "random", "auto")
        
        Returns:
            不重叠的 MicroRegion 列表
        """
        if layout_type == "auto":
            if num_micros <= 2:
                layout_type = "horizontal"
            elif num_micros <= 4:
                layout_type = "grid"
            else:
                layout_type = "treemap"
        
        layout_methods = {
            "horizontal": self._horizontal_layout,
            "vertical": self._vertical_layout,
            "grid": self._grid_layout,
            "treemap": self._treemap_layout,
            "random": self._random_layout,
        }
        
        method = layout_methods.get(layout_type, self._grid_layout)
        return method(num_micros)
    
    def _horizontal_layout(self, num_micros: int) -> List[MicroRegion]:
        """水平分割布局"""
        regions = []
        usable_width = self.canvas_width - self.margin * 2
        usable_height = self.canvas_height - self.margin * 2
        
        ratios = np.random.dirichlet(np.ones(num_micros) * 2)
        
        current_x = self.margin
        for i, ratio in enumerate(ratios):
            width = usable_width * ratio - self.margin * (num_micros - 1) / num_micros
            width = max(width, 10.0)
            
            bbox = (
                current_x,
                self.margin,
                current_x + width,
                self.margin + usable_height
            )
            
            regions.append(MicroRegion(id=f"Micro_{i+1}", bbox=bbox))
            current_x += width + self.margin
        
        return regions
    
    def _vertical_layout(self, num_micros: int) -> List[MicroRegion]:
        """垂直分割布局"""
        regions = []
        usable_width = self.canvas_width - self.margin * 2
        usable_height = self.canvas_height - self.margin * 2
        
        ratios = np.random.dirichlet(np.ones(num_micros) * 2)
        
        current_y = self.margin
        for i, ratio in enumerate(ratios):
            height = usable_height * ratio - self.margin * (num_micros - 1) / num_micros
            height = max(height, 10.0)
            
            bbox = (
                self.margin,
                current_y,
                self.margin + usable_width,
                current_y + height
            )
            
            regions.append(MicroRegion(id=f"Micro_{i+1}", bbox=bbox))
            current_y += height + self.margin
        
        return regions
    
    def _grid_layout(self, num_micros: int) -> List[MicroRegion]:
        """网格布局"""
        regions = []
        
        cols = int(np.ceil(np.sqrt(num_micros)))
        rows = int(np.ceil(num_micros / cols))
        
        usable_width = self.canvas_width - self.margin * (cols + 1)
        usable_height = self.canvas_height - self.margin * (rows + 1)
        
        cell_width = usable_width / cols
        cell_height = usable_height / rows
        
        idx = 0
        for row in range(rows):
            for col in range(cols):
                if idx >= num_micros:
                    break
                
                width_var = np.random.uniform(0.8, 1.0) * cell_width
                height_var = np.random.uniform(0.8, 1.0) * cell_height
                
                x_offset = (cell_width - width_var) / 2
                y_offset = (cell_height - height_var) / 2
                
                min_x = self.margin + col * (cell_width + self.margin) + x_offset
                min_y = self.margin + row * (cell_height + self.margin) + y_offset
                
                bbox = (min_x, min_y, min_x + width_var, min_y + height_var)
                
                regions.append(MicroRegion(id=f"Micro_{idx+1}", bbox=bbox))
                idx += 1
        
        return regions
    
    def _treemap_layout(self, num_micros: int) -> List[MicroRegion]:
        """Treemap 布局（递归分割）"""
        weights = np.random.dirichlet(np.ones(num_micros) * 1.5)
        
        initial_bbox = (
            self.margin,
            self.margin,
            self.canvas_width - self.margin,
            self.canvas_height - self.margin
        )
        
        regions = self._squarify(weights, initial_bbox, num_micros)
        
        for i, region in enumerate(regions):
            region.id = f"Micro_{i+1}"
        
        return regions
    
    def _squarify(self, weights: np.ndarray, bbox: Tuple[float, float, float, float],
                  num_regions: int) -> List[MicroRegion]:
        """Squarified Treemap 算法"""
        if num_regions <= 0:
            return []
        if num_regions == 1:
            shrunk_bbox = (
                bbox[0] + self.margin / 2,
                bbox[1] + self.margin / 2,
                bbox[2] - self.margin / 2,
                bbox[3] - self.margin / 2
            )
            return [MicroRegion(id="", bbox=shrunk_bbox)]
        
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        
        if width >= height:
            split_idx = num_regions // 2
            left_weight = weights[:split_idx].sum()
            right_weight = weights[split_idx:].sum()
            total_weight = left_weight + right_weight
            
            split_x = bbox[0] + width * (left_weight / total_weight)
            
            left_bbox = (bbox[0], bbox[1], split_x - self.margin / 2, bbox[3])
            right_bbox = (split_x + self.margin / 2, bbox[1], bbox[2], bbox[3])
            
            left_regions = self._squarify(
                weights[:split_idx] / left_weight if left_weight > 0 else weights[:split_idx],
                left_bbox, split_idx
            )
            right_regions = self._squarify(
                weights[split_idx:] / right_weight if right_weight > 0 else weights[split_idx:],
                right_bbox, num_regions - split_idx
            )
            
            return left_regions + right_regions
        else:
            split_idx = num_regions // 2
            top_weight = weights[:split_idx].sum()
            bottom_weight = weights[split_idx:].sum()
            total_weight = top_weight + bottom_weight
            
            split_y = bbox[1] + height * (top_weight / total_weight)
            
            top_bbox = (bbox[0], bbox[1], bbox[2], split_y - self.margin / 2)
            bottom_bbox = (bbox[0], split_y + self.margin / 2, bbox[2], bbox[3])
            
            top_regions = self._squarify(
                weights[:split_idx] / top_weight if top_weight > 0 else weights[:split_idx],
                top_bbox, split_idx
            )
            bottom_regions = self._squarify(
                weights[split_idx:] / bottom_weight if bottom_weight > 0 else weights[split_idx:],
                bottom_bbox, num_regions - split_idx
            )
            
            return top_regions + bottom_regions
    
    def _random_layout(self, num_micros: int) -> List[MicroRegion]:
        """随机布局（保证不重叠）"""
        regions = []
        max_attempts = 1000
        
        total_area = (self.canvas_width - 2 * self.margin) * (self.canvas_height - 2 * self.margin)
        avg_area = total_area / num_micros * 0.7
        avg_size = np.sqrt(avg_area)
        
        for i in range(num_micros):
            width = np.random.uniform(avg_size * 0.6, avg_size * 1.4)
            height = np.random.uniform(avg_size * 0.6, avg_size * 1.4)
            
            placed = False
            for _ in range(max_attempts):
                min_x = np.random.uniform(self.margin, self.canvas_width - self.margin - width)
                min_y = np.random.uniform(self.margin, self.canvas_height - self.margin - height)
                
                new_bbox = (min_x, min_y, min_x + width, min_y + height)
                
                if not self._check_overlap_with_regions(new_bbox, regions):
                    regions.append(MicroRegion(id=f"Micro_{i+1}", bbox=new_bbox))
                    placed = True
                    break
            
            if not placed:
                print(f"Warning: Random placement failed for Micro_{i+1}, falling back to grid")
                return self._grid_layout(num_micros)
        
        return regions
    
    def _check_overlap_with_regions(self, bbox: Tuple[float, float, float, float],
                                   regions: List[MicroRegion]) -> bool:
        """检查边界框是否与现有区域重叠"""
        for region in regions:
            if self._bboxes_overlap(bbox, region.bbox):
                return True
        return False
    
    def _bboxes_overlap(self, bbox1: Tuple[float, float, float, float],
                        bbox2: Tuple[float, float, float, float]) -> bool:
        """检查两个边界框是否重叠（包含间距）"""
        return not (
            bbox1[2] + self.margin < bbox2[0] or
            bbox2[2] + self.margin < bbox1[0] or
            bbox1[3] + self.margin < bbox2[1] or
            bbox2[3] + self.margin < bbox1[1]
        )