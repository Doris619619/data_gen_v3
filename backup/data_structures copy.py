"""
数据结构定义
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple
from enum import Enum


class NetType(Enum):
    """网络类型"""
    POWER = "power"
    GROUND = "ground"
    SIGNAL = "signal"


@dataclass
class Pin:
    """引脚定义"""
    pin_id: str
    rel_pos: List[float]
    size: List[float]
    net: str = ""
    rotation: float = 0.0


@dataclass
class Component:
    """元件定义"""
    id: str
    comp_type: str
    position: List[float]
    size: List[float]
    rotation: float = 0.0
    pins: List[Pin] = field(default_factory=list)
    micro_id: str = ""  # 所属的Micro
    
    @property
    def width(self) -> float:
        if self.rotation in [90.0, 270.0, -90.0]:
            return self.size[1]
        return self.size[0]
    
    @property
    def height(self) -> float:
        if self.rotation in [90.0, 270.0, -90.0]:
            return self.size[0]
        return self.size[1]
    
    def get_bbox(self) -> Tuple[float, float, float, float]:
        """获取边界框 (min_x, min_y, max_x, max_y)"""
        half_w = self.width / 2
        half_h = self.height / 2
        return (
            self.position[0] - half_w,
            self.position[1] - half_h,
            self.position[0] + half_w,
            self.position[1] + half_h
        )
    
    def get_pin_absolute_position(self, pin: Pin) -> List[float]:
        """获取引脚的绝对位置"""
        angle_rad = math.radians(self.rotation)
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        rx = pin.rel_pos[0] * cos_a - pin.rel_pos[1] * sin_a
        ry = pin.rel_pos[0] * sin_a + pin.rel_pos[1] * cos_a
        return [self.position[0] + rx, self.position[1] + ry]


@dataclass
class MicroRegion:
    """Micro 区域定义"""
    id: str
    bbox: Tuple[float, float, float, float]  # (min_x, min_y, max_x, max_y)
    components: List[Component] = field(default_factory=list)
    
    @property
    def width(self) -> float:
        return self.bbox[2] - self.bbox[0]
    
    @property
    def height(self) -> float:
        return self.bbox[3] - self.bbox[1]
    
    @property
    def center(self) -> List[float]:
        return [
            (self.bbox[0] + self.bbox[2]) / 2,
            (self.bbox[1] + self.bbox[3]) / 2
        ]
    
    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass
class NetConnection:
    """网络连接"""
    net_id: str
    net_type: NetType
    pins: List[dict]


@dataclass
class Edge:
    """边（连接）"""
    source_comp: str
    source_pin: str
    target_comp: str
    target_pin: str
    net: str
    distance: float