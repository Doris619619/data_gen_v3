"""
生成器配置
"""

from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class GeneratorConfig:
    """数据生成器配置"""
    
    # 画布设置
    #canvas_width: float = 40.0
    #canvas_height: float = 40.0
    
    canvas_width: float = 20.0
    canvas_height: float = 20.0
    # Micro 分区设置
    num_micros_range: Tuple[int, int] = (2, 3) # Micro 数量范围
    micro_margin: float = 0.3  # Micro 之间的最小间距
    micro_padding: float = 2  # Micro 内部边距
    
    # 元件数量分布（每个 Micro）
    # components_per_micro_range: Tuple[int, int] = (60, 90)

    #components_per_micro_range: Tuple[int, int] = (10, 20)
    components_per_micro_range: Tuple[int, int] = (3, 5)
    # 密度分布
    density_range: Tuple[float, float] = (0.5, 0.6)
    
    # 元件尺寸分布
    size_scale: float = 0.1
    size_max: float = 0.8
    size_min: float = 0.02

    # 元件间距
    component_spacing: float = 1.0  # 元件（含焊盘）之间的最小间距
    
    # 边分布参数
    scale_parameter: float = 0.2
    edge_max_prob: float = 0.5
    gamma: float = 0.15
    
    # 跨 Micro 连接参数
    cross_micro_connection_prob: float = 0.3  # 跨 Micro 连接的基础概率
    cross_micro_gamma: float = 0.05  # 跨 Micro 连接的 gamma（更低，因为距离更远）
    min_cross_micro_edges: int = 2  # 每对 Micro 之间的最小边数
    
    # Micro 内部连通性参数
    min_edges_per_component: int = 3  # 每个组件至少有这么多条边（防止孤立组件）
    
    # 网络类型比例
    power_net_ratio: float = 0.15
    ground_net_ratio: float = 0.1
    
    # 封装类型权重
    footprint_weights: Dict[str, float] = field(default_factory=lambda: {
        "0603_R": 0.25,
        "0603_C": 0.25,
        "0805_R": 0.1,
        "1206_C": 0.05,
        "SOT23-3": 0.15,
        "SOT89-3": 0.05,
        "SOIC-8": 0.1,
        "SMA": 0.03,
        "CONN_2P": 0.02
    })


# 预设配置，默认使用default
CONFIG_DEFAULT = GeneratorConfig()

CONFIG_SPARSE = GeneratorConfig(
    num_micros_range=(2, 4),
    components_per_micro_range=(15, 25),
    density_range=(0.25, 0.4),
    scale_parameter=0.25,
    gamma=0.12,
    edge_max_prob=0.4,
    cross_micro_connection_prob=0.2,
    cross_micro_gamma=0.03,
    min_edges_per_component=1
)

CONFIG_DENSE = GeneratorConfig(
    num_micros_range=(3, 6),
    components_per_micro_range=(20, 40),
    density_range=(0.4, 0.6),
    scale_parameter=0.15,
    gamma=0.2,
    edge_max_prob=0.6,
    cross_micro_connection_prob=0.4,
    cross_micro_gamma=0.08,
    min_edges_per_component=2  # 密集配置可以要求更多连接
)