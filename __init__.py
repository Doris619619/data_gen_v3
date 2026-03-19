"""
PCB Placement Synthetic Data Generator (v3)
"""

from data_structures import (
    NetType, Pin, Component, MicroRegion, NetConnection, Edge
)
from footprint_library import FOOTPRINT_LIBRARY
from config import (
    GeneratorConfig, CONFIG_DEFAULT, CONFIG_SPARSE, CONFIG_DENSE
)
from micro_layout import MicroLayoutPlanner
from generator import PCBPlacementGenerator
from ucg_converter import UCGConverter
from visualizer import PlacementVisualizer
from validator import DatasetValidator, DatasetStatistics, PromptUCGValidator

__all__ = [
    # 数据结构
    'NetType', 'Pin', 'Component', 'MicroRegion', 'NetConnection', 'Edge',
    # 封装库
    'FOOTPRINT_LIBRARY',
    # 配置
    'GeneratorConfig', 'CONFIG_DEFAULT', 'CONFIG_SPARSE', 'CONFIG_DENSE',
    # 核心类
    'MicroLayoutPlanner', 'PCBPlacementGenerator', 'UCGConverter',
    # 工具类
    'PlacementVisualizer', 'DatasetValidator', 'DatasetStatistics', 'PromptUCGValidator',
]