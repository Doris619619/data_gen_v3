import json
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class GridNode:
    """网格图的节点类"""
    def __init__(self, x, y):
        self.x = round(x, 4)
        self.y = round(y, 4)
        self.neighbors = set()

    def add_neighbor(self, node):
        if node != self:
            self.neighbors.add(node)


class GridGraph:
    """管理各个网格边界坐标的图结构类"""
    def __init__(self, micro_id):
        self.micro_id = micro_id
        self.nodes = {}         
        self.raw_segments = []  

    def get_or_create_node(self, x, y):
        coord = (round(x, 4), round(y, 4))
        if coord not in self.nodes:
            self.nodes[coord] = GridNode(*coord)
        return self.nodes[coord]

    def add_raw_segment(self, x1, y1, x2, y2):
        if round(x1, 4) == round(x2, 4) and round(y1, 4) == round(y2, 4):
            return  
        if x1 > x2 or y1 > y2:
            x1, x2, y1, y2 = x2, x1, y2, y1
        self.raw_segments.append((x1, y1, x2, y2))

    def build_graph(self):
        h_lines = [s for s in self.raw_segments if round(s[1], 4) == round(s[3], 4)]
        v_lines = [s for s in self.raw_segments if round(s[0], 4) == round(s[2], 4)]

        points_on_segment = {seg: set() for seg in self.raw_segments}
        for seg in self.raw_segments:
            points_on_segment[seg].add((round(seg[0], 4), round(seg[1], 4)))
            points_on_segment[seg].add((round(seg[2], 4), round(seg[3], 4)))

        for h in h_lines:
            hx1, hy, hx2, _ = h
            for v in v_lines:
                vx, vy1, _, vy2 = v
                if min(hx1, hx2) <= vx <= max(hx1, hx2) and min(vy1, vy2) <= hy <= max(vy1, vy2):
                    ix, iy = round(vx, 4), round(hy, 4)
                    points_on_segment[h].add((ix, iy))
                    points_on_segment[v].add((ix, iy))

        for seg, pts in points_on_segment.items():
            sorted_pts = sorted(list(pts))
            for i in range(len(sorted_pts) - 1):
                p1, p2 = sorted_pts[i], sorted_pts[i+1]
                n1 = self.get_or_create_node(*p1)
                n2 = self.get_or_create_node(*p2)
                n1.add_neighbor(n2)
                n2.add_neighbor(n1)


class PCBGridGenerator:
    def __init__(self, dataset_path):
        with open(dataset_path, 'r', encoding='utf-8') as f:
            self.data = json.load(f)
        self.canvas = self.data['placement']['canvas']
        self.micros = {}
        # 新增：保存原始 netlist 数据
        self.netlist = self.data.get('netlist', {'edges': []})
        # 新增：创建一个快速查找 Pin 绝对坐标的字典，格式: {"Comp_1": {"Pad_1": (x, y)}}
        self.pin_coords = {} 
        self._parse_and_convert()

    def _parse_and_convert(self):
        components = self.data['placement']['components']
        micro_comps = {}

        for comp in components:
            cx, cy = comp['position']
            w, h = comp['size']
            angle_deg = comp.get('rotation', 0.0)
            angle_rad = math.radians(angle_deg)
            cos_a, sin_a = abs(math.cos(angle_rad)), abs(math.sin(angle_rad))
            
            comp_id = comp['id']
            self.pin_coords[comp_id] = {}

            aabb = {
                'id': comp_id,
                'x_min': cx - (w * cos_a + h * sin_a) / 2, 
                'x_max': cx + (w * cos_a + h * sin_a) / 2,
                'y_min': cy - (w * sin_a + h * cos_a) / 2, 
                'y_max': cy + (w * sin_a + h * cos_a) / 2,
                'pins': []
            }
            
            for pin in comp.get('pins', []):
                dx, dy = pin['rel_pos']
                nx = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
                ny = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
                px, py = cx + nx, cy + ny
                
                # 记录绝对坐标，供可视化连线使用
                self.pin_coords[comp_id][pin['pin_id']] = (px, py)
                
                aabb['pins'].append({
                    'id': pin['pin_id'],
                    'abs_pos': (px, py),
                    'size': pin['size']
                })

            m_id = comp['micro_id']
            if m_id not in micro_comps: micro_comps[m_id] = []
            micro_comps[m_id].append(aabb)

        # 计算局部 Micro 边界
        for m_id, comps in micro_comps.items():
            m_x_min = min(c['x_min'] for c in comps)
            m_x_max = max(c['x_max'] for c in comps)
            m_y_min = min(c['y_min'] for c in comps)
            m_y_max = max(c['y_max'] for c in comps)
            self.micros[m_id] = {
                'components': comps,
                'boundary': (m_x_min, m_y_min, m_x_max, m_y_max)
            }

    def generate(self):
        graphs = {}
        for m_id, micro_data in self.micros.items():
            graph = GridGraph(m_id)
            comps = micro_data['components']
            m_x_min, m_y_min, m_x_max, m_y_max = micro_data['boundary']

            # Micro 边框加入网格（可选，作为最外围通道）
            graph.add_raw_segment(m_x_min, m_y_min, m_x_max, m_y_min)
            graph.add_raw_segment(m_x_min, m_y_max, m_x_max, m_y_max)
            graph.add_raw_segment(m_x_min, m_y_min, m_x_min, m_y_max)
            graph.add_raw_segment(m_x_max, m_y_min, m_x_max, m_y_max)

            for c in comps:
                # 自身 AABB 边框加入网格
                graph.add_raw_segment(c['x_min'], c['y_min'], c['x_max'], c['y_min'])
                graph.add_raw_segment(c['x_min'], c['y_max'], c['x_max'], c['y_max'])
                graph.add_raw_segment(c['x_min'], c['y_min'], c['x_min'], c['y_max'])
                graph.add_raw_segment(c['x_max'], c['y_min'], c['x_max'], c['y_max'])

                # 射线1: 右上角向右
                r1_y, r1_x = c['y_max'], c['x_max']
                hit_x1 = m_x_max
                for o in comps:
                    if o['id'] != c['id'] and o['y_min'] <= r1_y <= o['y_max'] and o['x_min'] >= r1_x:
                        hit_x1 = min(hit_x1, o['x_min'])
                graph.add_raw_segment(r1_x, r1_y, hit_x1, r1_y)

                # 射线2: 右下角向右
                r2_y, r2_x = c['y_min'], c['x_max']
                hit_x2 = m_x_max
                for o in comps:
                    if o['id'] != c['id'] and o['y_min'] <= r2_y <= o['y_max'] and o['x_min'] >= r2_x:
                        hit_x2 = min(hit_x2, o['x_min'])
                graph.add_raw_segment(r2_x, r2_y, hit_x2, r2_y)

                # 射线3: 右上角向上
                r3_x, r3_y = c['x_max'], c['y_max']
                hit_y1 = m_y_max
                for o in comps:
                    if o['id'] != c['id'] and o['x_min'] <= r3_x <= o['x_max'] and o['y_min'] >= r3_y:
                        hit_y1 = min(hit_y1, o['y_min'])
                graph.add_raw_segment(r3_x, r3_y, r3_x, hit_y1)

                # 射线4: 左上角向上
                r4_x, r4_y = c['x_min'], c['y_max']
                hit_y2 = m_y_max
                for o in comps:
                    if o['id'] != c['id'] and o['x_min'] <= r4_x <= o['x_max'] and o['y_min'] >= r4_y:
                        hit_y2 = min(hit_y2, o['y_min'])
                graph.add_raw_segment(r4_x, r4_y, r4_x, hit_y2)

            graph.build_graph()
            graphs[m_id] = graph

        return graphs


class PlacementVisualizer:
    """提取自你的 visualizer.py 的布局图例展示类"""
    def __init__(self, generator, graphs):
        self.gen = generator
        self.graphs = graphs
        
        # 严格复刻你原代码的色彩系统
        self.net_colors = {
            "GND": "#2E7D32",
            "VCC_5V": "#C62828",
            "VCC_3V3": "#F57C00",
            "VCC_12V": "#6A1B9A",
        }
        self.default_net_color = "#1976D2"
        self.micro_colors = [
            "#FFCDD2", "#C8E6C9", "#BBDEFB", "#FFF9C4",
            "#E1BEE7", "#B2DFDB", "#FFE0B2", "#D7CCC8",
        ]
        self.micro_border_colors = [
            "#C62828", "#2E7D32", "#1565C0", "#F9A825",
            "#7B1FA2", "#00695C", "#EF6C00", "#5D4037",
        ]

    def render(self, save_path="global_layout_view.png"):
        fig, ax = plt.subplots(figsize=(14, 14))
        
        canvas_w = self.gen.canvas['width']
        canvas_h = self.gen.canvas['height']
        
        # 1. 绘制全局 Canvas
        ax.add_patch(patches.Rectangle((0, 0), canvas_w, canvas_h, 
                                       linewidth=2, edgecolor='black', facecolor='#FAFAFA', linestyle='-'))

        # 2. 循环绘制每个 Micro 和网格
        for i, (m_id, m_data) in enumerate(self.gen.micros.items()):
            bg_color = self.micro_colors[i % len(self.micro_colors)]
            bd_color = self.micro_border_colors[i % len(self.micro_border_colors)]
            m_x_min, m_y_min, m_x_max, m_y_max = m_data['boundary']
            
            # Micro 边界区域 (添加微弱的透明背景填充)
            ax.add_patch(patches.Rectangle((m_x_min, m_y_min), m_x_max - m_x_min, m_y_max - m_y_min, 
                                           linewidth=2, edgecolor=bd_color, facecolor=bg_color, alpha=0.3, linestyle='--'))

            # 绘制专属的 Hanan Grid
            graph = self.graphs[m_id]
            drawn_edges = set()
            for (x, y), node in graph.nodes.items():
                for nbr in node.neighbors:
                    edge = tuple(sorted(((x, y), (nbr.x, nbr.y))))
                    if edge not in drawn_edges:
                        drawn_edges.add(edge)
                        # 网格线绘制为灰色虚线（避免干扰重点信息）
                        ax.plot([x, nbr.x], [y, nbr.y], color='gray', linewidth=0.6, alpha=0.5, linestyle=':')

            # 绘制元器件和焊盘
            for c in m_data['components']:
                # 元器件深灰色 AABB
                ax.add_patch(patches.Rectangle((c['x_min'], c['y_min']), c['x_max'] - c['x_min'], c['y_max'] - c['y_min'], 
                                               linewidth=1.2, edgecolor='#424242', facecolor='#E0E0E0', alpha=0.9))
                # 绘制小焊盘矩形
                for p in c['pins']:
                    px, py = p['abs_pos']
                    pw, ph = p['size']
                    # 省略 Pin 上的文字，只留下显眼的小金块作为焊盘
                    ax.add_patch(patches.Rectangle((px - pw/2, py - ph/2), pw, ph, 
                                                   linewidth=0.5, edgecolor='black', facecolor='#FFCA28', zorder=3))

        # 3. 绘制飞线 (Flylines) - 严格遵循 netlist 逻辑
        for edge in self.gen.netlist.get("edges", []):
            src_comp = edge["source"]["component"]
            src_pin = edge["source"]["pin"]
            tgt_comp = edge["target"]["component"]
            tgt_pin = edge["target"]["pin"]
            net_name = edge.get("net", "")
            
            # 从之前缓存的坐标字典中获取两端点坐标
            try:
                src_pos = self.gen.pin_coords[src_comp][src_pin]
                tgt_pos = self.gen.pin_coords[tgt_comp][tgt_pin]
                
                color = self.net_colors.get(net_name, self.default_net_color)
                # 绘制点对点的直线 (遵循原版 visualizer 风格)
                ax.plot([src_pos[0], tgt_pos[0]], [src_pos[1], tgt_pos[1]], 
                        color=color, linewidth=1.0, alpha=0.6, zorder=2)
            except KeyError:
                # 忽略找不到对应坐标的异常引脚
                continue

        # 全局视口居中调整
        ax.set_xlim(-5, canvas_w + 5)
        ax.set_ylim(-5, canvas_h + 5)
        ax.set_aspect('equal')
        plt.axis('off')  # 隐藏全局座标轴，视觉上更清晰
        plt.title("Global PCB Layout (Micro Regions, Hanan Grids & Flylines)", fontsize=16, pad=15)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"可视化图像已成功生成并保存至：{save_path}")
        plt.show()

def export_grid_to_json(generator, graphs, output_path="dataset_v3_with_grid.json"):
    """
    将生成的 Hanan Grid 按照 Node & Edge List (方案 A) 格式导出为 JSON，
    并追加到原有的 dataset 数据中。
    """
    routing_grid = {}

    for m_id, graph in graphs.items():
        micro_grid = {
            "nodes": {},              # 格式: {"n0": [x, y], "n1": [x, y], ...}
            "edges": [],              # 格式: [["n0", "n1", weight], ...]
            "pin_access_nodes": {}    # 格式: {"Comp_1.Pad_1": "n0", ...}
        }

        # 1. 为所有节点分配唯一 ID (n0, n1, n2...)，并记录坐标
        node_to_id = {}
        id_counter = 0
        for (x, y), node in graph.nodes.items():
            node_id = f"n{id_counter}"
            node_to_id[node] = node_id
            micro_grid["nodes"][node_id] = [round(x, 4), round(y, 4)]
            id_counter += 1

        # 2. 提取所有的边 (无向图，避免重复添加)
        added_edges = set()
        for node in graph.nodes.values():
            n1_id = node_to_id[node]
            for nbr in node.neighbors:
                n2_id = node_to_id[nbr]
                
                # 对 ID 进行排序作为唯一键，确保 n1-n2 和 n2-n1 只被记录一次
                edge_key = tuple(sorted([n1_id, n2_id]))
                if edge_key not in added_edges:
                    added_edges.add(edge_key)
                    # 计算两个节点之间的欧几里得距离作为边的权重 (weight)
                    dist = math.hypot(node.x - nbr.x, node.y - nbr.y)
                    micro_grid["edges"].append([n1_id, n2_id, round(dist, 4)])

        # 3. 计算 Pin Access Nodes (寻找距离每个焊盘最近的网格节点)
        micro_data = generator.micros[m_id]
        for comp in micro_data['components']:
            comp_id = comp['id']
            for pin in comp['pins']:
                pin_id = pin['id']
                px, py = pin['abs_pos']
                
                # 寻找最近的网格点
                min_dist = float('inf')
                nearest_node_id = None
                
                for node in graph.nodes.values():
                    dist = math.hypot(node.x - px, node.y - py)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_node_id = node_to_id[node]
                
                # 记录映射关系，键名例如 "Comp_1.Pad_1"
                pin_key = f"{comp_id}.{pin_id}"
                micro_grid["pin_access_nodes"][pin_key] = nearest_node_id

        routing_grid[m_id] = micro_grid

    # 4. 将网格数据合并到原始数据中
    # 拷贝一份原始数据字典，避免污染内存中的原对象
    output_data = generator.data.copy()
    output_data["routing_grid"] = routing_grid

    # 5. 写入 JSON 文件
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2)
        
    print(f"✅ 包含网格结构的全新数据集已成功导出至：{output_path}")


if __name__ == "__main__":
    # 1. 自动读取 dataset、转换 AABB，并执行向右/向上射线打断网格计算
    generator = PCBGridGenerator('dataset/dataset_v3.json')
    result_graphs = generator.generate()

    # 2. 调用符合 visualizer.py 设计理念的渲染器
    visualizer = PlacementVisualizer(generator, result_graphs)
    visualizer.render(save_path='global_layout_view.png')
    
    # 3. 导出包含网格结构的新 JSON 文件 
    export_grid_to_json(generator, result_graphs, output_path="dataset_v3_with_grid.json")