"""
主入口脚本
"""

import json
from pathlib import Path

from config import CONFIG_DEFAULT
from PCB_placement_generator import PCBPlacementGenerator
from PCB_routing_generator import PCBRouter, RouterConfig, route_pcb
from visualizer import PlacementVisualizer


def main():
    print("=" * 60)
    print("PCB Placement & Routing Generator v3")
    print("=" * 60)
    
    # 确保输出目录存在
    output_dir = Path("./dataset")
    output_dir.mkdir(exist_ok=True)
    
    # 1. 创建生成器
    generator = PCBPlacementGenerator(config=CONFIG_DEFAULT, seed=17)
    
    # 2. 生成单个电路
    print("\n[1] Generating single circuit...")
    placement_data, circuit_txt = generator.generate_single_circuit("my_circuit")
    
    # 打印 Micro 信息
    print(f"\nMicro Regions:")
    for micro_id, info in placement_data["metadata"]["micro_info"].items():
        print(f"  {micro_id}:")
        print(f"    - BBox: {[round(x, 2) for x in info['bbox']]}")
        print(f"    - Components: {info['num_components']}")
        print(f"    - Area: {info['area']:.2f} mm²")
    
    print(f"\nTotal Components: {placement_data['metadata']['num_components']}")
    print(f"Total Edges: {placement_data['metadata']['num_edges']}")
    print(f"Total Nets: {placement_data['metadata']['num_nets']}")
    
    # 4. 生成布线
    print("\n[3] Generating PCB routing...")
    routing_config = RouterConfig(
        # grid_size=0.5,
        # num_layers=2,
        # trace_width=0.15,
        # via_diameter=0.6,
        # via_drill=0.3,
        # clearance_trace_trace=0.15,
        # clearance_trace_pad=0.15,
        # clearance_trace_via=0.15,
        # clearance_via_via=0.15,
        # via_cost=10.0,
        # max_iterations=100000
    )

    # routing_data = route_pcb(circuit["placement"], circuit["netlist"], routing_config)
    # routing_data, ucg = route_pcb_with_ucg(circuit["placement"], circuit["netlist"], routing_config)
    routing_data, ucg = route_pcb(
        placement_data["placement"], 
        placement_data["netlist"], 
        routing_config,
        placement_data["metadata"]
    )
    
    # 打印布线统计
    stats = routing_data["statistics"]
    print(f"\nRouting Statistics:")
    print(f"  - Total Edges: {stats['total_edges']}")
    print(f"  - Successful Routes: {stats['success_count']}")
    print(f"  - Failed Routes: {stats['failed_count']}")
    success_rate = (stats['success_count'] / stats['total_edges'] * 100) if stats['total_edges'] > 0 else 0
    print(f"  - Success Rate: {success_rate:.1f}%")
    print(f"  - Total Length: {stats['total_length']:.2f} mm")
    print(f"  - Total Vias: {stats['total_vias']}")
    
    # 5. 保存结果
    print("\n[4] Saving outputs...")
    
    # 定义输出路径
    placement_path = output_dir / "placement.json"
    circuit_txt_path = output_dir / "circuit.txt"
    routing_path = output_dir / "routing.json"
    ucg_path = output_dir / "circuit_ucg.json"
    placement_img_path = output_dir / "placement.png"
    routing_layers_img_path = output_dir / "routing_layers.png"
    routing_combined_img_path = output_dir / "routing_combined.png"
    
    with open(placement_path, "w") as f:
        json.dump(placement_data, f, indent=2)
    print(f"  - Saved: {placement_path}")
    
    with open(circuit_txt_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(circuit_txt)
    print(f"  - Saved: {circuit_txt_path}")

    with open(routing_path, "w") as f:
        json.dump(routing_data, f, indent=2)
    print(f"  - Saved: {routing_path}")
    
    with open(ucg_path, "w") as f:
        json.dump(ucg, f, indent=2)
    print(f"  - Saved: {ucg_path}")

    # 6. 验证 Micro 不重叠
    print("\n[5] Validating Micro regions...")
    micro_info = placement_data["metadata"]["micro_info"]
    micro_ids = list(micro_info.keys())
    
    all_valid = True
    for i, m1_id in enumerate(micro_ids):
        for j, m2_id in enumerate(micro_ids):
            if i >= j:
                continue
            
            bbox1 = micro_info[m1_id]["bbox"]
            bbox2 = micro_info[m2_id]["bbox"]
            
            overlap = not (
                bbox1[2] < bbox2[0] or bbox2[2] < bbox1[0] or
                bbox1[3] < bbox2[1] or bbox2[3] < bbox1[1]
            )
            
            if overlap:
                print(f"  WARNING: {m1_id} and {m2_id} overlap!")
                all_valid = False
    
    if all_valid:
        print("  All Micro regions are non-overlapping!")
    
    # 7. 可视化
    print("\n[6] Generating visualizations...")
    
    # 从 UCG 提取 micro_partition
    micro_partition = {}
    ucg_root = ucg.get("UCG_Graph", ucg)
    level1 = ucg_root["level_1_details"]
    level0 = ucg_root["level_0_global"]

    micro_bbox_info = {node["id"]: node for node in level0["nodes"]}

    for micro_id, details in level1.items():
        micro_partition[micro_id] = [node["id"] for node in details["nodes"]]
        bbox_node = micro_bbox_info.get(micro_id, {})
        w = bbox_node.get("w", "N/A")
        h = bbox_node.get("h", "N/A")
        center_x = bbox_node.get("center_x", "N/A")
        center_y = bbox_node.get("center_y", "N/A")
        
        print(f"  {micro_id}:")
        print(f"    - Components: {len(details['nodes'])}")
        print(f"    - Size: w={w}, h={h}")
        print(f"    - Center: ({center_x}, {center_y})")
    
    # 创建可视化器
    visualizer = PlacementVisualizer(figsize=(14, 12))
    
    # 6.1 布局结果可视化（带Micro分区和网络连接）
    print("\n  [6.1] PCB placement with Micro regions and nets...")
    visualizer.visualize_placement(
        placement_data,
        show_nets=True,
        show_micros=True,
        micro_partition=micro_partition,
        save_path=str(placement_img_path)
    )
    
    # 6.2 布线结果可视化（分层视图）
    print("\n  [6.2] PCB routing - layer view...")
    visualizer.visualize_routing(
        placement_data,
        routing_data,
        show_components=True,
        show_vias=True,
        layer_filter=None,  # 显示所有层
        save_path=str(routing_layers_img_path)
    )
    
    # 6.3 布线结果可视化（合并视图）
    print("\n  [6.3] PCB routing - combined view...")
    visualizer.visualize_routing_combined(
        placement_data,
        routing_data,
        show_components=True,
        show_vias=True,
        save_path=str(routing_combined_img_path)
    )
    
    print("\n" + "=" * 60)
    print("Generation complete!")
    print("=" * 60)
    print(f"\nOutput files:")
    print(f"  - {placement_path}        (电路布局数据)")
    print(f"  - {routing_path}          (布线数据)")
    print(f"  - {placement_img_path}         (布局可视化)")
    print(f"  - {routing_layers_img_path}    (布线分层视图)")
    print(f"  - {routing_combined_img_path}  (布线合并视图)")
    print(f"  - {circuit_txt_path}           (电路文本描述)")
    print(f"  - {ucg_path}      (UCG格式数据)")



def batch_generate(num_circuits: int = 10, output_dir: str = "./dataset/batch/"):
    """批量生成多个电路"""
    print("=" * 60)
    print(f"Batch Generation: {num_circuits} circuits")
    print("=" * 60)
    
    # 确保输出目录存在
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # 批量生成统计
    total_success = 0
    total_failed = 0
    
    for i in range(num_circuits):
        circuit_id = f"circuit_{i:04d}"
        print(f"\n{'='*60}")
        print(f"[{i+1}/{num_circuits}] Generating {circuit_id}...")
        print(f"{'='*60}")
        
        try:
            # 创建当前电路的输出目录
            circuit_dir = output_path / circuit_id
            circuit_dir.mkdir(exist_ok=True)
            
            # 1. 创建生成器（使用不同的seed）
            generator = PCBPlacementGenerator(config=CONFIG_DEFAULT, seed=i)
            
            # 2. 生成电路
            print(f"  [1] Generating circuit layout...")
            placement_data, circuit_txt = generator.generate_single_circuit(circuit_id)
            
            print(f"    - Components: {placement_data['metadata']['num_components']}")
            print(f"    - Edges: {placement_data['metadata']['num_edges']}")
            print(f"    - Nets: {placement_data['metadata']['num_nets']}")
            
            # 3. 生成布线
            print(f"  [2] Generating routing...")
            routing_config = RouterConfig(
                # grid_size=0.5,
                # num_layers=2,
                # trace_width=0.15,
                # via_diameter=0.6,
                # via_drill=0.3,
                # clearance_trace_trace=0.15,
                # clearance_trace_pad=0.15,
                # clearance_trace_via=0.15,
                # clearance_via_via=0.15,
                # via_cost=10.0,
                # max_iterations=100000
            )
            
            routing_data, ucg = route_pcb(
                placement_data["placement"], 
                placement_data["netlist"], 
                routing_config,
                placement_data["metadata"]
            )
            
            stats = routing_data["statistics"]
            success_rate = (stats['success_count'] / stats['total_edges'] * 100) if stats['total_edges'] > 0 else 0
            print(f"    - Success Rate: {success_rate:.1f}%")
            print(f"    - Total Length: {stats['total_length']:.2f} mm")
            print(f"    - Total Vias: {stats['total_vias']}")
            
            # 4. 定义输出路径
            placement_path = circuit_dir / f"{circuit_id}_placement.json"
            circuit_txt_path = circuit_dir / f"{circuit_id}_circuit.txt"
            routing_path = circuit_dir / f"{circuit_id}_routing.json"
            ucg_path = circuit_dir / f"{circuit_id}_ucg.json"
            placement_img_path = circuit_dir / f"{circuit_id}_placement.png"
            routing_combined_img_path = circuit_dir / f"{circuit_id}_routing_combined.png"
            routing_layers_img_path = circuit_dir / f"{circuit_id}_routing_layers.png"

            # 5. 保存文件
            print(f"  [3] Saving files...")
            
            with open(placement_path, "w") as f:
                json.dump(placement_data, f, indent=2)
            
            with open(circuit_txt_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(circuit_txt)
            
            with open(routing_path, "w") as f:
                json.dump(routing_data, f, indent=2)
            
            with open(ucg_path, "w") as f:
                json.dump(ucg, f, indent=2)
            
            # 6. 生成可视化
            print(f"  [4] Generating visualizations...")
            
            # 从 UCG 提取 micro_partition
            micro_partition = {}
            ucg_root = ucg.get("UCG_Graph", ucg)
            level1 = ucg_root["level_1_details"]
            for micro_id, details in level1.items():
                micro_partition[micro_id] = [node["id"] for node in details["nodes"]]
            
            # 创建可视化器
            visualizer = PlacementVisualizer(figsize=(14, 12))
            
            # 布局可视化
            visualizer.visualize_placement(
                placement_data,
                show_nets=True,
                show_micros=True,
                micro_partition=micro_partition,
                save_path=str(placement_img_path)
            )
            
            # 布线可视化
            visualizer.visualize_routing(
                placement_data,
                routing_data,
                show_components=True,
                show_vias=True,
                save_path=str(routing_layers_img_path)
            )

            visualizer.visualize_routing_combined(
                placement_data,
                routing_data,
                show_components=True,
                show_vias=True,
                save_path=str(routing_combined_img_path)
            )
            
            print(f"Successfully generated {circuit_id}")
            print(f"Saved to: {circuit_dir}/")
            total_success += 1
            
        except Exception as e:
            print(f"Error: Failed to generate {circuit_id}: {str(e)}")
            total_failed += 1
            import traceback
            traceback.print_exc()
    
    # 打印总结
    print("\n" + "=" * 60)
    print("Batch Generation Complete!")
    print("=" * 60)
    print(f"Total Circuits: {num_circuits}")
    print(f"Successfully Generated: {total_success}")
    print(f"Failed: {total_failed}")
    print(f"Success Rate: {(total_success/num_circuits*100):.1f}%")
    print(f"\nOutput Directory: {output_path.absolute()}")
    print("\nGenerated files per circuit:")
    print(f"  - {{circuit_id}}_placement.json")
    print(f"  - {{circuit_id}}_routing.json")
    print(f"  - {{circuit_id}}_circuit.txt")
    print(f"  - {{circuit_id}}_ucg.json")
    print(f"  - {{circuit_id}}_placement.png")
    print(f"  - {{circuit_id}}_routing.png")



if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="PCB Placement & Routing Generator")
    parser.add_argument("--batch", type=int, default=0, 
                        help="Batch generation mode: number of circuits to generate")
    parser.add_argument("--output", "-o", type=str, default="./dataset",
                        help="Output directory")
    
    args = parser.parse_args()
    
    if args.batch > 0:
        batch_generate(num_circuits=args.batch, output_dir=f"{args.output}/batch")
    else:
        main()