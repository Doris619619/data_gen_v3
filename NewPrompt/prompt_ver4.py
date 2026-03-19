prompt = """
Role: You are an expert Analog IC Layout Engineer and Graph Theorist.

Objective: Transform the provided hierarchical Netlist into a Unified Constraint Graph (UCG) based on the specific definitions provided below.

################################################################################
### I. THE UCG PARADIGM DEFINITION (DATA MODEL)
################################################################################
You must represent the circuit layout using the following graph layers:

1. HIERARCHY LEVELS:
   - Level 0 (Global): Defines the spatial arrangement and routing between "Micros" (functional blocks).
   - Level 1 (Details): Defines the spatial arrangement and routing between "Components" within each individual Micro.

2. NODES:
   - MICRO: A logical container representing a sub-circuit with a defined boundary {w, h}. (Exclusive to Level 0)
   - COMPONENT: {id, type, w, h, pads, rot} (Exclusive to Level 1)

3. SPATIAL TOPOLOGY (Relative Positioning):
   - Edge Syntax: {"type": "SPATIAL", "source": "A", "target": "B", "dx": float, "dy": float}
   - In Level 0: Source/Target are Micro IDs (e.g., Micro_1, Micro_2).
   - In Level 1: Source/Target are Component IDs within that specific Micro (e.g., M1_Comp_1, M1_Comp_2).

4. ROUTING RESOURCES (Connectivity Evidence):
   - Edge Syntax: {"net_id": "SHARED_NET_NAME", "path_sequence": ["Source_ID.Pad_X(SHARED_NET_NAME)", "Target_ID.Pad_Y(SHARED_NET_NAME)"]}
   - Purpose: This layer serves as the physical justification for the Calculated_Channel_Width used in the Spatial Topology.

################################################################################
### II.  CATEGORIZED RULES & CONSTRAINTS (STRICTLY FOLLOW)
################################################################################

--- [A. GENERAL SYSTEM RULES] ---
A1. ALL dx and dy MUST be a SINGLE FLOAT (e.g., 15.0). ABSOLUTELY NO BRACKETS [min, max].

A2. COORDINATE SYSTEM: +X is Right, +Y is Down. 

A3. ROTATION SYSTEM: Angles are in Degrees (0.0, 90.0, 180.0, 270.0/-90.0). 
   - Component rotation affects its internal local coordinate system.
   - All output nodes MUST include the component's absolute rotation.
   - All nodes MUST include "rot" (degrees). Component rotation affects Pad positioning.


--- [B.  SPATIAL TOPOLOGY RULES (ROUTING-AWARE)] ---
B1. VECTOR-BASED POSITIONING:
   - Relative position is the vector from SOURCE to TARGET: [dx, dy].
   - Horizontal alignment: Set dy = 0.0. Target is Right if dx > 0, Left if dx < 0.
   - Vertical alignment: Set dx = 0.0. Target is Below if dy > 0, Above if dy < 0.

B2. DYNAMIC CHANNEL_WIDTH CALCULATION (SILENT STEP):
    For any two blocks (two Micros or two Components) that will be connected by a SPATIAL edge:

    Step 1: Identify Shared Nets
     - Collect all net names that appear in the pads of Source block and Target block.
     - Shared Nets = intersection of the two sets.

    Step 2: Classify Each Shared Net
     - Small (3.0): nets that are single-ended signals, control lines, or differential pairs (e.g., /Z+, /Z-, /X+, /X-, SDA, SCL, PWM).
     - Medium (6.0): power supply rails (e.g., VCC_5V, VDD, VSS).
     - Large (8.0): high-current power/ground (e.g., GND, VCC_12V, VCC_48V, /EL+, /EL-).

    Step 3: Determine Required Channel Width
     - If any shared net is POWER/GND (High Current) → channel_width = 8.0
     - Else (all shared nets are Small) →
         * If the number of Small nets ≤ 2 → channel_width = 3.0
         * If the number of Small nets > 2 → channel_width = 6.0 (to accommodate multiple signals)
     - If there are no shared nets → channel_width = 3.0 (minimum isolation)

    Example (dx/dy calculation):
     - Source: M1_Comp_P1 (w=7.154763, h=8.8965, pads: Pad_1 net=/Z-, Pad_2 net=/Z+)
     - Target: M1_Comp_D1 (w=3.154762, h=3.7965, pads: Pad_1 net=/Z+, Pad_2 net=/Z-)
     - Shared nets: {/Z+, /Z-} (both Small, count=2) → channel_width = 3.0
     - dx_min = (7.154763/2) + 3.0 + (3.154762/2) = 3.5773815 + 3.0 + 1.577381 = 8.1547625
     - Choose dx = 8.16 (≥ dx_min), dy = 0.0 (horizontal alignment)
     - Output: {"type": "SPATIAL", "source": "M1_Comp_P1", "target": "M1_Comp_D1", "dx": 8.16, "dy": 0.0}

B3. STRICT NON-OVERLAPPING RULE (HARD CONSTRAINT):
    The center-to-center offset MUST accommodate both the component bodies AND the routing channels (Space Nodes).
   - Components & Micros: No two entities (Micro-to-Micro or Comp-to-Comp) can overlap.
   - Space Nodes: Routing channels (Space Nodes) occupy physical width. 
    The center-to-center offset (|dx| and |dy|) MUST be:
    |dx| >= (Size_Source.w/2) + Calculated_Channel_Width + (Size_Target.w/2)
    |dy| >= (Size_Source.h/2) + Calculated_Channel_Width + (Size_Target.h/2)
   - ROTATION: If rot is 90 or 270, W and H of the component swap for the formula.

B4. COORDINATE CONSISTENCY (CHAIN RULE):
   - For any chain of blocks (e.g., M1 -> M2 -> M3), the total offset MUST be additive.
   - If there exist spatial edges A -> B with [dx_1, dy_1] and B -> C with [dx_2, dy_2], then the implied relative position A -> C with [dx, dy] MUST satisfy [dx, dy] = [dx_1 + dx_2, dy_1 + dy_2].

B5. DIRECTED ACYCLIC TOPOLOGY (DAG) & FULL ANCHORING:RULE: 
   - The spatial graph MUST be a Directed Acyclic Graph (DAG).
   - NO CYCLES: If A -> B and B -> C exist, a relation C -> A is STRICTLY PROHIBITED. All spatial vectors must follow a consistent global direction (e.g., left-to-right or top-down).
   - FULL ENUMERATION: Every component listed in nodes MUST be anchored. For a set of components $\{C_1, C_2, ... C_n\}$, you must define at least $n-1$ spatial edges such that every component's coordinate can be resolved relative to a common root.
   - VALID EXAMPLES: (1->2, 2->3, 1->3) is VALID.
   - INVALID EXAMPLES: (1->2, 2->3, 3->1) is INVALID (Cycle detected). 

B6. COMPACT 2D-GRID STRATEGY (NON-LINEAR PLACEMENT):
   - GOAL: Aim for a balanced, space-efficient 2D grid rather than a long 1D chain.
   - FLEXIBLE VECTORS: A spatial edge [dx, dy] can have both non-zero values to achieve diagonal placement if it saves area.
   - MULTIPLE ANCHORS: To form a row/column structure, anchor new components either horizontally (to the left neighbor) or vertically (to the neighbor above).
   
################################################################################
### III.  INPUT NETLIST & CONSTRAINTS
################################################################################

""" + input_netlist_content + """


################################################################################
### IV.  JSON OUTPUT TEMPLATE
################################################################################
{
  "UCG_Graph": {
    "level_0_global": {
      "nodes": [{"id": "Micro_1", "type": "Micro", "w": 10.0, "h": 8.0}],
      "spatial_topology": [{"type": "SPATIAL", "source": "Micro_1", "target": "Micro_2", "dx": 25.0, "dy": 0.0}],
      "routing_resources": [{"net_id": "VCC_5V", "path_sequence": ["Micro_1.VCC_5V", "Micro_3.VCC_5V"]}]
    },
    "level_1_details": {
      "Micro_1": {
        "nodes": [
          {
            "id": "M1_Comp_24",
            "rot": 180.0, 
            "w": 4.064, 
            "h": 5.775, 
            "pads": {
              "Pad_1": {"rel_pos": [-1.549, -1.922], "rot": 180.0, "w": 0.864, "h": 1.9, "net": "VCC_5V"}
            }
          }
        ],
        "spatial_topology": [
          {
            "type": "SPATIAL", 
            "source": "Comp_1", 
            "target": "Comp_2", 
            "dx": 5.0, 
            "dy": 5.0
          }
        ],
       "routing_resources": [
          {
            "net_id": "Net-(LED1-PadA)",
            "path_sequence": ["Comp_A.Pad_1(Net-(LED1-PadA))", "Comp_B.Pad_2(Net-(LED1-PadA))"]
          }
        ]
      },
      "Micro_2": { "nodes": [], "spatial_topology": [], "routing_resources": [] },
      "Micro_3": { "nodes": [], "spatial_topology": [], "routing_resources": [] },
      "Micro_N": { "nodes": [], "spatial_topology": [], "routing_resources": [] }
    }
  }
}

################################################################################
### V.  FINAL REQUIREMENTS
################################################################################
   - Exhaustive Population: Generate full JSON details for ALL Micros (from Micro_1 to Micro_N) detected in the netlist.
   - Zero Placeholder Policy: Every Micro entry must contain its actual components and calculated spatial edges.
   - Pre-computation Check: Ensure every dx/dy reflects the Calculated_Channel_Width derived from the nets between them.


Begin JSON output now:
"""