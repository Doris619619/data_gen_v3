prompt = """
Role: You are an expert Analog IC Layout Engineer and Graph Theorist.

Objective: Transform the provided hierarchical Netlist into a Unified Constraint Graph (UCG) based on the specific definitions provided below.

################################################################################
### CATEGORIZED RULES & CONSTRAINTS (STRICTLY FOLLOW)
################################################################################

I. GENERAL SYSTEM RULES
1. ALL dx and dy MUST be a SINGLE FLOAT (e.g., 15.0). ABSOLUTELY NO BRACKETS [min, max].
2. COORDINATE SYSTEM: +X is Right, +Y is Down. 
3. HIERARCHY ISOLATION (STRICT BOUNDARIES):
   - level_0_global.routing_resources: EXCLUSIVELY for inter-micro connectivity. 
     Mandatory Syntax: "Micro_X.NetName". PROHIBITED: Never use specific Component IDs.
   - level_1_details.{Micro_N}.routing_resources: EXCLUSIVELY for intra-micro connectivity. 
     Mandatory Syntax: "MN_Comp_X.Pad_Y(NetName)". All components MUST belong to {Micro_N}.
4. NO SPURIOUS CONNECTIONS: Do not force a connection just to fill the JSON. If a net only appears on one pin, no path_sequence unless it connects to Global.
5. ROTATION SYSTEM: Angles are in Degrees (0.0, 90.0, 180.0, 270.0/-90.0). 
   - Component rotation affects its internal local coordinate system.
   - All output nodes MUST include the component's absolute rotation.
   - All nodes MUST include "rot" (degrees). Component rotation affects Pad positioning.

II. SPATIAL TOPOLOGY RULES (POSITIONING & OFFSET)
1. VECTOR-BASED POSITIONING:
   - Relative position is the vector from SOURCE to TARGET: [dx, dy].
   - Horizontal alignment: Set dy = 0.0. Target is Right if dx > 0, Left if dx < 0.
   - Vertical alignment: Set dx = 0.0. Target is Below if dy > 0, Above if dy < 0.
2. STRICT NON-OVERLAPPING RULE (HARD CONSTRAINT):
   - Components & Micros: No two entities (Micro-to-Micro or Comp-to-Comp) can overlap.
   - Space Nodes: Routing channels (Space Nodes) occupy physical width. 
    The center-to-center offset (|dx| and |dy|) MUST be:
    |dx| >= (Size_Source.w/2) + Channel_Width.x + (Size_Target.w/2)
    |dy| >= (Size_Source.h/2) + Channel_Width.y + (Size_Target.h/2)
3. SPATIAL-CHANNEL COUPLING:
   - Scenario A (Shared Nets): If blocks share nets, use Channel_Width of the largest net type.
   - Scenario B (No Shared Nets): If adjacent but NO shared nets, MUST still apply Offset Formula with Default Channel_Width = 3.0.
   - CRITICAL: SPATIAL edge existence DOES NOT require a corresponding ROUTING edge.
4. ROTATION:
   - If rot is 90 or 270, W and H of the component swap for the formula.
5. COORDINATE CONSISTENCY (CHAIN RULE):
   - For any chain of blocks (e.g., M1 -> M2 -> M3), the total offset MUST be additive.If M1 -> M2 has dx_1 and M2 ->  M3 has dx_2, then M1 -> M3 MUST have dx = dx_1 + dx_2.
6. DIRECTED ACYCLIC TOPOLOGY (DAG) & FULL ANCHORING:RULE: 
   - The spatial graph MUST be a Directed Acyclic Graph (DAG).
   - NO CYCLES: If A -> B and B -> C exist, a relation C -> A is STRICTLY PROHIBITED. All spatial vectors must follow a consistent global direction (e.g., left-to-right or top-down).
   - FULL ENUMERATION: Every component listed in nodes MUST be anchored. For a set of components $\{C_1, C_2, ... C_n\}$, you must define at least $n-1$ spatial edges such that every component's coordinate can be resolved relative to a common root.
   - VALID EXAMPLES: (1->2, 2->3, 1->3) is VALID.
   - INVALID EXAMPLES: (1->2, 2->3, 3->1) is INVALID (Cycle detected).  


III. ROUTING RESOURCES & SIZE INFERENCE
1. SPACE NODE OCCUPANCY:
   - Treat every "Space Node" as a physical routing channel. 
   - Every routing edge uses a Space Node, so Channel_Width MUST be > 0.0.
   - Routing_resources must contain inferred Space Node sizes (Small/Medium/Large).
2. FIXED CHANNEL_WIDTH MAPPING: channel width should map with the types of space node
   - Small  (3.0): Control/data, Local bias and Signal nets (e.g., RX, TX, PWM, NetC*, SDA, SCL, /A, /B, /LED).
   - Medium (6.0): Supplied distribution rails (e.g., VCC_5V).
   - Large  (8.0): High-current/power rails (e.g., GND, VCC_12V, VCC_48V, /EL+, /EL-, Inductors, TO-252 MOSFETs, large diodes/caps.).


IV. CONNECTIVITY & NET CHECK (HARD CONSTRAINTS)
1. SAME-NET ONLY (Literal String Comparison):
   - A routing edge is ONLY allowed if Source.Pin.Net == Target.Pin.Net.
   - Every string in path_sequence MUST end with a net label in parentheses that EXACTLY matches net_id. (e.g., net_id "GND" -> "Pad_X(GND)").
   - Check net of each pad. Connecting RX to TX is a SHORT CIRCUIT violation.
   - If a Pin has no partner with same net name in the level, NO routing edge.
2. PIN NAMING DISCIPLINE: Endpoints must use pin names explicitly defined in the node's "pins" map.
3. ONE-TO-MANY CONNECTIVITY (Shared Nodes): A single Pad CAN be used in multiple path_sequence entries if it belongs to a shared net (e.g., GND). Ensure all GND pads connect to common GND resources.
4. EXHAUSTIVE NET MAPPING (TOTAL COVERAGE): You MUST scan EVERY pad of EVERY component. If a pad has a net name that exists on any other pad or micro-interface within the same level, a routing_resources entry MUST be generated.

################################################################################

1. The UCG Paradigm Definition
Layer 1: Spatial Topology ({"type": "SPATIAL", "source": "A", "target": "B", "dx": float, "dy": float}).
Layer 2: Routing Graph ({"source": "NodeA.Pin", "via_space_node": {"id": "SN_X", "size": "Small/Medium/Large"}, "target": "NodeB.Pin"}).
Nodes: COMPONENT {id, type, w, h, pins}, SPACE {id, anchor_node, direction}.
Edges: SPATIAL, AFFINITY (weight 1-5), ROUTING_FLOW {net_id, path_sequence}.


2. Input Netlist & Constraints
""" + input_netlist_content + """


3. JSON Output Template (STRICTLY FOLLOW)
{
  "UCG_Graph": {
    "level_0_global": {
      "nodes": [{"id": "Micro_1", "type": "Micro", "w": 10.0, "h": 8.0}],
      "spatial_topology": [{"type": "SPATIAL", "source": "Micro_1", "target": "Micro_2", "dx": 25.0, "dy": 0.0}],
      "routing_resources": [{"net_id": "VCC_5V", "path_sequence": ["Micro_1.VCC_5V", "SN_1", "Micro_3.VCC_5V"]}]
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
            "__comment": "MANDATORY: MUST list ALL components to form a full DAG chain. No truncation."
            "type": "SPATIAL", 
            "source": "Comp_1", 
            "target": "Comp_2", 
            "dx": 10.0, 
            "dy": 0.0
          }
        ],
        "routing_resources": [
          {
            "__comment": "MANDATORY: Generate path_sequence for EVERY same-net ID found in nodes."
            "net_id": "Net-(LED1-PadA)",
            "path_sequence": ["Comp_A.Pad_1(Net-(LED1-PadA))", "SN_X", "Comp_B.Pad_2(Net-(LED1-PadA))"]
          }
        ]
      },
      "Micro_2": { "nodes": [], "spatial_topology": [], "routing_resources": [] },
      "Micro_3": { "nodes": [], "spatial_topology": [], "routing_resources": [] },
      "Micro_N": { "nodes": [], "spatial_topology": [], "routing_resources": [] }
    }
  }
}


4. Final Requirements: Generate full JSON. Exhaustive level_1_details for ALL Micros in the input (from Micro_1 to Micro_N). Check Net Consistency for every edge. 



Begin JSON output now:
"""
