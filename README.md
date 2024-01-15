# Chiplet Flow Control
## To Build
- `bash ./chiplet-flow-ctrl/scripts/build_garnetalone.sh` (Garnet standalone mode)
- `bash ./chiplet-flow-ctrl/scripts/build.sh` (SE mode)

## To Run
- `bash ./chiplet-flow-ctrl/scripts/run_syn_traffic_distL2_MC.sh` (Garnet standalone mode)
- `bash ./chiplet-flow-ctrl/scripts/run.sh` (SE mode)

## To Contribute
- `bash ./chiplet-flow-ctrl/scripts/push-to-github.sh`

## 1. Overview
This is the source code of gem5 with: Chiplet Flow Control Network on Chip.

In Order to provide a **High-performance Deadlock-free** NoC service in situation of **Chiplet-based architecture**, the following part of NoC is redesigned:
- A three-stage routing algorithm is proposed
- A mirco-architecture of routers is redesinged
- A protocol to estabulish a fast transport channel bufferlessly
- to be continued...

## 2. Baseline
### 2.1 NoC Topology
Raleted files:  `/chiplet-flow-ctrl/configs/topologies/Chiplets_Mesh_distrbuteL2_MC_se.py`

![baseTopology](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/baseTopology.jpg)

The baseline network on chip's topology composed of four 4x4 mesh chiplet and a 4x4 interposer.
#### Connection
There are 16 interposer router and 64 (4x4x4) chiplet router in the system:
- Chiplet router connect to Chiplet router via chiplet links
- Chiplet router connect to interposer router via vertical links
- Interposer router connect to interposer router via interposer links
- Chiplet router connect to non-NoC components via Network interface(NI)
#### Direction
- There are four direction in mesh: South, North, East, West
- There are two direction in vertiacl links: Up, Down
- Another four logic direction used for routing algorithm: X+, X-, Y+, Y-
#### Identification
- Chiplet router's ID start from 0 to 63
- Interposer router's ID start from 64 to 79
- Chiplet's ID start from 0 to 3
- Boundary router means those chiplet routers who connect to interposer router: 4 each chiplet


### 2.2 Memory Architecture
Raleted files:  `/chiplet-flow-ctrl/configs/topologies/Chiplets_Mesh_distrbuteL2_MC_se.py`

![memoryArch](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/memoryArch.jpg)

Each non-NoC node has instruction L1/L2 cache, data L1/L2 cache, Memory Controller(Directory) and Core. These components connect with NoC router via NI.
- Core: x86 ISA, 1 Ghz Out of Order cores, No prefetcher
- L1 Cache: Private, 32kB Ins., 64kB Data, 4-way set associative (configurable)
- L2 Cache: Shared, distributed, 2MB, 8-way set associative (configurable)
- Coherent protocal: MESI, Directory-based

## 3. Three-stage routing algorithem
Raleted files: `gem5/chiplet-flow-ctrl/src/mem/ruby/network/garnet/RoutingUnit.cc`

![routingAlgorithm](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/Routing.jpg)

A three-stage routing algorithm is implmented in RoutingUnit::outportComputeXY(...)
it allows three different routing algorithm in src chiplet, interposer and dest chiplet.

NOTE: this routingUnit DO NOT ensure deadlock free, which will be ensured through flow control technology.

## 4. MicroArchitecture of Routers

## Contact me
Zixuan Liu: <a href="mailto:testmail@gmail.com">zishuan.leo@gmail.com</a> or <a href="mailto:testmail@gmail.com">zx_liu@sjtu.edu.cn</a>
