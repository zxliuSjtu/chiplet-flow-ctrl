# Chiplet Flow Control
## To Build
- `scons build/Garnet_standalone/gem5.opt -j$(nproc)` (Garnet standalone mode)
- `scons build/X86/gem5.opt -j$(nproc) PROTOCOL=MESI_Three_Level` (Full System mode)

## To Run
### 1. Garnet standalone mode
- `bash ./chiplet-flow-ctrl/scripts/run_syn_traffic_distL2_MC.sh`
### 2. Full System mode
- **Run from checkpiont:** `bash ./chiplet-flow-ctrl/scripts/x86_parsec_fs_run_from_ckpt.sh`
- **Run from begining/to make checkpiont :** `bash ./chiplet-flow-ctrl/scripts/x86_parsec_fs_checkpoint.sh`

note: remember to use: /gem5/chiplet-flow-ctrl/configs/boot/hack_back_ckpt.rcS as your script when you make a checkpoint, this will help you to update new script when you run from this checkpoint, instead of the old script when making the checkpiont.

- **Run PARSEC without checkpoint :** `bash ./chiplet-flow-ctrl/scripts/x86_parsec_fs_run_parsec.sh`

(Dont forget to modify the directory in .sh file)
### 3. Docker
gem5 Docker: https://www.gem5.org/documentation/general_docs/building#docker


## To Draw
### 1. Draw Synthetic Traffic Latency
#### step1: run all the synthetic traffic, output directory is /myM5out
- `bash ./scripts/run_garnetalone_cycle.sh <test_id>`
#### step2: draw all the synthetic traffic, output directory is /pythonOut
- `bash ./scripts/draw_cycle.sh <test_id>`
### 2. Draw Synthetic Traffic Saturation Throughput
- flit_size of garnet = 128 bits
- throughput = (received_flits_number * flit_size)/sim_seconds
- `python ./python_scripts/plot_sat_throughput.py `

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

![baseTopology](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/baseTopology.png)

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

## 3. Three-stage routing algorithm
Raleted files: `gem5/chiplet-flow-ctrl/src/mem/ruby/network/garnet/RoutingUnit.cc`

![routingAlgorithm](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/Routing.jpg)

A three-stage routing algorithm is implmented in RoutingUnit::outportComputeXY(...)
it allows three different routing algorithm in src chiplet, interposer and dest chiplet.

Use `--routing-algorithm` to control routing algorithm:
- `--routing-algorithm = 0`: The default routing algorithm is a deterministic table-based routing algorithm with shortest paths. Link weights can be used to prioritize certain links over others. See src/mem/ruby/network/Topology.cc for details about how the routing table is populated.
- `--routing-algorithm = 1`: Mesh XY routing, deprecated under chiplet situation.
- `--routing-algorithm = 2`: Three-stage routing algorithm

NOTE: this routingUnit DO NOT ensure deadlock free, which will be ensured through flow control technology.

## 4. MicroArchitecture of Routers
![MicroArch](https://github.com/zxliuSjtu/chiplet-flow-ctrl/blob/main/figures/MicroArch.png)
## Code format
Use small Camel-Case for varibles name, use big Camel-Case for function name, like:
- myPassword, meshRows, onChipRouter
- IsMyTurn(), GetNumRows()

Use m_ for private class member, like:
- m_latency, m_cfc

## Contact me
Zixuan Liu: <a href="mailto:testmail@gmail.com">zishuan.leo@gmail.com</a> or <a href="mailto:testmail@gmail.com">zx_liu@sjtu.edu.cn</a>
