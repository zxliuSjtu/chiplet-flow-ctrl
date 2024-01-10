./build/Garnet_standalone/gem5.opt configs/example/garnet_synth_traffic.py \
            --topology=Mesh_XY \
            --num-cpus=16 \
            --num-dirs=16 \
            --mesh-rows=4 \
            --network=garnet \
            --router-latency=1 \
            --sim-cycles=10000 \
            --injectionrate=1.00 \
            --synthetic=uniform_random \
            --routing-algorithm=1