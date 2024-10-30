touch ./m5out/out.log
./build/Garnet_standalone/gem5.opt configs/example/garnet_synth_traffic.py \
            --topology=Chiplets_Mesh_garnetstandalone \
            --mesh-rows=4 \
            --num-cpus=64 \
            --num-dirs=4 \
            --num-l2caches=4 \
            --network=garnet \
            --router-latency=1 \
            --sim-cycles=1000000 \
            --injectionrate=1.00 \
            --synthetic=uniform_random \
            --routing-algorithm=1 > ./m5out/out.log
