export M5_PATH=/gem5/chiplet-flow-ctrl/fs
./build/X86/gem5.opt \
-d m5out/16core/test_cfc_1 \
configs/deprecated/example/fs.py \
--num-cpus=16 \
--kernel=x86-linux-kernel-5.4.49 \
--disk=x86-parsec \
-r 1 \
--checkpoint-dir=m5out/checkpoints/16kvmCores_v2 \
--restore-with-cpu=X86KvmCPU \
--cpu-type=X86O3CPU \
--ruby \
--network=garnet \
--topology=Mesh_XY_Oneflit \
--link-width-bits=128 \
--mesh-rows=4 \
--router-latency=3 \
--vcs-per-vnet=1 \
--routing-algorithm=0 \
--cfc=1 \
--fastpass=0 \
--upp=0 \
--l1d_size=512kB \
--l1d_assoc=8 \
--num-l2caches=16 \
--l2_size=16MB \
--num-dirs=16 \
--mem-size=8GB \
--mem-type=DDR4_2400_4x16 \
--script=./scripts/run_parsec_fs.sh
# --l0d_size=32kB \
# --l0i_size=32kB \
# --l0d_assoc=8 \
# --l0i_assoc=8 \
