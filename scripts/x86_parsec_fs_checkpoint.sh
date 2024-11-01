export M5_PATH=/gem5/chiplet-flow-ctrl/fs
./build/X86/gem5.opt \
-d m5out/checkpoints/16kvmCores_v2/ \
configs/deprecated/example/fs.py \
--num-cpus=16 \
--kernel=x86-linux-kernel-5.4.49 \
--disk=x86-parsec \
--mem-size=8GB \
--cpu-type=X86KvmCPU \
--script=configs/boot/hack_back_ckpt.rcS
# --ruby \
# --network=garnet \
# --num-cpus=4 \
# --num-dirs=4 \
# --num-l2caches=4 \
# --topology=Mesh_XY \
# --mesh-rows=2
