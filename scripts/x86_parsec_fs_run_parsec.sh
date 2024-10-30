./build/X86/gem5.opt \
-d m5out/16core/test1 \
configs/deprecated/example/fs.py \
--num-cpus=16 \
--kernel=x86-linux-kernel-5.4.49 \
--disk=x86-parsec \
--script=./scripts/run_parsec_fs.sh \
--cpu-type=X86O3CPU \
--ruby \
--network=garnet \
--topology=Mesh_XY \
--mesh-rows=4 \
--l1d_size=512kB \
--l1d_assoc=8 \
--num-l2caches=16 \
--l2_size=16MB \
--num-dirs=16 \
--mem-size=4GB \
--mem-type=DDR4_2400_4x16
# --l0d_size=32kB \
# --l0i_size=32kB \
# --l0d_assoc=8 \
# --l0i_assoc=8 \
