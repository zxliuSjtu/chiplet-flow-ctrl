cd ..; cd /home/gem5/parsec-benchmark; source env.sh; echo " ZXLIU source done "; parsecmgmt -a run -p fluidanimate  -c gcc-hooks -i simsmall  -n 16; sleep 5; m5 exit;
