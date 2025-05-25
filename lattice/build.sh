bender script flist-plus -t fpga -t rtl --define SYNTHESIS=1 -t ecp5 > croc.flist
mkdir -p _build
yosys -s run_yosys.ys