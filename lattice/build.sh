mkdir -p _build
bender script flist-plus -t fpga -t rtl --define SYNTHESIS=1 -t ecp5 > _build/croc.flist
yosys -s run_yosys.ys

nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json _build/hardware.json --textcfg _build/hardware.config --report _build/hardware.pnr --lpf constraints/iCESugar-Pro.lpf

ecppack --svf _build/hardware.svf _build/hardware.config _build/hardware.bit
