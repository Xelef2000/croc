#!/bin/bash
FILES=()
INCLUDES=()
DEFINES=()
while IFS= read -r line || [[ -n "$line" ]]; do
  [[ -z "$line" || "$line" =~ ^[[:space:]]*# ]] && continue
  if [[ "$line" == +incdir+* ]]; then
    INCLUDES+=("-I${line#'+incdir+'}")
  elif [[ "$line" == +define+* ]]; then
    DEFINES+=("-D${line#'+define+'}")
  else
    FILES+=("$line")
  fi
done < croc.flist

# Sort files into Verilog and SystemVerilog based on extension
VERILOG_FILES=()
SV_FILES=()
for file in "${FILES[@]}"; do
  if [[ "$file" =~ \.(sv|svh)$ ]]; then
    SV_FILES+=("$file")
  else
    VERILOG_FILES+=("$file")
  fi
done

# Construct command for common options (includes and defines)
COMMON_OPTIONS=""
for inc in "${INCLUDES[@]}"; do
  COMMON_OPTIONS+=" $inc"
done
for def in "${DEFINES[@]}"; do
  COMMON_OPTIONS+=" $def"
done

# Construct yosys script with separate read commands for Verilog and SystemVerilog
YOSYS_SCRIPT=""

# Add SystemVerilog files with -sv flag
if [ ${#SV_FILES[@]} -gt 0 ]; then
  YOSYS_SCRIPT+="read_verilog -sv$COMMON_OPTIONS"
  for file in "${SV_FILES[@]}"; do
    YOSYS_SCRIPT+=" $file"
  done
  YOSYS_SCRIPT+="; "
fi

# Add regular Verilog files
if [ ${#VERILOG_FILES[@]} -gt 0 ]; then
  YOSYS_SCRIPT+="read_verilog$COMMON_OPTIONS"
  for file in "${VERILOG_FILES[@]}"; do
    YOSYS_SCRIPT+=" $file"
  done
  YOSYS_SCRIPT+="; "
fi

# Add synthesis command
YOSYS_SCRIPT+="synth_ecp5 -top top -json _build/hardware.json"

# Run Yosys with the composed script
yosys -q -p "$YOSYS_SCRIPT"