change file directory in []

baseline rtl simulation instruction:
1. cd ./Src/Baseline/
2. ncverilog [Final_tb] ./rtl/CHIP.v [slow_memory] +define+[noHazard/hasHazard] +access+r 

baseline gate level simulation instruction:
1. cd ./Src/Baseline/
2. change `define SDFFILE to "./syn/CHIP_syn.sdf"
3. ncverilog [Final_tb] ./syn/CHIP_syn.v [slow_memory] [tsmc13] +define+[noHazard/hasHazard] +define+SDF +access+r 

extension rtl simulation instruction: 
1. cd ./Src/Extension/
2. ncverilog [tb] [slow_memory] -f files_[BrPred/L2Cache/MultDiv].f +define+[testbed]

extension gate level simulation instruction:
1. cd ./Src/Extension/
2. change `define SDFFILE to "./[BrPred/L2Cache/MultDiv]/syn/CHIP_syn.sdf"
3. ncverilog [tb] ./[BrPred/L2Cache/MultDiv]/syn/CHIP_syn.v [slow_memory] [tsmc13] +define+[testbed] +define+SDF