# Re-synthesize after SPAR patch insertion
set DESIGN   "your_design"
set LIB_DB   "saed32hvt_tt1p05v25c.db"

set_app_var target_library $LIB_DB
set_app_var link_library   [list "*" $LIB_DB]

# Read original scan netlist + SPAR patch
read_verilog [list "netlist/scan_netlist.v" \
                   "netlist/spar_patch.v"]
current_design $DESIGN
link

# TPEnable is a test-mode input — tie to 0 for functional timing
set_case_analysis 0 [get_ports TPEnable]

# Re-compile (preserve scan structure)
compile_ultra -incremental -scan

report_area         > "reports/area_spar.rpt"
report_timing       > "reports/timing_spar.rpt"
report_constraint   > "reports/constraints_spar.rpt"

write -format verilog -hier -out "netlist/spar_netlist.v"
write_sdc "netlist/spar_netlist.sdc"