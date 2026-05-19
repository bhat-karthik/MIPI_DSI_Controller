# ── run_syn_pass1.tcl ──
set_db init_lib_search_path ./lib
set_db init_hdl_search_path ./rtl

# Read liberty (slow corner for setup)
read_libs slow.lib

# Read all RTL files
read_hdl -v2001 {
    reset_sync.v dsi_ecc.v dsi_crc16.v dsi_vtc.v
    dsi_fov_mapper.v dsi_packer.v dsi_async_fifo.v
    dsi_packetizer.v dsi_lane_mgr.v dsi_dphy_tx.v
    dsi_apb_regs.v dsi_host_top.v
}

# Elaborate top module
elaborate dsi_host_top

# Read SDC constraints
read_sdc dsi_host_constraints.sdc

# Synthesize
set_db syn_generic_effort medium
syn_generic
syn_map
syn_opt

# Reports
report_timing > reports/timing_pass1.rpt
report_area   > reports/area_pass1.rpt
report_power  > reports/power_pass1.rpt

# Write outputs
write_hdl > outputs/dsi_host_top_synth.v
write_sdc > outputs/dsi_host_top_synth.sdc
write_sdf > outputs/dsi_host_top_synth.sdf