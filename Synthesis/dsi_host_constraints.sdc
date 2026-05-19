###############################################################################
# File:    dsi_host_constraints.sdc
# Project: MIPI DSI Host Controller for Foveated Rendering Systems
# Library: GSCL 45nm (gsclib045)
# Target:  200 MHz byte_clk, 148.5 MHz pixel_clk
###############################################################################

###############################################################################
# 1. Clock Definitions
###############################################################################

# pixel_clk: 148.5 MHz, 6.734 ns period
create_clock -name pixel_clk -period 6.734 [get_ports pixel_clk]

# byte_clk: 200 MHz, 5.0 ns period
create_clock -name byte_clk -period 5.0 [get_ports byte_clk]

# Clock uncertainty (jitter + OCV + skew margin, pre-CTS)
set_clock_uncertainty -setup 0.3 [get_clocks pixel_clk]
set_clock_uncertainty -setup 0.3 [get_clocks byte_clk]
set_clock_uncertainty -hold  0.1 [get_clocks pixel_clk]
set_clock_uncertainty -hold  0.1 [get_clocks byte_clk]

# Clock transition
set_clock_transition 0.15 [get_clocks pixel_clk]
set_clock_transition 0.15 [get_clocks byte_clk]

###############################################################################
# 2. CDC False Paths (async FIFO Gray-code synchronizers)
###############################################################################

set_false_path -from [get_clocks pixel_clk] -to [get_clocks byte_clk]
set_false_path -from [get_clocks byte_clk]  -to [get_clocks pixel_clk]

###############################################################################
# 3. Reset — Asynchronous
###############################################################################

set_false_path -from [get_ports rst_n]

###############################################################################
# 4. Input Delays
###############################################################################

# DPI video inputs (pixel_clk domain)
set_input_delay -clock pixel_clk -max 2.0 [get_ports {dpi_pdata[*] dpi_hsync dpi_vsync dpi_de}]
set_input_delay -clock pixel_clk -min 0.5 [get_ports {dpi_pdata[*] dpi_hsync dpi_vsync dpi_de}]

# APB bus (byte_clk domain)
set_input_delay -clock byte_clk -max 2.0 [get_ports {PADDR[*] PSEL PENABLE PWRITE PWDATA[*]}]
set_input_delay -clock byte_clk -min 0.5 [get_ports {PADDR[*] PSEL PENABLE PWRITE PWDATA[*]}]

# D-PHY ready
set_input_delay -clock byte_clk -max 2.0 [get_ports {phy_txreadyhs[*]}]
set_input_delay -clock byte_clk -min 0.5 [get_ports {phy_txreadyhs[*]}]

###############################################################################
# 5. Output Delays
###############################################################################

# APB read data
set_output_delay -clock byte_clk -max 2.0 [get_ports {PRDATA[*] PREADY}]
set_output_delay -clock byte_clk -min 0.5 [get_ports {PRDATA[*] PREADY}]

# D-PHY PPI data outputs
set_output_delay -clock byte_clk -max 1.5 [get_ports {phy_txdatahs_0[*] phy_txdatahs_1[*] phy_txdatahs_2[*] phy_txdatahs_3[*]}]
set_output_delay -clock byte_clk -min 0.5 [get_ports {phy_txdatahs_0[*] phy_txdatahs_1[*] phy_txdatahs_2[*] phy_txdatahs_3[*]}]

# D-PHY PPI control outputs
set_output_delay -clock byte_clk -max 1.5 [get_ports {phy_txrequesths[*] phy_txdatalp_0[*] phy_txdatalp_1[*] phy_txdatalp_2[*] phy_txdatalp_3[*] phy_txclkhs phy_txclklp[*]}]
set_output_delay -clock byte_clk -min 0.5 [get_ports {phy_txrequesths[*] phy_txdatalp_0[*] phy_txdatalp_1[*] phy_txdatalp_2[*] phy_txdatalp_3[*] phy_txclkhs phy_txclklp[*]}]

# Interrupt
set_output_delay -clock byte_clk -max 2.0 [get_ports dsi_irq]
set_output_delay -clock byte_clk -min 0.5 [get_ports dsi_irq]

###############################################################################
# 6. Design Rule Constraints
###############################################################################

set_max_transition 0.3 [current_design]
set_max_fanout 20 [current_design]

puts "INFO: SDC constraints loaded for GSCL045 target."
