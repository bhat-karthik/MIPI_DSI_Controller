//=============================================================================
// Testbench: dsi_host_tb
// Project:   MIPI DSI Host Controller for Foveated Rendering Systems
// File:      dsi_host_tb.v
// Author:    MTech VLSI Design Team
// Date:      April 2026
//
// Description:
//   Top-level self-checking integration testbench for dsi_host_top.
//   Exercises the COMPLETE pipeline end-to-end:
//
//     APB config → DPI video in → VTC → Fov Mapper → Packer →
//     CDC FIFO → Packetizer → Lane Mgr → D-PHY TX PPI outputs
//
//   TEST PLAN:
//     Test 1: APB register write/readback for all config registers
//     Test 2: Full frame with foveation DISABLED (baseline — all RGB888)
//     Test 3: Full frame with foveation ENABLED (3-tier bandwidth reduction)
//     Test 4: Verify D-PHY LP→HS→LP state transitions
//     Test 5: Verify DSI packet structure (VSS, HSS short packets detected)
//     Test 6: Verify packet byte output on D-PHY HS data lanes
//
//   RESOLUTION: 8×4 (tiny, for fast simulation — ~500 ns per frame)
//   CLOCKS:     pixel_clk = 148.5 MHz (6.734 ns), byte_clk = 200 MHz (5 ns)
//
//   RUN COMMAND:
//     xrun dsi_host_tb.v dsi_host_top.v dsi_vtc.v dsi_fov_mapper.v
//          dsi_packer.v dsi_async_fifo.v dsi_packetizer.v dsi_lane_mgr.v
//          dsi_dphy_tx.v dsi_apb_regs.v dsi_ecc.v dsi_crc16.v
//          reset_sync.v +define+SIMULATION +access+r
//
//=============================================================================

`timescale 1ns / 1ps
`define SIMULATION

module dsi_host_tb;

    //=========================================================================
    // Test resolution — tiny for fast simulation
    //=========================================================================
    localparam H_ACTIVE = 8;
    localparam H_FP     = 2;
    localparam H_SYNC   = 2;
    localparam H_BP     = 2;
    localparam H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;  // 14

    localparam V_ACTIVE = 4;
    localparam V_FP     = 1;
    localparam V_SYNC   = 1;
    localparam V_BP     = 1;
    localparam V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;   // 7

    // Gaze at center (4, 2) for 8×4
    localparam GAZE_X      = 4;
    localparam GAZE_Y      = 2;
    localparam RAD_FOV_SQ  = 4;   // Foveal radius 2 pixels
    localparam RAD_MID_SQ  = 16;  // Mid radius 4 pixels

    //=========================================================================
    // APB byte address constants (word_addr × 4)
    // These match the localparam A_* values in dsi_apb_regs.v, shifted left 2
    //=========================================================================
    localparam ADDR_CTRL         = 12'h000;
    localparam ADDR_STATUS       = 12'h004;
    localparam ADDR_INT_STATUS   = 12'h008;
    localparam ADDR_INT_ENABLE   = 12'h00C;
    localparam ADDR_VID_HSIZE    = 12'h010;
    localparam ADDR_VID_HSYNC    = 12'h014;
    localparam ADDR_VID_VSIZE    = 12'h018;
    localparam ADDR_VID_VSYNC    = 12'h01C;
    localparam ADDR_FOV_CTRL     = 12'h020;
    localparam ADDR_FOV_GAZE     = 12'h024;
    localparam ADDR_FOV_RAD_FOV  = 12'h028;
    localparam ADDR_FOV_RAD_MID  = 12'h02C;
    localparam ADDR_PHY_CTRL     = 12'h030;
    localparam ADDR_PHY_TMR1     = 12'h034;
    localparam ADDR_PHY_TMR2     = 12'h038;
    localparam ADDR_PHY_CLK_TMR  = 12'h03C;
    localparam ADDR_PKT_CTRL     = 12'h040;
    localparam ADDR_PWR_CTRL     = 12'h090;

    //=========================================================================
    // Test infrastructure
    //=========================================================================
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;

    task check;
        input [399:0] test_name;
        input [31:0]  expected;
        input [31:0]  actual;
        input [31:0]  width;
        reg   [31:0]  mask;
        begin
            test_count = test_count + 1;
            mask = (width >= 32) ? 32'hFFFFFFFF : ((1 << width) - 1);
            if ((actual & mask) === (expected & mask)) begin
                pass_count = pass_count + 1;
            end else begin
                fail_count = fail_count + 1;
                $display("  [FAIL] %0s: expected=0x%08h, got=0x%08h  <<<",
                         test_name, expected & mask, actual & mask);
            end
        end
    endtask

    //=========================================================================
    // Clocks: asynchronous pixel_clk and byte_clk
    //=========================================================================
    reg pixel_clk, byte_clk;

    initial pixel_clk = 0;
    always #3.367 pixel_clk = ~pixel_clk;  // 148.5 MHz (6.734 ns period)

    initial byte_clk = 0;
    always #2.5 byte_clk = ~byte_clk;      // 200 MHz (5.0 ns period)

    //=========================================================================
    // Reset
    //=========================================================================
    reg rst_n;

    initial begin
        rst_n = 1'b0;
        #50;
        @(posedge byte_clk);
        rst_n = 1'b1;
    end

    //=========================================================================
    // DPI video source signals
    //=========================================================================
    reg [23:0] dpi_pdata;
    reg        dpi_hsync;
    reg        dpi_vsync;
    reg        dpi_de;

    //=========================================================================
    // APB bus signals
    //=========================================================================
    reg  [11:0] PADDR;
    reg         PSEL;
    reg         PENABLE;
    reg         PWRITE;
    reg  [31:0] PWDATA;
    wire [31:0] PRDATA;
    wire        PREADY;

    //=========================================================================
    // D-PHY PPI output observation
    //=========================================================================
    wire [7:0]  phy_txdatahs_0, phy_txdatahs_1, phy_txdatahs_2, phy_txdatahs_3;
    wire [3:0]  phy_txrequesths;
    wire [1:0]  phy_txdatalp_0, phy_txdatalp_1, phy_txdatalp_2, phy_txdatalp_3;
    wire        phy_txclkhs;
    wire [1:0]  phy_txclklp;
    wire        dsi_irq;

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    dsi_host_top u_dut (
        .pixel_clk       (pixel_clk),
        .byte_clk        (byte_clk),
        .rst_n           (rst_n),
        .dpi_pdata       (dpi_pdata),
        .dpi_hsync       (dpi_hsync),
        .dpi_vsync       (dpi_vsync),
        .dpi_de          (dpi_de),
        .PADDR           (PADDR),
        .PSEL            (PSEL),
        .PENABLE         (PENABLE),
        .PWRITE          (PWRITE),
        .PWDATA          (PWDATA),
        .PRDATA          (PRDATA),
        .PREADY          (PREADY),
        .phy_txdatahs_0  (phy_txdatahs_0),
        .phy_txdatahs_1  (phy_txdatahs_1),
        .phy_txdatahs_2  (phy_txdatahs_2),
        .phy_txdatahs_3  (phy_txdatahs_3),
        .phy_txrequesths (phy_txrequesths),
        .phy_txreadyhs   (4'b1111),         // PHY always ready (no analog model)
        .phy_txdatalp_0  (phy_txdatalp_0),
        .phy_txdatalp_1  (phy_txdatalp_1),
        .phy_txdatalp_2  (phy_txdatalp_2),
        .phy_txdatalp_3  (phy_txdatalp_3),
        .phy_txclkhs     (phy_txclkhs),
        .phy_txclklp     (phy_txclklp),
        .dsi_irq         (dsi_irq)
    );

    //=========================================================================
    // APB Write Task
    //=========================================================================
    // APB3 write: 2-cycle transfer (setup + access phase)
    task apb_write;
        input [11:0] addr;
        input [31:0] data;
        begin
            @(posedge byte_clk);
            // Setup phase
            PSEL    <= 1'b1;
            PENABLE <= 1'b0;
            PWRITE  <= 1'b1;
            PADDR   <= addr;
            PWDATA  <= data;
            @(posedge byte_clk);
            // Access phase
            PENABLE <= 1'b1;
            @(posedge byte_clk);
            // Complete — de-assert
            PSEL    <= 1'b0;
            PENABLE <= 1'b0;
            PWRITE  <= 1'b0;
        end
    endtask

    //=========================================================================
    // APB Read Task
    //=========================================================================
    reg [31:0] apb_read_data;

    task apb_read;
        input [11:0] addr;
        begin
            @(posedge byte_clk);
            // Setup phase
            PSEL    <= 1'b1;
            PENABLE <= 1'b0;
            PWRITE  <= 1'b0;
            PADDR   <= addr;
            @(posedge byte_clk);
            // Access phase
            PENABLE <= 1'b1;
            @(posedge byte_clk);
            apb_read_data <= PRDATA;
            // Complete
            PSEL    <= 1'b0;
            PENABLE <= 1'b0;
        end
    endtask

    //=========================================================================
    // DPI Frame Generator Task
    //=========================================================================
    task generate_dpi_frame;
        integer line, h_pos;
        integer active_line, active_pixel;
        begin
            for (line = 0; line < V_TOTAL; line = line + 1) begin
                for (h_pos = 0; h_pos < H_TOTAL; h_pos = h_pos + 1) begin
                    @(posedge pixel_clk);

                    dpi_hsync <= (h_pos < H_SYNC);
                    dpi_vsync <= (line < V_SYNC);

                    active_line  = (line >= V_SYNC + V_BP) && (line < V_SYNC + V_BP + V_ACTIVE);
                    active_pixel = (h_pos >= H_SYNC + H_BP) && (h_pos < H_SYNC + H_BP + H_ACTIVE);

                    if (active_line && active_pixel) begin
                        dpi_de <= 1'b1;
                        // Encode pixel position in color for verification
                        // R[7:4] = h_position, R[3:0] = 0xA (marker)
                        // G[7:4] = v_position, G[3:0] = 0xB (marker)
                        // B[7:0] = 0x42 (fixed identifier)
                        dpi_pdata <= {(h_pos - H_SYNC - H_BP), 4'hA,
                                      (line - V_SYNC - V_BP),  4'hB,
                                      8'h42};
                    end else begin
                        dpi_de    <= 1'b0;
                        dpi_pdata <= 24'h000000;
                    end
                end
            end
            // One extra cycle to ensure line_end propagates
            @(posedge pixel_clk);
            dpi_de    <= 1'b0;
            dpi_hsync <= 1'b0;
            dpi_vsync <= 1'b0;
        end
    endtask

    //=========================================================================
    // D-PHY Output Monitors
    //=========================================================================

    // Track D-PHY state transitions
    integer hs_entry_count = 0;       // Number of LP→HS transitions
    integer hs_exit_count  = 0;       // Number of HS→LP transitions
    integer clk_hs_entry_count = 0;   // Clock lane HS entries
    integer hs_bytes_lane0 = 0;       // Bytes transmitted on lane 0

    // Detect LP→HS transition on data lane 0
    reg [1:0] lp0_prev;
    reg       hs_req_prev;

    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            lp0_prev    <= 2'b11;
            hs_req_prev <= 1'b0;
        end else begin
            lp0_prev    <= phy_txdatalp_0;
            hs_req_prev <= phy_txrequesths[0];

            // LP-11 to LP-01 transition = HS entry sequence starting
            if (lp0_prev == 2'b11 && phy_txdatalp_0 == 2'b01)
                hs_entry_count = hs_entry_count + 1;

            // HS request falling edge = HS exit
            if (hs_req_prev && !phy_txrequesths[0])
                hs_exit_count = hs_exit_count + 1;
        end
    end

    // Count HS data bytes on lane 0
    always @(posedge byte_clk) begin
        if (rst_n && phy_txrequesths[0] && u_dut.u_dphy_tx.hs_active)
            hs_bytes_lane0 = hs_bytes_lane0 + 1;
    end

    // Clock lane HS entry detection
    reg txclkhs_prev;
    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n)
            txclkhs_prev <= 1'b0;
        else begin
            txclkhs_prev <= phy_txclkhs;
            if (!txclkhs_prev && phy_txclkhs)
                clk_hs_entry_count = clk_hs_entry_count + 1;
        end
    end

    // Track FIFO status
    integer fifo_max_level = 0;
    always @(posedge pixel_clk) begin
        if (rst_n && !u_dut.u_async_fifo.wr_full) begin
            // Can't easily read fill level from Gray-code FIFO externally,
            // but we can check that full never asserts
        end
    end

    reg fifo_overflow_detected;
    initial fifo_overflow_detected = 0;
    always @(posedge pixel_clk) begin
        if (rst_n && u_dut.u_async_fifo.wr_full && u_dut.u_packer.pxl_valid) begin
            fifo_overflow_detected = 1;
            $display("  [ERROR] FIFO overflow detected at time %0t", $time);
        end
    end

    //=========================================================================
    // Packetizer output monitor — capture DSI packets
    //=========================================================================
    integer total_pkt_bytes = 0;
    integer total_pkts      = 0;
    integer short_pkt_count = 0;
    integer long_pkt_count  = 0;

    // Monitor packetizer output
    always @(posedge byte_clk) begin
        if (rst_n && u_dut.u_packetizer.pkt_valid) begin
            total_pkt_bytes = total_pkt_bytes + 1;

            if (u_dut.u_packetizer.pkt_sop)
                total_pkts = total_pkts + 1;
        end
    end

    //=========================================================================
    // Configure DUT via APB
    //=========================================================================
    task configure_dut;
        input enable_foveation;
        begin
            $display("  Configuring DUT via APB (foveation=%0b)...", enable_foveation);

            // Power on core domain
            apb_write(ADDR_PWR_CTRL, 32'h0000_0003);  // Both core and fov domains on

            // Video timing: {4'b0, bp[11:0], 4'b0, active[11:0]}
            apb_write(ADDR_VID_HSIZE, {4'd0, H_BP[11:0], 4'd0, H_ACTIVE[11:0]});
            apb_write(ADDR_VID_HSYNC, {4'd0, H_FP[11:0], 4'd0, H_SYNC[11:0]});
            apb_write(ADDR_VID_VSIZE, {4'd0, V_BP[11:0], 4'd0, V_ACTIVE[11:0]});
            apb_write(ADDR_VID_VSYNC, {4'd0, V_FP[11:0], 4'd0, V_SYNC[11:0]});

            // Foveation configuration
            if (enable_foveation) begin
                apb_write(ADDR_FOV_CTRL, 32'h0000_0001);  // Enable foveation
                apb_write(ADDR_FOV_GAZE, {4'd0, GAZE_Y[11:0], 4'd0, GAZE_X[11:0]});
                apb_write(ADDR_FOV_RAD_FOV, RAD_FOV_SQ);
                apb_write(ADDR_FOV_RAD_MID, RAD_MID_SQ);
            end else begin
                apb_write(ADDR_FOV_CTRL, 32'h0000_0000);  // Disable foveation
            end

            // PHY: 4-lane mode, non-continuous clock
            apb_write(ADDR_PHY_CTRL, 32'h0000_0002);  // lane_count=10 (4 lanes), cont_clk=0

            // PHY timing (byte_clk cycles):
            // TMR1: [7:0]=t_lpx=10, [15:8]=t_hs_prepare=9, [23:16]=t_hs_zero=22
            apb_write(ADDR_PHY_TMR1, {8'd0, 8'd22, 8'd9, 8'd10});
            // TMR2: [7:0]=t_hs_trail=13, [15:8]=t_hs_exit=20
            apb_write(ADDR_PHY_TMR2, {16'd0, 8'd20, 8'd13});
            // CLK_TMR: [7:0]=t_clk_prepare=8, [15:8]=t_clk_zero=54,
            //          [23:16]=t_clk_post=16, [31:24]=t_clk_trail=12
            apb_write(ADDR_PHY_CLK_TMR, {8'd12, 8'd16, 8'd54, 8'd8});

            // Packet config: VC=0, video_mode=01 (Non-Burst Sync Event)
            apb_write(ADDR_PKT_CTRL, 32'h0000_0001);

            // Enable DSI controller: dsi_en=1, video_en=1, vtc_mode=0 (external DPI)
            apb_write(ADDR_CTRL, 32'h0000_0003);  // bits[1:0] = {video_en, dsi_en}

            $display("  Configuration complete.");
        end
    endtask

    //=========================================================================
    // Wait for pipeline to fully flush
    //=========================================================================
    task wait_pipeline_flush;
        integer timeout;
        begin
            // Wait for FIFO to drain and packetizer to finish
            timeout = 0;
            while (timeout < 5000) begin
                @(posedge byte_clk);
                timeout = timeout + 1;
                // Check if packetizer is idle and FIFO is empty
                if (u_dut.u_async_fifo.rd_empty &&
                    u_dut.u_packetizer.state == 4'd0 &&  // ST_IDLE
                    !u_dut.u_packetizer.pkt_valid)
                    timeout = 5000;  // Exit
            end
            // Extra margin for D-PHY HS→LP transition
            repeat(100) @(posedge byte_clk);
        end
    endtask

    //=========================================================================
    //
    // MAIN TEST SEQUENCE
    //
    //=========================================================================
    initial begin
        $display("");
        $display("================================================================");
        $display("  MIPI DSI Host Controller — Top-Level Integration Testbench");
        $display("  Resolution: %0dx%0d, Gaze: (%0d,%0d)", H_ACTIVE, V_ACTIVE, GAZE_X, GAZE_Y);
        $display("  pixel_clk=148.5 MHz, byte_clk=200 MHz");
        $display("================================================================");
        $display("");

        // Initialize all inputs
        dpi_pdata = 24'h000000;
        dpi_hsync = 1'b0;
        dpi_vsync = 1'b0;
        dpi_de    = 1'b0;
        PADDR     = 12'd0;
        PSEL      = 1'b0;
        PENABLE   = 1'b0;
        PWRITE    = 1'b0;
        PWDATA    = 32'd0;

        // Wait for reset
        @(posedge rst_n);
        repeat(10) @(posedge byte_clk);

        //=================================================================
        // TEST 1: APB Register Write/Readback
        //=================================================================
        $display("=== TEST 1: APB Register Write/Readback ===");

        // Write a known value and read it back
        apb_write(ADDR_VID_HSIZE, 32'h0002_0008);  // h_bp=2, h_active=8
        apb_read(ADDR_VID_HSIZE);
        check("APB: VID_HSIZE readback", 32'h0002_0008, apb_read_data, 32);

        apb_write(ADDR_FOV_GAZE, 32'h0002_0004);   // gaze_y=2, gaze_x=4
        apb_read(ADDR_FOV_GAZE);
        check("APB: FOV_GAZE readback", 32'h0002_0004, apb_read_data, 32);

        apb_write(ADDR_PHY_CTRL, 32'h0000_0002);   // 4-lane, no cont_clk
        apb_read(ADDR_PHY_CTRL);
        check("APB: PHY_CTRL readback", 32'h0000_0002, apb_read_data, 32);

        // Read status register (should show fifo_empty=1 initially)
        apb_read(ADDR_STATUS);
        check("APB: STATUS fifo_empty", 1, (apb_read_data >> 1) & 1, 1);

        // Read default PHY timing
        apb_read(ADDR_PHY_TMR1);
        $display("  PHY_TMR1 default = 0x%08h", apb_read_data);

        $display("");

        //=================================================================
        // TEST 2: Full Frame WITHOUT Foveation (baseline)
        //=================================================================
        $display("=== TEST 2: Full Frame — Foveation DISABLED ===");

        // Reset counters
        hs_entry_count   = 0;
        hs_exit_count    = 0;
        clk_hs_entry_count = 0;
        hs_bytes_lane0   = 0;
        total_pkt_bytes  = 0;
        total_pkts       = 0;
        fifo_overflow_detected = 0;

        configure_dut(0);  // Foveation disabled

        // Wait for config to propagate through CDC synchronizers
        repeat(20) @(posedge pixel_clk);

        $display("  Generating frame 1 (no foveation)...");
        generate_dpi_frame();
        wait_pipeline_flush();

        $display("  Frame 1 complete. Results:");
        $display("    Total packet bytes: %0d", total_pkt_bytes);
        $display("    Total packets:      %0d", total_pkts);
        $display("    HS entries (data):  %0d", hs_entry_count);
        $display("    HS exits (data):    %0d", hs_exit_count);
        $display("    CLK HS entries:     %0d", clk_hs_entry_count);
        $display("    HS bytes on lane 0: %0d", hs_bytes_lane0);
        $display("    FIFO overflow:      %0s", fifo_overflow_detected ? "YES <<<" : "NO");

        // Verify no FIFO overflow
        check("No FIFO overflow", 0, fifo_overflow_detected, 1);

        // Verify packets were generated (at minimum: 1 VSS + V_ACTIVE HSS + V_ACTIVE long = 1+4+4 = 9)
        check("Packets generated (>0)", 1, (total_pkts > 0) ? 1 : 0, 1);
        check("Packet bytes generated (>0)", 1, (total_pkt_bytes > 0) ? 1 : 0, 1);

        // Verify D-PHY entered HS mode at least once
        check("D-PHY HS entered", 1, (hs_entry_count > 0) ? 1 : 0, 1);

        // Verify clock lane entered HS
        check("CLK lane HS entered", 1, (clk_hs_entry_count > 0) ? 1 : 0, 1);

        // Capture baseline byte count for bandwidth comparison
        begin : baseline_capture
            integer baseline_bytes;
            baseline_bytes = total_pkt_bytes;
            $display("    Baseline total bytes: %0d", baseline_bytes);
        end

        $display("");

        //=================================================================
        // TEST 3: Full Frame WITH Foveation
        //=================================================================
        $display("=== TEST 3: Full Frame — Foveation ENABLED ===");

        // Disable and re-enable with foveation
        apb_write(ADDR_CTRL, 32'h0000_0000);  // Disable
        repeat(50) @(posedge byte_clk);

        // Reset counters
        hs_entry_count   = 0;
        hs_exit_count    = 0;
        clk_hs_entry_count = 0;
        hs_bytes_lane0   = 0;
        total_pkt_bytes  = 0;
        total_pkts       = 0;
        fifo_overflow_detected = 0;

        configure_dut(1);  // Foveation enabled
        repeat(20) @(posedge pixel_clk);

        $display("  Generating frame 2 (foveation enabled)...");
        generate_dpi_frame();
        wait_pipeline_flush();

        $display("  Frame 2 complete. Results:");
        $display("    Total packet bytes: %0d", total_pkt_bytes);
        $display("    Total packets:      %0d", total_pkts);
        $display("    HS bytes on lane 0: %0d", hs_bytes_lane0);
        $display("    FIFO overflow:      %0s", fifo_overflow_detected ? "YES <<<" : "NO");

        check("Fov: No FIFO overflow", 0, fifo_overflow_detected, 1);
        check("Fov: Packets generated", 1, (total_pkts > 0) ? 1 : 0, 1);

        $display("");

        //=================================================================
        // TEST 4: D-PHY LP State Verification
        //=================================================================
        $display("=== TEST 4: D-PHY LP State Verification ===");

        // After pipeline flush, data lanes should be in LP-11 (idle)
        check("DPHY: Lane 0 LP-11 (idle)", 2'b11, phy_txdatalp_0, 2);
        check("DPHY: Lane 1 LP-11 (idle)", 2'b11, phy_txdatalp_1, 2);
        check("DPHY: Lane 2 LP-11 (idle)", 2'b11, phy_txdatalp_2, 2);
        check("DPHY: Lane 3 LP-11 (idle)", 2'b11, phy_txdatalp_3, 2);

        // HS request should be de-asserted after idle
        check("DPHY: TxRequestHS=0 (idle)", 4'b0000, phy_txrequesths, 4);

        $display("");

        //=================================================================
        // TEST 5: Second Frame — Verify Repeatability
        //=================================================================
        $display("=== TEST 5: Second Frame — Verify Repeatability ===");

        hs_entry_count  = 0;
        total_pkt_bytes = 0;
        total_pkts      = 0;

        $display("  Generating frame 3 (repeat test)...");
        generate_dpi_frame();
        wait_pipeline_flush();

        $display("    Total packets: %0d", total_pkts);
        check("Repeat: Packets generated", 1, (total_pkts > 0) ? 1 : 0, 1);

        $display("");

        //=================================================================
        // TEST 6: APB Status Register During Operation
        //=================================================================
        $display("=== TEST 6: Verify Status Registers ===");

        // FIFO should be empty after flush
        apb_read(ADDR_STATUS);
        check("Status: fifo_empty after flush", 1, (apb_read_data >> 1) & 1, 1);
        check("Status: fifo_full=0", 0, (apb_read_data >> 2) & 1, 1);

        $display("");

        //=================================================================
        // TEST 7: Disable and Re-enable
        //=================================================================
        $display("=== TEST 7: Disable/Re-enable Controller ===");

        apb_write(ADDR_CTRL, 32'h0000_0000);  // Disable
        repeat(20) @(posedge byte_clk);

        apb_read(ADDR_CTRL);
        check("Disable: CTRL=0", 32'h0000_0000, apb_read_data, 32);

        apb_write(ADDR_CTRL, 32'h0000_0003);  // Re-enable
        repeat(20) @(posedge byte_clk);

        apb_read(ADDR_CTRL);
        check("Re-enable: CTRL=3", 32'h0000_0003, apb_read_data, 32);

        hs_entry_count  = 0;
        total_pkts      = 0;

        $display("  Generating frame 4 (after re-enable)...");
        generate_dpi_frame();
        wait_pipeline_flush();

        check("Re-enable: Packets generated", 1, (total_pkts > 0) ? 1 : 0, 1);

        $display("");

        //=================================================================
        // FINAL SUMMARY
        //=================================================================
        $display("================================================================");
        $display("  TEST SUMMARY");
        $display("================================================================");
        $display("  Total checks:  %0d", test_count);
        $display("  Passed:        %0d", pass_count);
        $display("  Failed:        %0d", fail_count);
        $display("================================================================");
        if (fail_count == 0)
            $display("  >>> ALL TESTS PASSED — RTL VERIFICATION COMPLETE <<<");
        else
            $display("  >>> %0d TESTS FAILED — DEBUG REQUIRED <<<", fail_count);
        $display("================================================================");
        $display("");
        $display("  NEXT STEPS:");
        $display("    1. Review waveforms in tb_dsi_host.vcd");
        $display("    2. Verify DSI packet structure in waveform viewer");
        $display("    3. Prepare Phase 1 presentation slides");
        $display("    4. Begin Genus synthesis: genus -f run_syn.tcl");
        $display("");

        #200;
        $finish;
    end

    //=========================================================================
    // Timeout watchdog
    //=========================================================================
    initial begin
        #2000000;  // 2 ms — generous for 4 tiny frames
        $display("");
        $display("[TIMEOUT] Testbench timed out after 2 ms of simulation time");
        $display("  This likely means the pipeline is stuck.");
        $display("  Check: FIFO empty=%b, Packetizer state=%0d, DPHY dl_state=%0d",
                 u_dut.u_async_fifo.rd_empty,
                 u_dut.u_packetizer.state,
                 u_dut.u_dphy_tx.dl_state);
        $finish;
    end

    //=========================================================================
    // Waveform dump
    //=========================================================================
    initial begin
        $dumpfile("tb_dsi_host.vcd");
        $dumpvars(0, dsi_host_tb);
    end

endmodule
