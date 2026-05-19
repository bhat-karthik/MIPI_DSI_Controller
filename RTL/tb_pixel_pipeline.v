//=============================================================================
// Testbench: tb_pixel_pipeline
// Project:   MIPI DSI Host Controller for Foveated Rendering Systems
//
// Tests: VTC → Foveated Region Mapper → Pixel Packer (v3, per-pixel tier)
// Resolution: 16×8, Gaze: (8,4), Foveal r²=9, Mid r²=36
//
// Run:  xrun tb_pixel_pipeline.v dsi_vtc.v dsi_fov_mapper.v dsi_packer.v
//            +define+SIMULATION +access+r
//=============================================================================

`timescale 1ns / 1ps
`define SIMULATION

module tb_pixel_pipeline;

    localparam H_ACTIVE = 16, H_FP = 4, H_SYNC = 4, H_BP = 4;
    localparam H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;
    localparam V_ACTIVE = 8,  V_FP = 2, V_SYNC = 2, V_BP = 2;
    localparam V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;
    localparam GAZE_X = 8, GAZE_Y = 4, RAD_FOV_SQ = 9, RAD_MID_SQ = 36;

    integer test_count = 0, pass_count = 0, fail_count = 0;
    integer pixel_count = 0, sol_count = 0, eol_count = 0, sof_count = 0;
    integer tier0_count = 0, tier1_count = 0, tier2_count = 0;

    task check;
        input [255:0] name; input [31:0] expected, actual, width;
        reg [31:0] mask;
        begin
            test_count = test_count + 1;
            mask = (1 << width) - 1;
            if ((actual & mask) === (expected & mask)) begin
                pass_count = pass_count + 1;
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0s: expected=0x%0h, got=0x%0h  <<<",
                         name, expected & mask, actual & mask);
            end
        end
    endtask

    reg pixel_clk;
    initial pixel_clk = 0;
    always #3 pixel_clk = ~pixel_clk;

    reg rst_n;
    initial begin rst_n = 1'b0; #25; @(posedge pixel_clk); rst_n = 1'b1; end

    reg [23:0] dpi_pixel_data;
    reg        dpi_hsync, dpi_vsync, dpi_de;

    task generate_frame;
        integer line, h_pos;
        begin
            for (line = 0; line < V_TOTAL; line = line + 1) begin
                for (h_pos = 0; h_pos < H_TOTAL; h_pos = h_pos + 1) begin
                    @(posedge pixel_clk);
                    dpi_hsync <= (h_pos < H_SYNC);
                    dpi_vsync <= (line < V_SYNC);
                    if (line >= (V_SYNC+V_BP) && line < (V_SYNC+V_BP+V_ACTIVE) &&
                        h_pos >= (H_SYNC+H_BP) && h_pos < (H_SYNC+H_BP+H_ACTIVE)) begin
                        dpi_de <= 1'b1;
                        dpi_pixel_data <= {(h_pos-H_SYNC-H_BP), 4'hA,
                                           (line-V_SYNC-V_BP),  4'hB, 8'h42};
                    end else begin
                        dpi_de <= 1'b0;
                        dpi_pixel_data <= 24'h000000;
                    end
                end
            end
            @(posedge pixel_clk);
            dpi_de <= 1'b0; dpi_hsync <= 1'b0; dpi_vsync <= 1'b0;
        end
    endtask

    // ── DUT: VTC ──
    wire [23:0] vtc_pixel_data;
    wire        vtc_pixel_valid;
    wire [11:0] vtc_h_count, vtc_v_count;
    wire        vtc_line_start, vtc_line_end, vtc_frame_start, vtc_frame_end;
    wire        vtc_in_hsync, vtc_in_vsync, vtc_in_hbp, vtc_in_hfp;

    dsi_vtc u_vtc (
        .pixel_clk(pixel_clk), .rst_n(rst_n),
        .cfg_vtc_mode(1'b0), .cfg_h_active(H_ACTIVE), .cfg_h_fp(H_FP),
        .cfg_h_sync(H_SYNC), .cfg_h_bp(H_BP), .cfg_v_active(V_ACTIVE),
        .cfg_v_fp(V_FP), .cfg_v_sync(V_SYNC), .cfg_v_bp(V_BP),
        .cfg_video_en(1'b1), .pixel_data_in(dpi_pixel_data),
        .hsync_in(dpi_hsync), .vsync_in(dpi_vsync), .de_in(dpi_de),
        .pixel_data_out(vtc_pixel_data), .pixel_valid(vtc_pixel_valid),
        .h_count(vtc_h_count), .v_count(vtc_v_count),
        .line_start(vtc_line_start), .line_end(vtc_line_end),
        .frame_start(vtc_frame_start), .frame_end(vtc_frame_end),
        .in_hsync(vtc_in_hsync), .in_vsync(vtc_in_vsync),
        .in_hbp(vtc_in_hbp), .in_hfp(vtc_in_hfp)
    );

    // ── DUT: Fov Mapper ──
    wire [23:0] fov_pixel_data;
    wire        fov_pixel_valid;
    wire [1:0]  fov_tier;
    wire        fov_line_start, fov_line_end, fov_frame_start, fov_frame_end;

    dsi_fov_mapper u_fov (
        .pixel_clk(pixel_clk), .rst_n(rst_n),
        .pixel_data_in(vtc_pixel_data), .pixel_valid_in(vtc_pixel_valid),
        .h_count(vtc_h_count), .v_count(vtc_v_count),
        .line_start_in(vtc_line_start), .line_end_in(vtc_line_end),
        .frame_start_in(vtc_frame_start), .frame_end_in(vtc_frame_end),
        .cfg_gaze_x(GAZE_X), .cfg_gaze_y(GAZE_Y),
        .cfg_radius_foveal_sq(RAD_FOV_SQ), .cfg_radius_mid_sq(RAD_MID_SQ),
        .cfg_fov_enable(1'b1),
        .pixel_data_out(fov_pixel_data), .pixel_valid_out(fov_pixel_valid),
        .fov_tier(fov_tier),
        .line_start_out(fov_line_start), .line_end_out(fov_line_end),
        .frame_start_out(fov_frame_start), .frame_end_out(fov_frame_end)
    );

    // ── DUT: Packer (v3) ──
    wire [23:0] pxl_data;
    wire [1:0]  pxl_tier;
    wire        pxl_valid, pxl_sol, pxl_eol, pxl_sof;

    dsi_packer u_packer (
        .pixel_clk(pixel_clk), .rst_n(rst_n),
        .pixel_data_in(fov_pixel_data), .pixel_valid_in(fov_pixel_valid),
        .fov_tier(fov_tier),
        .line_start_in(fov_line_start), .line_end_in(fov_line_end),
        .frame_start_in(fov_frame_start), .frame_end_in(fov_frame_end),
        .cfg_h_active(H_ACTIVE),
        .pxl_data(pxl_data), .pxl_tier(pxl_tier), .pxl_valid(pxl_valid),
        .pxl_sol(pxl_sol), .pxl_eol(pxl_eol), .pxl_sof(pxl_sof)
    );

    // ── Output monitors ──
    always @(posedge pixel_clk) begin
        if (rst_n && pxl_valid) begin
            pixel_count = pixel_count + 1;
            case (pxl_tier)
                2'b00: tier0_count = tier0_count + 1;
                2'b01: tier1_count = tier1_count + 1;
                2'b10: tier2_count = tier2_count + 1;
            endcase
        end
        if (rst_n && pxl_sol) sol_count = sol_count + 1;
        if (rst_n && pxl_eol) eol_count = eol_count + 1;
        if (rst_n && pxl_sof) sof_count = sof_count + 1;
    end

    always @(posedge pixel_clk) begin
        if (pxl_sol) $display("  PACKER: SOL tier=%0d (first pixel tier)", pxl_tier);
        if (pxl_eol) $display("  PACKER: EOL");
    end

    // ── Main test ──
    initial begin
        $display("");
        $display("==========================================================");
        $display("  Pixel Pipeline Testbench (VTC -> Fov -> Packer v3)");
        $display("  Resolution: %0dx%0d, Gaze: (%0d,%0d)", H_ACTIVE, V_ACTIVE, GAZE_X, GAZE_Y);
        $display("  Foveal r^2=%0d, Mid r^2=%0d", RAD_FOV_SQ, RAD_MID_SQ);
        $display("==========================================================");
        $display("");

        dpi_pixel_data = 24'h000000;
        dpi_hsync = 1'b0; dpi_vsync = 1'b0; dpi_de = 1'b0;

        @(posedge rst_n);
        repeat(5) @(posedge pixel_clk);

        // ── Frame 1 ──
        $display("--- Generating Frame 1 (foveation ENABLED) ---");
        pixel_count = 0; sol_count = 0; eol_count = 0; sof_count = 0;
        tier0_count = 0; tier1_count = 0; tier2_count = 0;

        generate_frame();
        repeat(10) @(posedge pixel_clk);

        $display("");
        $display("--- Structural Verification ---");
        check("Total pixels", H_ACTIVE * V_ACTIVE, pixel_count, 32);
        check("SOL count", V_ACTIVE, sol_count, 32);
        check("EOL count", V_ACTIVE, eol_count, 32);
        check("SOF count", 1, sof_count, 32);

        $display("");
        $display("--- Foveation Tier Distribution (Frame 1) ---");
        $display("  Tier 0 (Foveal):  %0d pixels  (%0d%%)", tier0_count, tier0_count * 100 / (H_ACTIVE*V_ACTIVE));
        $display("  Tier 1 (Mid):     %0d pixels  (%0d%%)", tier1_count, tier1_count * 100 / (H_ACTIVE*V_ACTIVE));
        $display("  Tier 2 (Outer):   %0d pixels  (%0d%%)", tier2_count, tier2_count * 100 / (H_ACTIVE*V_ACTIVE));
        $display("  Total:            %0d pixels", tier0_count + tier1_count + tier2_count);

        check("Tier 0 has pixels", 1, (tier0_count > 0) ? 1 : 0, 1);
        check("Tier 1 has pixels", 1, (tier1_count > 0) ? 1 : 0, 1);
        check("Tier 2 has pixels", 1, (tier2_count > 0) ? 1 : 0, 1);
        check("Tier sum = total", pixel_count, tier0_count + tier1_count + tier2_count, 32);

        // ── Bandwidth analysis (single frame, correct calculation) ──
        $display("");
        $display("--- Bandwidth Analysis (Single Frame) ---");
        begin : bw_analysis
            integer baseline_bytes, foveated_bytes, saving_pct;
            baseline_bytes  = H_ACTIVE * V_ACTIVE * 3;
            foveated_bytes  = tier0_count * 3 + tier1_count * 3 + tier2_count * 2;
            saving_pct      = (baseline_bytes > 0) ?
                              ((baseline_bytes - foveated_bytes) * 100) / baseline_bytes : 0;
            $display("  Uniform RGB888:   %0d bytes/frame", baseline_bytes);
            $display("  Foveated:         %0d bytes/frame", foveated_bytes);
            $display("  Bandwidth saved:  %0d bytes  (%0d%%)", baseline_bytes - foveated_bytes, saving_pct);
        end

        // ── Frame 2 (repeatability) ──
        $display("");
        $display("--- Generating Frame 2 (repeatability check) ---");
        pixel_count = 0; sol_count = 0; eol_count = 0; sof_count = 0;

        generate_frame();
        repeat(10) @(posedge pixel_clk);

        check("Frame 2: Total pixels", H_ACTIVE * V_ACTIVE, pixel_count, 32);

        // ── Summary ──
        $display("");
        $display("==========================================================");
        $display("  TEST SUMMARY");
        $display("==========================================================");
        $display("  Total checks:  %0d", test_count);
        $display("  Passed:        %0d", pass_count);
        $display("  Failed:        %0d", fail_count);
        $display("==========================================================");
        if (fail_count == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> %0d TESTS FAILED <<<", fail_count);
        $display("==========================================================");
        $display("");
        #100; $finish;
    end

    initial begin #500000; $display("[TIMEOUT]"); $finish; end
    initial begin $dumpfile("tb_pixel_pipeline.vcd"); $dumpvars(0, tb_pixel_pipeline); end

endmodule
