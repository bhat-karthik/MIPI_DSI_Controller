//=============================================================================
// Module:      dsi_vtc
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_vtc.v
// Author:      MTech VLSI Design Team
// Date:        April 2026
//
// Description:
//   Video Timing Controller (VTC) for the MIPI DSI Host Controller.
//   Generates display timing signals and passes pixel data downstream
//   to the Foveated Region Mapper with position coordinates.
//
//   TWO OPERATING MODES:
//   Mode 0 (cfg_vtc_mode=0): External DPI passthrough
//     - Uses hsync_in, vsync_in, de_in directly from a DPI video source
//     - h_count and v_count derived by counting active pixels/lines
//     - Simplest mode, used when an upstream DPI source exists
//
//   Mode 1 (cfg_vtc_mode=1): Internal timing generation
//     - Free-running H/V counters generate all timing internally
//     - DPI input signals (hsync_in, vsync_in, de_in) are ignored
//     - pixel_data_in is sampled when counters indicate active region
//     - Used for standalone operation or test pattern generation
//
//   TIMING REGIONS (per line):
//     |<-SYNC->|<---BP--->|<--------ACTIVE-------->|<---FP--->|
//     |  hsync |   hbp    |     pixel data          |   hfp    |
//     0     H_SYNC  H_SYNC+BP  H_SYNC+BP+ACTIVE  H_TOTAL-1
//
//   PIXEL COORDINATE SYSTEM:
//     h_count: 0-based index within active region (0 to H_ACTIVE-1)
//     v_count: 0-based line index within active region (0 to V_ACTIVE-1)
//     These coordinates feed into the Foveated Region Mapper for
//     distance-from-gaze computation.
//
//   CDC NOTES:
//     All cfg_* inputs come from byte_clk domain (APB registers).
//     They are quasi-static (changed only during blanking or by software
//     when video is disabled). No 2-FF sync is strictly needed for
//     quasi-static signals, but we register them once on entry for
//     clean timing paths. A design with runtime gaze updates would
//     need proper 2-FF synchronization (handled in the fov_mapper).
//
//   POWER DOMAIN:
//     This module belongs to PD_DSI_CORE (switchable).
//
// Estimated gates: ~2,000
// Estimated lines: ~180
//
//=============================================================================

module dsi_vtc (
    //=========================================================================
    // Clock and Reset
    //=========================================================================
    input  wire        pixel_clk,         // Pixel clock (148.5 MHz for 1080p/60)
    input  wire        rst_n,             // Active-low synchronous reset (pixel_clk domain)

    //=========================================================================
    // Configuration (from APB registers, byte_clk domain — quasi-static)
    //=========================================================================
    input  wire        cfg_vtc_mode,      // 0=external DPI, 1=internal generation
    input  wire [11:0] cfg_h_active,      // Horizontal active pixels (e.g., 1920)
    input  wire [11:0] cfg_h_fp,          // Horizontal front porch (pixels)
    input  wire [11:0] cfg_h_sync,        // Horizontal sync width (pixels)
    input  wire [11:0] cfg_h_bp,          // Horizontal back porch (pixels)
    input  wire [11:0] cfg_v_active,      // Vertical active lines (e.g., 1080)
    input  wire [11:0] cfg_v_fp,          // Vertical front porch (lines)
    input  wire [11:0] cfg_v_sync,        // Vertical sync width (lines)
    input  wire [11:0] cfg_v_bp,          // Vertical back porch (lines)
    input  wire        cfg_video_en,      // Video enable (master gate)

    //=========================================================================
    // DPI Video Input
    //=========================================================================
    input  wire [23:0] pixel_data_in,     // RGB888 pixel data from DPI source
    input  wire        hsync_in,          // Horizontal sync from DPI
    input  wire        vsync_in,          // Vertical sync from DPI
    input  wire        de_in,             // Data enable from DPI

    //=========================================================================
    // Outputs to Foveated Region Mapper
    //=========================================================================
    output reg  [23:0] pixel_data_out,    // Pixel data (registered)
    output reg         pixel_valid,       // Pixel data valid strobe
    output reg  [11:0] h_count,           // Horizontal pixel position (0-based, active region)
    output reg  [11:0] v_count,           // Vertical line position (0-based, active region)
    output reg         line_start,        // Pulse: first active pixel of line
    output reg         line_end,          // Pulse: last active pixel of line
    output reg         frame_start,       // Pulse: first active pixel of frame
    output reg         frame_end,         // Pulse: last active pixel of frame

    //=========================================================================
    // Blanking/Sync Status (used by Packetizer for sync packet insertion)
    //=========================================================================
    output wire        in_hsync,          // Currently in horizontal sync period
    output wire        in_vsync,          // Currently in vertical sync period
    output wire        in_hbp,            // Currently in horizontal back porch
    output wire        in_hfp             // Currently in horizontal front porch
);

    //=========================================================================
    // Internal timing parameters (computed from config)
    //=========================================================================
    // These are the boundary values for region detection in internal mode.
    // Registered once from config inputs for clean timing.
    reg [11:0] h_total_m1;  // H_TOTAL - 1 (counter wraps here)
    reg [11:0] v_total_m1;  // V_TOTAL - 1

    // Region boundary indices (horizontal)
    // |<-SYNC->|<---BP--->|<--------ACTIVE-------->|<---FP--->|
    // 0     sync_end  bp_end                    active_end  h_total_m1
    reg [11:0] h_sync_end;   // Last pixel of sync region
    reg [11:0] h_bp_end;     // Last pixel of back porch
    reg [11:0] h_active_end; // Last pixel of active region
    // Vertical boundaries (same structure)
    reg [11:0] v_sync_end;
    reg [11:0] v_bp_end;
    reg [11:0] v_active_end;

    always @(posedge pixel_clk) begin
        h_sync_end   <= cfg_h_sync - 12'd1;
        h_bp_end     <= cfg_h_sync + cfg_h_bp - 12'd1;
        h_active_end <= cfg_h_sync + cfg_h_bp + cfg_h_active - 12'd1;
        h_total_m1   <= cfg_h_sync + cfg_h_bp + cfg_h_active + cfg_h_fp - 12'd1;

        v_sync_end   <= cfg_v_sync - 12'd1;
        v_bp_end     <= cfg_v_sync + cfg_v_bp - 12'd1;
        v_active_end <= cfg_v_sync + cfg_v_bp + cfg_v_active - 12'd1;
        v_total_m1   <= cfg_v_sync + cfg_v_bp + cfg_v_active + cfg_v_fp - 12'd1;
    end

    //=========================================================================
    // Internal Mode: Free-running counters
    //=========================================================================
    reg [11:0] h_cnt_int;  // Horizontal counter (0 to H_TOTAL-1)
    reg [11:0] v_cnt_int;  // Vertical counter (0 to V_TOTAL-1)

    wire h_cnt_wrap = (h_cnt_int == h_total_m1);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt_int <= 12'd0;
            v_cnt_int <= 12'd0;
        end else if (cfg_vtc_mode && cfg_video_en) begin
            // Internal mode: free-running counters
            if (h_cnt_wrap) begin
                h_cnt_int <= 12'd0;
                if (v_cnt_int == v_total_m1)
                    v_cnt_int <= 12'd0;
                else
                    v_cnt_int <= v_cnt_int + 12'd1;
            end else begin
                h_cnt_int <= h_cnt_int + 12'd1;
            end
        end else begin
            h_cnt_int <= 12'd0;
            v_cnt_int <= 12'd0;
        end
    end

    // Internal mode: region detection
    wire int_in_hsync  = (h_cnt_int <= h_sync_end);
    wire int_in_hbp    = (h_cnt_int > h_sync_end)   && (h_cnt_int <= h_bp_end);
    wire int_in_active = (h_cnt_int > h_bp_end)      && (h_cnt_int <= h_active_end);
    wire int_in_hfp    = (h_cnt_int > h_active_end);

    wire int_in_vsync  = (v_cnt_int <= v_sync_end);
    wire int_v_active  = (v_cnt_int > v_bp_end) && (v_cnt_int <= v_active_end);

    wire int_pixel_valid = int_in_active && int_v_active && cfg_video_en;

    //=========================================================================
    // External Mode: DPI signal edge detection and counting
    //=========================================================================
    reg de_d1;
    reg hsync_d1;
    reg vsync_d1;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d1    <= 1'b0;
            hsync_d1 <= 1'b0;
            vsync_d1 <= 1'b0;
        end else begin
            de_d1    <= de_in;
            hsync_d1 <= hsync_in;
            vsync_d1 <= vsync_in;
        end
    end

    wire de_rising  = de_in  & ~de_d1;    // Start of active data
    wire de_falling = ~de_in & de_d1;     // End of active data
    wire vsync_rising  = vsync_in  & ~vsync_d1;
    wire vsync_falling = ~vsync_in & vsync_d1;

    // External mode: count active pixels within de_in window
    reg [11:0] ext_h_count;
    reg [11:0] ext_v_count;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            ext_h_count <= 12'd0;
            ext_v_count <= 12'd0;
        end else begin
            // Horizontal pixel counter: resets at de rising, counts during de
            if (de_rising)
                ext_h_count <= 12'd0;
            else if (de_in && cfg_video_en)
                ext_h_count <= ext_h_count + 12'd1;

            // Vertical line counter: resets at vsync, increments at de falling
            if (vsync_rising)
                ext_v_count <= 12'd0;
            else if (de_falling)
                ext_v_count <= ext_v_count + 12'd1;
        end
    end

    //=========================================================================
    // Output multiplexer: select between internal and external mode
    //=========================================================================
    wire        mux_pixel_valid;
    wire [11:0] mux_h_count;
    wire [11:0] mux_v_count;
    wire        mux_in_hsync;
    wire        mux_in_vsync;
    wire        mux_in_hbp;
    wire        mux_in_hfp;

    assign mux_pixel_valid = cfg_vtc_mode ? int_pixel_valid
                                          : (de_in && cfg_video_en);

    assign mux_h_count = cfg_vtc_mode ? (h_cnt_int - h_bp_end - 12'd1)
                                      : (de_rising ? 12'd0 : ext_h_count);

    assign mux_v_count = cfg_vtc_mode ? (v_cnt_int - v_bp_end - 12'd1)
                                      : ext_v_count;

    assign mux_in_hsync = cfg_vtc_mode ? int_in_hsync  : hsync_in;
    assign mux_in_vsync = cfg_vtc_mode ? int_in_vsync  : vsync_in;
    assign mux_in_hbp   = cfg_vtc_mode ? int_in_hbp    : 1'b0;  // Not tracked in ext mode
    assign mux_in_hfp   = cfg_vtc_mode ? int_in_hfp    : 1'b0;

    //=========================================================================
    // Output registers (one pipeline stage for clean timing)
    //=========================================================================
    reg mux_pixel_valid_d1;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_data_out   <= 24'd0;
            pixel_valid      <= 1'b0;
            h_count          <= 12'd0;
            v_count          <= 12'd0;
            line_start       <= 1'b0;
            line_end         <= 1'b0;
            frame_start      <= 1'b0;
            frame_end        <= 1'b0;
            mux_pixel_valid_d1 <= 1'b0;
        end else begin
            // Register pixel data — pass through when valid, hold when not
            pixel_valid <= mux_pixel_valid;
            if (mux_pixel_valid)
                pixel_data_out <= pixel_data_in;

            // Register coordinates
            h_count <= mux_h_count;
            v_count <= mux_v_count;

            // Delayed valid for edge detection
            mux_pixel_valid_d1 <= mux_pixel_valid;

            // Line start: rising edge of pixel_valid
            // (first active pixel of each line)
            line_start <= mux_pixel_valid & ~mux_pixel_valid_d1;

            // Line end: falling edge of pixel_valid
            // (registered one cycle after last active pixel)
            line_end <= ~mux_pixel_valid & mux_pixel_valid_d1;

            // Frame start: first pixel_valid after vsync
            // Detect: pixel_valid rises AND v_count == 0
            if (cfg_vtc_mode)
                frame_start <= mux_pixel_valid & ~mux_pixel_valid_d1 &
                               (mux_v_count == 12'd0);
            else
                frame_start <= mux_pixel_valid & ~mux_pixel_valid_d1 &
                               (ext_v_count == 12'd0);

            // Frame end: last line_end of frame
            // Detect: pixel_valid falls AND v_count == V_ACTIVE-1
            if (cfg_vtc_mode)
                frame_end <= ~mux_pixel_valid & mux_pixel_valid_d1 &
                             (v_cnt_int == v_active_end);
            else
                frame_end <= ~mux_pixel_valid & mux_pixel_valid_d1 &
                             (ext_v_count == cfg_v_active);
        end
    end

    //=========================================================================
    // Blanking/sync status outputs (directly from mux, unregistered)
    //=========================================================================
    assign in_hsync = mux_in_hsync;
    assign in_vsync = mux_in_vsync;
    assign in_hbp   = mux_in_hbp;
    assign in_hfp   = mux_in_hfp;

    //=========================================================================
    // SVA Assertions (simulation only)
    //=========================================================================
    `ifdef SIMULATION

    // h_count should never exceed cfg_h_active-1 during pixel_valid
    property p_h_count_range;
        @(posedge pixel_clk) disable iff (!rst_n)
        pixel_valid |-> (h_count < cfg_h_active);
    endproperty
    assert property (p_h_count_range)
        else $error("VTC: h_count=%0d exceeds H_ACTIVE=%0d", h_count, cfg_h_active);

    // v_count should never exceed cfg_v_active-1 during pixel_valid
    property p_v_count_range;
        @(posedge pixel_clk) disable iff (!rst_n)
        pixel_valid |-> (v_count < cfg_v_active);
    endproperty
    assert property (p_v_count_range)
        else $error("VTC: v_count=%0d exceeds V_ACTIVE=%0d", v_count, cfg_v_active);

    // line_start should be exactly 1 cycle wide
    property p_line_start_pulse;
        @(posedge pixel_clk) disable iff (!rst_n)
        line_start |=> !line_start;
    endproperty
    assert property (p_line_start_pulse)
        else $error("VTC: line_start was not a single-cycle pulse");

    // frame_start should be exactly 1 cycle wide
    property p_frame_start_pulse;
        @(posedge pixel_clk) disable iff (!rst_n)
        frame_start |=> !frame_start;
    endproperty
    assert property (p_frame_start_pulse)
        else $error("VTC: frame_start was not a single-cycle pulse");

    `endif

endmodule
