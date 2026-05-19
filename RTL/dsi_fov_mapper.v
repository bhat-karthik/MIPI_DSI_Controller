//=============================================================================
// Module:      dsi_fov_mapper
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_fov_mapper.v
// Author:      MTech VLSI Design Team
// Date:        April 2026
//
// Description:
//   Foveated Region Mapper — the NOVEL ARCHITECTURAL CONTRIBUTION of this
//   project. This module computes the squared Euclidean distance from each
//   pixel to the current gaze fixation point and assigns a quality tier
//   based on configurable radius thresholds.
//
//   ALGORITHM:
//     dist_sq = (h_count - gaze_x)^2 + (v_count - gaze_y)^2
//
//     if      dist_sq <= radius_foveal_sq → tier = 00 (Foveal, full RGB888)
//     else if dist_sq <= radius_mid_sq   → tier = 01 (Mid, reduced quality)
//     else                                → tier = 10 (Outer, minimum quality)
//
//   WHY SQUARED DISTANCE:
//     Square root is not synthesizable as single-cycle combinational logic
//     at 148.5 MHz. By comparing dist_sq against radius_sq (pre-squared
//     thresholds stored in registers), we avoid the sqrt entirely.
//     The comparison dist <= radius is equivalent to dist^2 <= radius^2
//     for non-negative values.
//
//   PIPELINE (2 stages, adds 2 pixel_clk latency):
//     Stage 1: Compute dx = h_count - gaze_x (signed 13-bit)
//              Compute dy = v_count - gaze_y (signed 13-bit)
//              Pipeline pixel_data and control signals
//
//     Stage 2: Compute dx*dx (25-bit unsigned) and dy*dy (25-bit unsigned)
//              Compute dist_sq = dx*dx + dy*dy (26-bit unsigned)
//              Compare against thresholds → assign tier
//              Pipeline pixel_data and control signals
//
//   ARITHMETIC:
//     h_count range: 0–1919, gaze_x range: 0–1919
//     dx range: -1919 to +1919 → needs 12-bit magnitude + 1 sign = 13 bits
//     dx*dx range: 0 to 1919^2 = 3,682,561 → needs 22 bits (fits in 25)
//     dist_sq range: 0 to 2*1919^2 = 7,365,122 → needs 23 bits (fits in 26)
//
//   CDC FOR GAZE COORDINATES:
//     Gaze coordinates (gaze_x, gaze_y) and radius thresholds are updated
//     by software once per frame (~16 ms at 60fps). They are quasi-static
//     signals crossing from byte_clk to pixel_clk. A 2-FF synchronizer
//     is used for each bit. The synchronizer adds 2 pixel_clk cycles of
//     latency, but since updates happen once per frame, this is negligible.
//
//   BYPASS MODE:
//     When cfg_fov_enable = 0: tier is forced to 2'b00 (foveal) for all
//     pixels, making the Packer select RGB888 for everything. This is the
//     "standard display" mode with no foveation, used as the power/bandwidth
//     comparison baseline.
//
//   POWER DOMAIN:
//     This module belongs to PD_FOVEATION (independently switchable).
//     When powered off, isolation cells clamp fov_tier to 2'b00 (foveal),
//     which is the correct bypass behavior.
//
// Estimated gates: ~4,000 (two 13x13 multipliers + comparators + pipeline FFs)
// Estimated lines: ~200
//
//=============================================================================

module dsi_fov_mapper (
    //=========================================================================
    // Clock and Reset
    //=========================================================================
    input  wire        pixel_clk,
    input  wire        rst_n,

    //=========================================================================
    // Pixel Input (from VTC)
    //=========================================================================
    input  wire [23:0] pixel_data_in,
    input  wire        pixel_valid_in,
    input  wire [11:0] h_count,           // Horizontal position (0-based active)
    input  wire [11:0] v_count,           // Vertical position (0-based active)
    input  wire        line_start_in,
    input  wire        line_end_in,
    input  wire        frame_start_in,
    input  wire        frame_end_in,

    //=========================================================================
    // Configuration (from APB registers, byte_clk domain — quasi-static)
    // These are synchronized internally via 2-FF synchronizers.
    //=========================================================================
    input  wire [11:0] cfg_gaze_x,        // Gaze X coordinate
    input  wire [11:0] cfg_gaze_y,        // Gaze Y coordinate
    input  wire [25:0] cfg_radius_foveal_sq, // Foveal radius squared (default: 10000)
    input  wire [25:0] cfg_radius_mid_sq,    // Mid-peripheral radius squared (default: 90000)
    input  wire        cfg_fov_enable,    // 1=enable foveation, 0=bypass (all Tier 0)

    //=========================================================================
    // Pixel Output (to Packer)
    // All outputs are delayed by 2 pixel_clk cycles from inputs.
    //=========================================================================
    output reg  [23:0] pixel_data_out,
    output reg         pixel_valid_out,
    output reg  [1:0]  fov_tier,          // 00=foveal, 01=mid, 10=outer
    output reg         line_start_out,
    output reg         line_end_out,
    output reg         frame_start_out,
    output reg         frame_end_out
);

    //=========================================================================
    // CDC Synchronization for gaze coordinates and thresholds
    //=========================================================================
    // 2-FF synchronizer for quasi-static config signals crossing from
    // byte_clk to pixel_clk domain. These change at most once per frame
    // (~16 ms), so 2-FF synchronization is more than sufficient.
    //
    // We synchronize the entire config word atomically by registering
    // through two stages. Since the signals are quasi-static (software
    // ensures they change only during blanking), there's no risk of
    // capturing a partially-updated set of coordinates.

    reg [11:0] gaze_x_sync1, gaze_x_sync2;
    reg [11:0] gaze_y_sync1, gaze_y_sync2;
    reg [25:0] rad_fov_sync1, rad_fov_sync2;
    reg [25:0] rad_mid_sync1, rad_mid_sync2;
    reg        fov_en_sync1, fov_en_sync2;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            gaze_x_sync1  <= 12'd0;  gaze_x_sync2  <= 12'd0;
            gaze_y_sync1  <= 12'd0;  gaze_y_sync2  <= 12'd0;
            rad_fov_sync1 <= 26'd10000; rad_fov_sync2 <= 26'd10000;
            rad_mid_sync1 <= 26'd90000; rad_mid_sync2 <= 26'd90000;
            fov_en_sync1  <= 1'b0;   fov_en_sync2  <= 1'b0;
        end else begin
            // Stage 1
            gaze_x_sync1  <= cfg_gaze_x;
            gaze_y_sync1  <= cfg_gaze_y;
            rad_fov_sync1 <= cfg_radius_foveal_sq;
            rad_mid_sync1 <= cfg_radius_mid_sq;
            fov_en_sync1  <= cfg_fov_enable;
            // Stage 2
            gaze_x_sync2  <= gaze_x_sync1;
            gaze_y_sync2  <= gaze_y_sync1;
            rad_fov_sync2 <= rad_fov_sync1;
            rad_mid_sync2 <= rad_mid_sync1;
            fov_en_sync2  <= fov_en_sync1;
        end
    end

    //=========================================================================
    // Pipeline Stage 1: Compute dx and dy (signed subtraction)
    //=========================================================================
    // dx = h_count - gaze_x : range [-1919, +1919], needs 13-bit signed
    // dy = v_count - gaze_y : range [-1079, +1079], needs 12-bit signed (13 for uniformity)

    reg signed [12:0] s1_dx;
    reg signed [12:0] s1_dy;
    reg        [23:0] s1_pixel_data;
    reg               s1_pixel_valid;
    reg               s1_line_start;
    reg               s1_line_end;
    reg               s1_frame_start;
    reg               s1_frame_end;
    reg               s1_fov_enable;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_dx          <= 13'sd0;
            s1_dy          <= 13'sd0;
            s1_pixel_data  <= 24'd0;
            s1_pixel_valid <= 1'b0;
            s1_line_start  <= 1'b0;
            s1_line_end    <= 1'b0;
            s1_frame_start <= 1'b0;
            s1_frame_end   <= 1'b0;
            s1_fov_enable  <= 1'b0;
        end else begin
            // Signed subtraction: extend 12-bit unsigned to 13-bit signed, then subtract
            s1_dx <= {1'b0, h_count} - {1'b0, gaze_x_sync2};
            s1_dy <= {1'b0, v_count} - {1'b0, gaze_y_sync2};

            // Pipeline passthrough
            s1_pixel_data  <= pixel_data_in;
            s1_pixel_valid <= pixel_valid_in;
            s1_line_start  <= line_start_in;
            s1_line_end    <= line_end_in;
            s1_frame_start <= frame_start_in;
            s1_frame_end   <= frame_end_in;
            s1_fov_enable  <= fov_en_sync2;
        end
    end

    //=========================================================================
    // Pipeline Stage 2: Square, sum, compare, assign tier
    //=========================================================================
    // dx*dx: 13-bit signed × 13-bit signed → 26-bit result (always positive)
    //        We take absolute value conceptually: (-x)^2 = x^2
    //        Signed multiply gives correct result automatically
    // dist_sq = dx*dx + dy*dy: max = 1919^2 + 1079^2 = 4,846,722 → 23 bits
    //           Using 26 bits for safety

    // Combinational multiply (Genus infers single-cycle multiplier)
    wire [25:0] dx_sq = s1_dx * s1_dx;  // 13b signed × 13b signed → 26b
    wire [25:0] dy_sq = s1_dy * s1_dy;
    wire [25:0] dist_sq = dx_sq + dy_sq;

    // Tier assignment (combinational)
    wire [1:0] tier_calc;
    assign tier_calc = (!s1_fov_enable)                  ? 2'b00 :  // Bypass: all foveal
                       (dist_sq <= rad_fov_sync2)        ? 2'b00 :  // Foveal
                       (dist_sq <= rad_mid_sync2)        ? 2'b01 :  // Mid-peripheral
                                                           2'b10;   // Outer-peripheral

    // Stage 2 output registers
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_data_out  <= 24'd0;
            pixel_valid_out <= 1'b0;
            fov_tier        <= 2'b00;
            line_start_out  <= 1'b0;
            line_end_out    <= 1'b0;
            frame_start_out <= 1'b0;
            frame_end_out   <= 1'b0;
        end else begin
            pixel_data_out  <= s1_pixel_data;
            pixel_valid_out <= s1_pixel_valid;
            fov_tier        <= tier_calc;
            line_start_out  <= s1_line_start;
            line_end_out    <= s1_line_end;
            frame_start_out <= s1_frame_start;
            frame_end_out   <= s1_frame_end;
        end
    end

    //=========================================================================
    // SVA Assertions (simulation only)
    //=========================================================================
    `ifdef SIMULATION

    // Tier must be in valid range {00, 01, 10}
    property p_tier_valid;
        @(posedge pixel_clk) disable iff (!rst_n)
        pixel_valid_out |-> (fov_tier != 2'b11);
    endproperty
    assert property (p_tier_valid)
        else $error("FOV: Invalid tier value 2'b11");

    // When foveation disabled, tier must always be 00
    property p_bypass_tier;
        @(posedge pixel_clk) disable iff (!rst_n)
        (pixel_valid_out && !fov_en_sync2) |-> (fov_tier == 2'b00);
    endproperty
    assert property (p_bypass_tier)
        else $error("FOV: Bypass mode but tier != 00");

    // Tier monotonicity: if dist_sq increases, tier should not decrease
    // (This is a consequence of foveal_sq < mid_sq by design)
    property p_radius_ordering;
        @(posedge pixel_clk) disable iff (!rst_n)
        (rad_fov_sync2 <= rad_mid_sync2);
    endproperty
    assert property (p_radius_ordering)
        else $warning("FOV: Foveal radius >= mid radius — check configuration");

    // Latency check: pixel_valid_out should follow pixel_valid_in by exactly 2 cycles
    // (This is structural — validated by construction, but good to assert)

    `endif

endmodule
