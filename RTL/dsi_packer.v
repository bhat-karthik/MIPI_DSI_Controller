//=============================================================================
// Module:      dsi_packer
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_packer.v  (v3 — simplified, per-pixel tier)
//
// Description:
//   Pixel format converter. Applies RGB format conversion based on the
//   PER-PIXEL tier from the Foveated Region Mapper. No line-tier latching,
//   no segment detection — those move to the Packetizer's line buffer.
//
//   This module:
//     1. Receives per-pixel tier from fov_mapper (true circular foveation)
//     2. Converts pixel data: Tier0→RGB888, Tier1→RGB666LP, Tier2→RGB565
//     3. Tags SOL/EOL/SOF on the appropriate pixel entries
//     4. Outputs to CDC FIFO at 1 entry per pixel_clk during active time
//
//   EOL is tagged on the LAST VALID PIXEL by counting pixels against
//   cfg_h_active (since VTC's line_end fires one cycle AFTER the last
//   valid pixel and cannot be attached to a FIFO entry).
//
// Power Domain: PD_DSI_CORE
//=============================================================================

module dsi_packer (
    input  wire        pixel_clk,
    input  wire        rst_n,

    // Pixel Input (from Foveated Region Mapper)
    input  wire [23:0] pixel_data_in,
    input  wire        pixel_valid_in,
    input  wire [1:0]  fov_tier,
    input  wire        line_start_in,
    input  wire        line_end_in,
    input  wire        frame_start_in,
    input  wire        frame_end_in,

    // Configuration
    input  wire [11:0] cfg_h_active,

    // Output to CDC FIFO (no pxl_dt/pxl_wc — packetizer computes those)
    output reg  [23:0] pxl_data,
    output reg  [1:0]  pxl_tier,
    output reg         pxl_valid,
    output reg         pxl_sol,
    output reg         pxl_eol,
    output reg         pxl_sof
);

    //=========================================================================
    // Pixel counter: detect last pixel of line
    //=========================================================================
    reg [11:0] pixel_count;
    wire       is_last_pixel = pixel_valid_in &&
                               (pixel_count == cfg_h_active - 12'd1);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n)
            pixel_count <= 12'd0;
        else if (line_start_in && pixel_valid_in)
            pixel_count <= 12'd1;
        else if (pixel_valid_in)
            pixel_count <= pixel_count + 12'd1;
        else if (line_end_in)
            pixel_count <= 12'd0;
    end

    //=========================================================================
    // Pixel format conversion (per-pixel, based on fov_tier)
    //=========================================================================
    reg [23:0] formatted_pixel;

    always @(*) begin
        case (fov_tier)
            2'b00:   formatted_pixel = pixel_data_in;
            2'b01:   formatted_pixel = {pixel_data_in[23:18], 2'b00,
                                        pixel_data_in[15:10], 2'b00,
                                        pixel_data_in[7:2],   2'b00};
            2'b10:   formatted_pixel = {8'h00,
                                        pixel_data_in[23:19],
                                        pixel_data_in[15:10],
                                        pixel_data_in[7:3]};
            default: formatted_pixel = pixel_data_in;
        endcase
    end

    //=========================================================================
    // Output registration
    //=========================================================================
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            pxl_data  <= 24'd0;
            pxl_tier  <= 2'b00;
            pxl_valid <= 1'b0;
            pxl_sol   <= 1'b0;
            pxl_eol   <= 1'b0;
            pxl_sof   <= 1'b0;
        end else begin
            pxl_valid <= pixel_valid_in;
            if (pixel_valid_in) begin
                pxl_data <= formatted_pixel;
                pxl_tier <= fov_tier;
            end
            pxl_sol <= line_start_in & pixel_valid_in;
            pxl_eol <= is_last_pixel;
            pxl_sof <= frame_start_in & pixel_valid_in;
        end
    end

endmodule
