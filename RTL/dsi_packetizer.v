//=============================================================================
// Module:      dsi_packetizer
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_packetizer.v  (v3 — line-buffer architecture)
//
// Description:
//   DSI Packet Formation Engine with internal line buffer for true circular
//   (multi-segment) foveated rendering support.
//
//   TWO-PHASE OPERATION per display line:
//
//   PHASE 1 — BUFFER (ST_BUFFER):
//     Read all pixels of one line from the CDC FIFO into an internal
//     line buffer. While buffering, detect tier transitions and build
//     a segment descriptor table. When EOL arrives, the entire line is
//     buffered and all segment boundaries are known.
//
//   PHASE 2 — EMIT (ST_VSS → ST_HSS → ST_PREP_SEG → ST_EMIT_HDR →
//                    ST_EMIT_DATA → ST_EMIT_CRC → ST_NEXT_SEG):
//     For each segment: compute WC, emit header, serialize pixels from
//     line buffer to bytes through CRC, emit CRC footer with pkt_eop.
//
//   FIFO READ (FWFT): rd_data valid when !empty. rd_en advances pointer.
//
// Parameters:
//   LINE_BUF_DEPTH — max pixels per line (default 2048)
//   MAX_SEGMENTS   — max tier segments per line (default 8)
//=============================================================================

module dsi_packetizer #(
    parameter LINE_BUF_DEPTH = 2048,
    parameter MAX_SEGMENTS   = 8
) (
    input  wire        byte_clk,
    input  wire        rst_n,

    // FIFO interface (FWFT)
    input  wire [31:0] fifo_rdata,
    input  wire        fifo_empty,
    output reg         fifo_rd_en,

    // Configuration
    input  wire [1:0]  cfg_vc,
    input  wire [11:0] cfg_h_active,

    // Packet byte output
    output reg  [7:0]  pkt_data,
    output reg         pkt_valid,
    output reg         pkt_sop,
    output reg         pkt_eop
);

    // FIFO word unpacking
    wire [23:0] fifo_pixel = fifo_rdata[23:0];
    wire [1:0]  fifo_tier  = fifo_rdata[25:24];
    wire        fifo_sol   = fifo_rdata[26];
    wire        fifo_eol   = fifo_rdata[27];
    wire        fifo_sof   = fifo_rdata[28];

    // Constants
    localparam [5:0] DT_VSS = 6'h01, DT_HSS = 6'h21;
    localparam [5:0] DT_RGB888 = 6'h3E, DT_RGB666 = 6'h2E, DT_RGB565 = 6'h0E;

    // FSM States
    localparam [3:0] ST_IDLE     = 4'd0, ST_BUFFER   = 4'd1,
                     ST_VSS      = 4'd2, ST_HSS      = 4'd3,
                     ST_PREP_SEG = 4'd4, ST_EMIT_HDR = 4'd5,
                     ST_EMIT_DATA= 4'd6, ST_EMIT_CRC = 4'd7,
                     ST_NEXT_SEG = 4'd8, ST_GAP      = 4'd9;
    reg [3:0] state;

    // Line buffer: {tier[1:0], pixel_data[23:0]} = 26 bits per entry
    reg [25:0] line_buf [0:LINE_BUF_DEPTH-1];
    reg [11:0] lb_wr_idx;

    // Segment table
    reg [11:0] seg_start [0:MAX_SEGMENTS-1];
    reg [11:0] seg_count [0:MAX_SEGMENTS-1];
    reg [1:0]  seg_tier  [0:MAX_SEGMENTS-1];
    reg [3:0]  num_segments, cur_seg, seg_build_idx;

    // Line metadata
    reg        line_has_sof;
    reg [1:0]  prev_tier;

    // Emit state
    reg [2:0]  hdr_idx;
    reg [11:0] emit_pix_idx;
    reg [1:0]  emit_byte_idx, emit_bpp;
    reg [5:0]  emit_dt, short_dt;
    reg [15:0] emit_wc;

    // ECC
    reg  [23:0] ecc_header_in;
    wire [7:0]  ecc_out;
    dsi_ecc u_ecc (.header_in(ecc_header_in), .ecc_out(ecc_out),
                   .ecc_rx(8'd0), .syndrome(), .err_single(), .err_double());

    // CRC
    reg crc_init_r, crc_en_r;
    reg [7:0] crc_data_r;
    wire [15:0] crc_out;
    dsi_crc16 u_crc (.clk(byte_clk), .rst_n(rst_n), .crc_init(crc_init_r),
                     .crc_en(crc_en_r), .data_in(crc_data_r), .crc_out(crc_out));

    // Helpers
    function [5:0] dt_from_tier; input [1:0] t; begin
        case (t) 2'b00: dt_from_tier=DT_RGB888; 2'b01: dt_from_tier=DT_RGB666;
                  2'b10: dt_from_tier=DT_RGB565; default: dt_from_tier=DT_RGB888; endcase
    end endfunction

    function [1:0] bpp_from_tier; input [1:0] t; begin
        bpp_from_tier = (t == 2'b10) ? 2'd2 : 2'd3;
    end endfunction

    // Current pixel from line buffer during emit
    wire [25:0] lb_entry = line_buf[seg_start[cur_seg] + emit_pix_idx];
    wire [23:0] lb_pixel = lb_entry[23:0];

    // Byte extraction from current pixel
    reg [7:0] cur_byte;
    always @(*) begin
        if (emit_bpp == 2'd2)
            case (emit_byte_idx)
                2'd0: cur_byte = lb_pixel[7:0]; 2'd1: cur_byte = lb_pixel[15:8];
                default: cur_byte = 8'h00;
            endcase
        else
            case (emit_byte_idx)
                2'd0: cur_byte = lb_pixel[7:0]; 2'd1: cur_byte = lb_pixel[15:8];
                2'd2: cur_byte = lb_pixel[23:16]; default: cur_byte = 8'h00;
            endcase
    end

    // ECC input mux
    always @(*) begin
        case (state)
            ST_VSS:      ecc_header_in = {8'h00, 8'h00, cfg_vc, DT_VSS};
            ST_HSS:      ecc_header_in = {8'h00, 8'h00, cfg_vc, DT_HSS};
            ST_EMIT_HDR: ecc_header_in = {emit_wc[15:8], emit_wc[7:0], cfg_vc, emit_dt};
            default:     ecc_header_in = 24'h000000;
        endcase
    end

    // Short packet byte mux
    reg [7:0] short_byte;
    always @(*) begin
        case (hdr_idx)
            3'd0: short_byte = {cfg_vc, short_dt}; 3'd1: short_byte = 8'h00;
            3'd2: short_byte = 8'h00;              3'd3: short_byte = ecc_out;
            default: short_byte = 8'h00;
        endcase
    end

    // Long header byte mux
    reg [7:0] long_hdr_byte;
    always @(*) begin
        case (hdr_idx)
            3'd0: long_hdr_byte = {cfg_vc, emit_dt}; 3'd1: long_hdr_byte = emit_wc[7:0];
            3'd2: long_hdr_byte = emit_wc[15:8];     3'd3: long_hdr_byte = ecc_out;
            default: long_hdr_byte = 8'h00;
        endcase
    end

    //=========================================================================
    // Main FSM
    //=========================================================================
    integer i;

    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; fifo_rd_en <= 1'b0;
            pkt_data <= 8'd0; pkt_valid <= 1'b0; pkt_sop <= 1'b0; pkt_eop <= 1'b0;
            lb_wr_idx <= 12'd0; num_segments <= 4'd0; cur_seg <= 4'd0;
            seg_build_idx <= 4'd0; line_has_sof <= 1'b0; prev_tier <= 2'b00;
            hdr_idx <= 3'd0; emit_pix_idx <= 12'd0; emit_byte_idx <= 2'd0;
            emit_bpp <= 2'd3; emit_dt <= DT_RGB888; emit_wc <= 16'd0;
            short_dt <= 6'h00; crc_init_r <= 1'b0; crc_en_r <= 1'b0; crc_data_r <= 8'd0;
            for (i = 0; i < MAX_SEGMENTS; i = i + 1) begin
                seg_start[i] <= 12'd0; seg_count[i] <= 12'd0; seg_tier[i] <= 2'b00;
            end
        end else begin
            fifo_rd_en <= 1'b0; pkt_valid <= 1'b0; pkt_sop <= 1'b0; pkt_eop <= 1'b0;
            crc_init_r <= 1'b0; crc_en_r <= 1'b0;

            case (state)

            // ─── IDLE: wait for FIFO data ───
            ST_IDLE: begin
                if (!fifo_empty) begin
                    lb_wr_idx <= 12'd0; seg_build_idx <= 4'd0;
                    num_segments <= 4'd0; line_has_sof <= 1'b0;
                    state <= ST_BUFFER;
                end
            end

            // ─── BUFFER: fill line buffer, build segment table ───
            ST_BUFFER: begin
                if (!fifo_empty) begin
                    // Write pixel to line buffer
                    line_buf[lb_wr_idx] <= {fifo_tier, fifo_pixel};

                    // Capture SOF
                    if (fifo_sof) line_has_sof <= 1'b1;

                    // Build segment table
                    if (lb_wr_idx == 12'd0) begin
                        // First pixel: start first segment
                        seg_start[0] <= 12'd0;
                        seg_tier[0]  <= fifo_tier;
                        seg_count[0] <= 12'd1;
                        seg_build_idx <= 4'd0;
                        prev_tier    <= fifo_tier;
                    end
                    else if (fifo_tier != prev_tier && seg_build_idx < MAX_SEGMENTS - 1) begin
                        // Tier transition: finalize prev segment, start new
                        seg_count[seg_build_idx] <= lb_wr_idx - seg_start[seg_build_idx];
                        seg_build_idx <= seg_build_idx + 4'd1;
                        seg_start[seg_build_idx + 1] <= lb_wr_idx;
                        seg_tier[seg_build_idx + 1]  <= fifo_tier;
                        seg_count[seg_build_idx + 1] <= 12'd1;
                        prev_tier <= fifo_tier;
                    end
                    else begin
                        seg_count[seg_build_idx] <= seg_count[seg_build_idx] + 12'd1;
                    end

                    // Check EOL
                    if (fifo_eol) begin
                        // Finalize last segment's count
                        // (seg_count was being incremented, now add the final pixel)
                        // Actually seg_count already includes this pixel from above
                        num_segments <= seg_build_idx + 4'd1;
                        cur_seg <= 4'd0; hdr_idx <= 3'd0;
                        if (line_has_sof || fifo_sof) begin
                            short_dt <= DT_VSS; state <= ST_VSS;
                        end else begin
                            short_dt <= DT_HSS; state <= ST_HSS;
                        end
                    end

                    lb_wr_idx  <= lb_wr_idx + 12'd1;
                    fifo_rd_en <= 1'b1;
                end
            end

            // ─── VSS: 4-byte short packet ───
            ST_VSS: begin
                pkt_data <= short_byte; pkt_valid <= 1'b1;
                pkt_sop <= (hdr_idx == 3'd0); pkt_eop <= (hdr_idx == 3'd3);
                if (hdr_idx == 3'd3) begin
                    short_dt <= DT_HSS; hdr_idx <= 3'd0; state <= ST_HSS;
                end else hdr_idx <= hdr_idx + 3'd1;
            end

            // ─── HSS: 4-byte short packet ───
            ST_HSS: begin
                pkt_data <= short_byte; pkt_valid <= 1'b1;
                pkt_sop <= (hdr_idx == 3'd0); pkt_eop <= (hdr_idx == 3'd3);
                if (hdr_idx == 3'd3) begin
                    hdr_idx <= 3'd0; state <= ST_PREP_SEG;
                end else hdr_idx <= hdr_idx + 3'd1;
            end

            // ─── PREP_SEG: compute WC, init CRC ───
            ST_PREP_SEG: begin
                emit_dt  <= dt_from_tier(seg_tier[cur_seg]);
                emit_bpp <= bpp_from_tier(seg_tier[cur_seg]);
                if (seg_tier[cur_seg] == 2'b10)
                    emit_wc <= {4'd0, seg_count[cur_seg]} + {4'd0, seg_count[cur_seg]};
                else
                    emit_wc <= {4'd0, seg_count[cur_seg]} + {4'd0, seg_count[cur_seg]}
                             + {4'd0, seg_count[cur_seg]};
                emit_pix_idx  <= 12'd0; emit_byte_idx <= 2'd0;
                hdr_idx       <= 3'd0;
                crc_init_r    <= 1'b1;
                state         <= ST_EMIT_HDR;
            end

            // ─── EMIT_HDR: 4-byte long packet header ───
            ST_EMIT_HDR: begin
                pkt_data <= long_hdr_byte; pkt_valid <= 1'b1;
                pkt_sop <= (hdr_idx == 3'd0);
                if (hdr_idx == 3'd3) state <= ST_EMIT_DATA;
                else hdr_idx <= hdr_idx + 3'd1;
            end

            // ─── EMIT_DATA: serialize pixels from line buffer ───
            ST_EMIT_DATA: begin
                pkt_data <= cur_byte; pkt_valid <= 1'b1;
                crc_en_r <= 1'b1; crc_data_r <= cur_byte;

                if (emit_byte_idx == emit_bpp - 2'd1) begin
                    emit_byte_idx <= 2'd0;
                    if (emit_pix_idx == seg_count[cur_seg] - 12'd1) begin
                        hdr_idx <= 3'd0; state <= ST_EMIT_CRC;
                    end else
                        emit_pix_idx <= emit_pix_idx + 12'd1;
                end else
                    emit_byte_idx <= emit_byte_idx + 2'd1;
            end

            // ─── EMIT_CRC: 2-byte footer, pkt_eop on last byte ───
            ST_EMIT_CRC: begin
                pkt_data <= (hdr_idx == 3'd0) ? crc_out[7:0] : crc_out[15:8];
                pkt_valid <= 1'b1; pkt_eop <= (hdr_idx == 3'd1);
                if (hdr_idx == 3'd1) state <= ST_NEXT_SEG;
                else hdr_idx <= hdr_idx + 3'd1;
            end

            // ─── NEXT_SEG: more segments? ───
            ST_NEXT_SEG: begin
                if (cur_seg + 4'd1 < num_segments) begin
                    cur_seg <= cur_seg + 4'd1; state <= ST_PREP_SEG;
                end else
                    state <= ST_GAP;
            end

            // ─── GAP: inter-line pause ───
            ST_GAP: state <= ST_IDLE;

            default: state <= ST_IDLE;
            endcase
        end
    end

    //=========================================================================
    // SVA Assertions
    //=========================================================================
    `ifdef SIMULATION
    property p_eop_follows_sop;
        @(posedge byte_clk) disable iff (!rst_n)
        pkt_sop |-> ##[1:8192] pkt_eop;
    endproperty
    assert property (p_eop_follows_sop)
        else $error("PACKETIZER: pkt_eop never fired — D-PHY lockup risk");

    property p_no_read_empty;
        @(posedge byte_clk) disable iff (!rst_n)
        fifo_rd_en |-> !fifo_empty;
    endproperty
    assert property (p_no_read_empty)
        else $error("PACKETIZER: Read from empty FIFO");
    `endif

endmodule
