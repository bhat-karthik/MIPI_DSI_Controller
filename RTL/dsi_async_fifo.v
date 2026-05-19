//=============================================================================
// Module:      dsi_async_fifo
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_async_fifo.v
//
// Description:
//   Gray-code pointer asynchronous FIFO bridging pixel_clk (write) and
//   byte_clk (read) domains. Classic Cliff Cummings design pattern.
//
//   Write side (pixel_clk, 148.5 MHz): Packer writes formatted pixel entries
//   Read side  (byte_clk,  200 MHz):   Packetizer reads and serializes
//
//   The read side is faster, so the FIFO tends to run near-empty.
//   Depth of 256 absorbs bursty read pauses during packet header/CRC insertion.
//
//   FIFO Word Format (32 bits):
//     [23:0]  pixel_data  — Formatted pixel (RGB888/666/565)
//     [25:24] tier        — Foveation tier (00/01/10)
//     [26]    sol         — Start of line
//     [27]    eol         — End of line (last pixel flag)
//     [28]    sof         — Start of frame
//     [31:29] reserved
//
// Power Domain: PD_DSI_CORE (switchable)
//=============================================================================

module dsi_async_fifo #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8     // Depth = 2^8 = 256
) (
    // Write side (pixel_clk domain)
    input  wire                   wr_clk,
    input  wire                   wr_rst_n,
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   wr_full,

    // Read side (byte_clk domain)
    input  wire                   rd_clk,
    input  wire                   rd_rst_n,
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  rd_data,
    output wire                   rd_empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    //=========================================================================
    // Dual-port memory
    //=========================================================================
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    //=========================================================================
    // Write pointer (binary and Gray) — write clock domain
    //=========================================================================
    reg [ADDR_WIDTH:0] wr_ptr_bin;    // Extra bit for full/empty distinction
    reg [ADDR_WIDTH:0] wr_ptr_gray;

    wire [ADDR_WIDTH:0] wr_ptr_bin_next  = wr_ptr_bin + (wr_en & ~wr_full);
    wire [ADDR_WIDTH:0] wr_ptr_gray_next = wr_ptr_bin_next ^ (wr_ptr_bin_next >> 1);

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            wr_ptr_bin  <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
        end
    end

    // Memory write
    always @(posedge wr_clk) begin
        if (wr_en && !wr_full)
            mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
    end

    //=========================================================================
    // Read pointer (binary and Gray) — read clock domain
    //=========================================================================
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    reg [ADDR_WIDTH:0] rd_ptr_gray;

    wire [ADDR_WIDTH:0] rd_ptr_bin_next  = rd_ptr_bin + (rd_en & ~rd_empty);
    wire [ADDR_WIDTH:0] rd_ptr_gray_next = rd_ptr_bin_next ^ (rd_ptr_bin_next >> 1);

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
        end
    end

    // Memory read (combinational — read data available same cycle as rd_en)
    assign rd_data = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];

    //=========================================================================
    // Gray-code pointer synchronization across clock domains
    //=========================================================================

    // Sync write pointer to read clock domain (for empty detection)
    (* ASYNC_REG = "TRUE" *)
    reg [ADDR_WIDTH:0] wr_ptr_gray_rd_sync1, wr_ptr_gray_rd_sync2;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_ptr_gray_rd_sync1 <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray_rd_sync2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            wr_ptr_gray_rd_sync1 <= wr_ptr_gray;
            wr_ptr_gray_rd_sync2 <= wr_ptr_gray_rd_sync1;
        end
    end

    // Sync read pointer to write clock domain (for full detection)
    (* ASYNC_REG = "TRUE" *)
    reg [ADDR_WIDTH:0] rd_ptr_gray_wr_sync1, rd_ptr_gray_wr_sync2;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_ptr_gray_wr_sync1 <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray_wr_sync2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rd_ptr_gray_wr_sync1 <= rd_ptr_gray;
            rd_ptr_gray_wr_sync2 <= rd_ptr_gray_wr_sync1;
        end
    end

    //=========================================================================
    // Full and Empty flags
    //=========================================================================
    // Empty: write pointer (synced to rd domain) == read pointer
    assign rd_empty = (wr_ptr_gray_rd_sync2 == rd_ptr_gray);

    // Full: write pointer and synced read pointer differ in the top 2 bits
    // but match in all remaining bits (Gray-code full condition)
    assign wr_full = (wr_ptr_gray_next == {~rd_ptr_gray_wr_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                            rd_ptr_gray_wr_sync2[ADDR_WIDTH-2:0]});

endmodule
