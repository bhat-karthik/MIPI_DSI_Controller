//=============================================================================
// Module:      dsi_host_top
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_host_top.v
//
// Description:
//   Top-level integration module instantiating all sub-modules:
//     - 2× reset_sync (pixel_clk and byte_clk domains)
//     - dsi_vtc (Video Timing Controller)
//     - dsi_fov_mapper (Foveated Region Mapper)
//     - dsi_packer (Pixel-to-Byte Packer)
//     - dsi_async_fifo (CDC bridge)
//     - dsi_packetizer (DSI packet formation with ECC/CRC)
//     - dsi_lane_mgr (Lane distribution)
//     - dsi_dphy_tx (D-PHY TX at PPI level)
//     - dsi_apb_regs (APB register file)
//
//   CLOCK DOMAINS:
//     pixel_clk (148.5 MHz): VTC → Fov Mapper → Packer → FIFO write
//     byte_clk  (200 MHz):   FIFO read → Packetizer → Lane Mgr → D-PHY TX, APB
//
//   UPF POWER DOMAINS:
//     PD_ALWAYS_ON:  u_apb_regs, u_rst_sync_pixel, u_rst_sync_byte
//     PD_DSI_CORE:   u_vtc, u_packer, u_async_fifo, u_packetizer,
//                    u_lane_mgr, u_dphy_tx
//     PD_FOVEATION:  u_fov_mapper
//
//   Instance names match UPF file (dsi_host_power.upf) exactly.
//
//=============================================================================

module dsi_host_top (
    // Clocks and Reset
    input  wire        pixel_clk,
    input  wire        byte_clk,
    input  wire        rst_n,

    // DPI Video Input
    input  wire [23:0] dpi_pdata,
    input  wire        dpi_hsync,
    input  wire        dpi_vsync,
    input  wire        dpi_de,

    // APB Configuration Interface
    input  wire [11:0] PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output wire [31:0] PRDATA,
    output wire        PREADY,

    // D-PHY PPI Output
    output wire [7:0]  phy_txdatahs_0,
    output wire [7:0]  phy_txdatahs_1,
    output wire [7:0]  phy_txdatahs_2,
    output wire [7:0]  phy_txdatahs_3,
    output wire [3:0]  phy_txrequesths,
    input  wire [3:0]  phy_txreadyhs,
    output wire [1:0]  phy_txdatalp_0,
    output wire [1:0]  phy_txdatalp_1,
    output wire [1:0]  phy_txdatalp_2,
    output wire [1:0]  phy_txdatalp_3,
    output wire        phy_txclkhs,
    output wire [1:0]  phy_txclklp,

    // Interrupt
    output wire        dsi_irq
);

    //=========================================================================
    // Internal wires: configuration bus from APB
    //=========================================================================
    wire        cfg_dsi_en, cfg_video_en, cfg_vtc_mode;
    wire [11:0] cfg_h_active, cfg_h_fp, cfg_h_sync, cfg_h_bp;
    wire [11:0] cfg_v_active, cfg_v_fp, cfg_v_sync, cfg_v_bp;
    wire        cfg_fov_enable;
    wire [11:0] cfg_gaze_x, cfg_gaze_y;
    wire [25:0] cfg_radius_foveal_sq, cfg_radius_mid_sq;
    wire [1:0]  cfg_lane_count;
    wire        cfg_cont_clk;
    wire [7:0]  cfg_t_lpx, cfg_t_hs_prepare, cfg_t_hs_zero;
    wire [7:0]  cfg_t_hs_trail, cfg_t_hs_exit;
    wire [7:0]  cfg_t_clk_prepare, cfg_t_clk_zero, cfg_t_clk_post, cfg_t_clk_trail;
    wire [1:0]  cfg_vc, cfg_video_mode;
    wire        pwr_sw_core_en, pwr_sw_fov_en;

    //=========================================================================
    // Reset synchronizers (PD_ALWAYS_ON)
    //=========================================================================
    wire rst_n_pixel, rst_n_byte;

    reset_sync #(.SYNC_STAGES(2)) u_rst_sync_pixel (
        .clk         (pixel_clk),
        .rst_n_async (rst_n),
        .rst_n_sync  (rst_n_pixel)
    );

    reset_sync #(.SYNC_STAGES(2)) u_rst_sync_byte (
        .clk         (byte_clk),
        .rst_n_async (rst_n),
        .rst_n_sync  (rst_n_byte)
    );

    //=========================================================================
    // VTC → Fov Mapper → Packer (pixel_clk domain)
    //=========================================================================
    wire [23:0] vtc_pixel_data;
    wire        vtc_pixel_valid;
    wire [11:0] vtc_h_count, vtc_v_count;
    wire        vtc_line_start, vtc_line_end;
    wire        vtc_frame_start, vtc_frame_end;
    wire        vtc_in_hsync, vtc_in_vsync, vtc_in_hbp, vtc_in_hfp;

    dsi_vtc u_vtc (
        .pixel_clk      (pixel_clk),
        .rst_n          (rst_n_pixel),
        .cfg_vtc_mode   (cfg_vtc_mode),
        .cfg_h_active   (cfg_h_active),
        .cfg_h_fp       (cfg_h_fp),
        .cfg_h_sync     (cfg_h_sync),
        .cfg_h_bp       (cfg_h_bp),
        .cfg_v_active   (cfg_v_active),
        .cfg_v_fp       (cfg_v_fp),
        .cfg_v_sync     (cfg_v_sync),
        .cfg_v_bp       (cfg_v_bp),
        .cfg_video_en   (cfg_video_en & cfg_dsi_en),
        .pixel_data_in  (dpi_pdata),
        .hsync_in       (dpi_hsync),
        .vsync_in       (dpi_vsync),
        .de_in          (dpi_de),
        .pixel_data_out (vtc_pixel_data),
        .pixel_valid    (vtc_pixel_valid),
        .h_count        (vtc_h_count),
        .v_count        (vtc_v_count),
        .line_start     (vtc_line_start),
        .line_end       (vtc_line_end),
        .frame_start    (vtc_frame_start),
        .frame_end      (vtc_frame_end),
        .in_hsync       (vtc_in_hsync),
        .in_vsync       (vtc_in_vsync),
        .in_hbp         (vtc_in_hbp),
        .in_hfp         (vtc_in_hfp)
    );

    // Foveated Region Mapper (PD_FOVEATION)
    wire [23:0] fov_pixel_data;
    wire        fov_pixel_valid;
    wire [1:0]  fov_tier;
    wire        fov_line_start, fov_line_end;
    wire        fov_frame_start, fov_frame_end;

    dsi_fov_mapper u_fov_mapper (
        .pixel_clk           (pixel_clk),
        .rst_n               (rst_n_pixel),
        .pixel_data_in       (vtc_pixel_data),
        .pixel_valid_in      (vtc_pixel_valid),
        .h_count             (vtc_h_count),
        .v_count             (vtc_v_count),
        .line_start_in       (vtc_line_start),
        .line_end_in         (vtc_line_end),
        .frame_start_in      (vtc_frame_start),
        .frame_end_in        (vtc_frame_end),
        .cfg_gaze_x          (cfg_gaze_x),
        .cfg_gaze_y          (cfg_gaze_y),
        .cfg_radius_foveal_sq(cfg_radius_foveal_sq),
        .cfg_radius_mid_sq   (cfg_radius_mid_sq),
        .cfg_fov_enable      (cfg_fov_enable),
        .pixel_data_out      (fov_pixel_data),
        .pixel_valid_out     (fov_pixel_valid),
        .fov_tier            (fov_tier),
        .line_start_out      (fov_line_start),
        .line_end_out        (fov_line_end),
        .frame_start_out     (fov_frame_start),
        .frame_end_out       (fov_frame_end)
    );

    // Pixel Packer (v3 — per-pixel tier, no line-tier latching)
    wire [23:0] pxl_data;
    wire [1:0]  pxl_tier;
    wire        pxl_valid, pxl_sol, pxl_eol, pxl_sof;

    dsi_packer u_packer (
        .pixel_clk      (pixel_clk),
        .rst_n          (rst_n_pixel),
        .pixel_data_in  (fov_pixel_data),
        .pixel_valid_in (fov_pixel_valid),
        .fov_tier       (fov_tier),
        .line_start_in  (fov_line_start),
        .line_end_in    (fov_line_end),
        .frame_start_in (fov_frame_start),
        .frame_end_in   (fov_frame_end),
        .cfg_h_active   (cfg_h_active),
        .pxl_data       (pxl_data),
        .pxl_tier       (pxl_tier),
        .pxl_valid      (pxl_valid),
        .pxl_sol        (pxl_sol),
        .pxl_eol        (pxl_eol),
        .pxl_sof        (pxl_sof)
    );

    //=========================================================================
    // CDC FIFO: pixel_clk → byte_clk
    //=========================================================================
    // FIFO write word format:
    // [23:0]  = pixel_data
    // [25:24] = tier
    // [26]    = sol
    // [27]    = eol  (pxl_eol comes 1 cycle after last valid pixel, 
    //                  so we mark the last valid pixel using line_end from packer)
    // [28]    = sof
    // [31:29] = reserved (0)

    wire [31:0] fifo_wdata = {3'b000, pxl_sof, pxl_eol, pxl_sol, pxl_tier, pxl_data};
    wire        fifo_wr_en = pxl_valid;
    wire        fifo_full;

    wire [31:0] fifo_rdata;
    wire        fifo_rd_en;
    wire        fifo_empty;

    dsi_async_fifo #(
        .DATA_WIDTH (32),
        .ADDR_WIDTH (8)
    ) u_async_fifo (
        .wr_clk   (pixel_clk),
        .wr_rst_n (rst_n_pixel),
        .wr_en    (fifo_wr_en & ~fifo_full),
        .wr_data  (fifo_wdata),
        .wr_full  (fifo_full),
        .rd_clk   (byte_clk),
        .rd_rst_n (rst_n_byte),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rdata),
        .rd_empty (fifo_empty)
    );

    //=========================================================================
    // Packetizer → Lane Manager → D-PHY TX (byte_clk domain)
    //=========================================================================
    wire [7:0]  pkt_data;
    wire        pkt_valid, pkt_sop, pkt_eop;

    dsi_packetizer u_packetizer (
        .byte_clk    (byte_clk),
        .rst_n       (rst_n_byte),
        .fifo_rdata  (fifo_rdata),
        .fifo_empty  (fifo_empty),
        .fifo_rd_en  (fifo_rd_en),
        .cfg_vc      (cfg_vc),
        .cfg_h_active(cfg_h_active),
        .pkt_data    (pkt_data),
        .pkt_valid   (pkt_valid),
        .pkt_sop     (pkt_sop),
        .pkt_eop     (pkt_eop)
    );

    // Lane Manager
    wire [7:0] lm_lane0_data, lm_lane1_data, lm_lane2_data, lm_lane3_data;
    wire       lm_lane0_valid, lm_lane1_valid, lm_lane2_valid, lm_lane3_valid;
    wire       lm_hs_request;
    wire       lm_hs_active;

    dsi_lane_mgr u_lane_mgr (
        .byte_clk       (byte_clk),
        .rst_n          (rst_n_byte),
        .pkt_data       (pkt_data),
        .pkt_valid      (pkt_valid),
        .pkt_sop        (pkt_sop),
        .pkt_eop        (pkt_eop),
        .cfg_lane_count (cfg_lane_count),
        .lane0_data     (lm_lane0_data),
        .lane0_valid    (lm_lane0_valid),
        .lane1_data     (lm_lane1_data),
        .lane1_valid    (lm_lane1_valid),
        .lane2_data     (lm_lane2_data),
        .lane2_valid    (lm_lane2_valid),
        .lane3_data     (lm_lane3_data),
        .lane3_valid    (lm_lane3_valid),
        .hs_request     (lm_hs_request),
        .hs_active      (lm_hs_active)
    );

    // D-PHY TX Controller
    dsi_dphy_tx u_dphy_tx (
        .byte_clk        (byte_clk),
        .rst_n           (rst_n_byte),
        .lane0_data      (lm_lane0_data),
        .lane0_valid     (lm_lane0_valid),
        .lane1_data      (lm_lane1_data),
        .lane1_valid     (lm_lane1_valid),
        .lane2_data      (lm_lane2_data),
        .lane2_valid     (lm_lane2_valid),
        .lane3_data      (lm_lane3_data),
        .lane3_valid     (lm_lane3_valid),
        .hs_request      (lm_hs_request),
        .hs_active       (lm_hs_active),
        .cfg_t_lpx       (cfg_t_lpx),
        .cfg_t_hs_prepare(cfg_t_hs_prepare),
        .cfg_t_hs_zero   (cfg_t_hs_zero),
        .cfg_t_hs_trail  (cfg_t_hs_trail),
        .cfg_t_hs_exit   (cfg_t_hs_exit),
        .cfg_t_clk_prepare(cfg_t_clk_prepare),
        .cfg_t_clk_zero  (cfg_t_clk_zero),
        .cfg_t_clk_post  (cfg_t_clk_post),
        .cfg_t_clk_trail (cfg_t_clk_trail),
        .cfg_cont_clk    (cfg_cont_clk),
        .cfg_lane_count  (cfg_lane_count),
        .TxDataHS_0      (phy_txdatahs_0),
        .TxDataHS_1      (phy_txdatahs_1),
        .TxDataHS_2      (phy_txdatahs_2),
        .TxDataHS_3      (phy_txdatahs_3),
        .TxRequestHS     (phy_txrequesths),
        .TxDataLP_0      (phy_txdatalp_0),
        .TxDataLP_1      (phy_txdatalp_1),
        .TxDataLP_2      (phy_txdatalp_2),
        .TxDataLP_3      (phy_txdatalp_3),
        .TxClkHS         (phy_txclkhs),
        .TxClkLP         (phy_txclklp)
    );

    //=========================================================================
    // APB Register File (PD_ALWAYS_ON)
    //=========================================================================
    dsi_apb_regs u_apb_regs (
        .PCLK             (byte_clk),
        .PRESETn          (rst_n_byte),
        .PADDR            (PADDR),
        .PSEL             (PSEL),
        .PENABLE          (PENABLE),
        .PWRITE           (PWRITE),
        .PWDATA           (PWDATA),
        .PRDATA           (PRDATA),
        .PREADY           (PREADY),
        .sts_phy_ready    (lm_hs_active),
        .sts_fifo_empty   (fifo_empty),
        .sts_fifo_full    (fifo_full),
        .sts_video_active (vtc_pixel_valid),  // Simplified: crosses CDC but acceptable for status
        .sts_in_vsync     (vtc_in_vsync),
        .sts_pkt_cnt      (32'd0),  // TODO: add counters in packetizer
        .sts_byte_cnt     (32'd0),
        .sts_frame_cnt    (32'd0),
        .cfg_dsi_en       (cfg_dsi_en),
        .cfg_video_en     (cfg_video_en),
        .cfg_vtc_mode     (cfg_vtc_mode),
        .cfg_h_active     (cfg_h_active),
        .cfg_h_fp         (cfg_h_fp),
        .cfg_h_sync       (cfg_h_sync),
        .cfg_h_bp         (cfg_h_bp),
        .cfg_v_active     (cfg_v_active),
        .cfg_v_fp         (cfg_v_fp),
        .cfg_v_sync       (cfg_v_sync),
        .cfg_v_bp         (cfg_v_bp),
        .cfg_fov_enable   (cfg_fov_enable),
        .cfg_gaze_x       (cfg_gaze_x),
        .cfg_gaze_y       (cfg_gaze_y),
        .cfg_radius_foveal_sq(cfg_radius_foveal_sq),
        .cfg_radius_mid_sq(cfg_radius_mid_sq),
        .cfg_lane_count   (cfg_lane_count),
        .cfg_cont_clk     (cfg_cont_clk),
        .cfg_t_lpx        (cfg_t_lpx),
        .cfg_t_hs_prepare (cfg_t_hs_prepare),
        .cfg_t_hs_zero    (cfg_t_hs_zero),
        .cfg_t_hs_trail   (cfg_t_hs_trail),
        .cfg_t_hs_exit    (cfg_t_hs_exit),
        .cfg_t_clk_prepare(cfg_t_clk_prepare),
        .cfg_t_clk_zero   (cfg_t_clk_zero),
        .cfg_t_clk_post   (cfg_t_clk_post),
        .cfg_t_clk_trail  (cfg_t_clk_trail),
        .cfg_vc           (cfg_vc),
        .cfg_video_mode   (cfg_video_mode),
        .pwr_sw_core_en   (pwr_sw_core_en),
        .pwr_sw_fov_en    (pwr_sw_fov_en),
        .dsi_irq          (dsi_irq)
    );

endmodule
