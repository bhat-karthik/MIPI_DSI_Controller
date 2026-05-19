//=============================================================================
// Module:      dsi_apb_regs
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_apb_regs.v
//
// Description:
//   AMBA APB3 slave register file providing read/write access to all
//   configuration, status, debug, and power management registers.
//   4KB address space (12-bit PADDR), no wait states (PREADY always 1).
//
//   REGISTER MAP: See micro-architecture spec Section 8.3
//   Power Domain: PD_ALWAYS_ON (this module stays powered when core is off)
//
// Estimated gates: ~4,000
//=============================================================================

module dsi_apb_regs (
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire [11:0] PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,

    // Status inputs (from other modules)
    input  wire        sts_phy_ready,
    input  wire        sts_fifo_empty,
    input  wire        sts_fifo_full,
    input  wire        sts_video_active,
    input  wire        sts_in_vsync,
    input  wire [31:0] sts_pkt_cnt,
    input  wire [31:0] sts_byte_cnt,
    input  wire [31:0] sts_frame_cnt,

    // Configuration outputs (directly to other modules)
    output wire        cfg_dsi_en,
    output wire        cfg_video_en,
    output wire        cfg_vtc_mode,
    output wire [11:0] cfg_h_active,
    output wire [11:0] cfg_h_fp,
    output wire [11:0] cfg_h_sync,
    output wire [11:0] cfg_h_bp,
    output wire [11:0] cfg_v_active,
    output wire [11:0] cfg_v_fp,
    output wire [11:0] cfg_v_sync,
    output wire [11:0] cfg_v_bp,
    output wire        cfg_fov_enable,
    output wire [11:0] cfg_gaze_x,
    output wire [11:0] cfg_gaze_y,
    output wire [25:0] cfg_radius_foveal_sq,
    output wire [25:0] cfg_radius_mid_sq,
    output wire [1:0]  cfg_lane_count,
    output wire        cfg_cont_clk,
    output wire [7:0]  cfg_t_lpx,
    output wire [7:0]  cfg_t_hs_prepare,
    output wire [7:0]  cfg_t_hs_zero,
    output wire [7:0]  cfg_t_hs_trail,
    output wire [7:0]  cfg_t_hs_exit,
    output wire [7:0]  cfg_t_clk_prepare,
    output wire [7:0]  cfg_t_clk_zero,
    output wire [7:0]  cfg_t_clk_post,
    output wire [7:0]  cfg_t_clk_trail,
    output wire [1:0]  cfg_vc,
    output wire [1:0]  cfg_video_mode,

    // Power management outputs
    output wire        pwr_sw_core_en,
    output wire        pwr_sw_fov_en,

    // Interrupt
    output wire        dsi_irq
);

    //=========================================================================
    // APB ready: always 1 (no wait states)
    //=========================================================================
    assign PREADY = 1'b1;

    // Write enable: APB write phase
    wire apb_wr = PSEL & PENABLE & PWRITE;
    wire apb_rd = PSEL & ~PWRITE;  // Read phase (setup stage)

    //=========================================================================
    // Register storage
    //=========================================================================
    // Address offsets (word-aligned, lower 2 bits ignored)
    localparam A_CTRL         = 10'h000;
    localparam A_STATUS       = 10'h001;
    localparam A_INT_STATUS   = 10'h002;
    localparam A_INT_ENABLE   = 10'h003;
    localparam A_VID_HSIZE    = 10'h004;
    localparam A_VID_HSYNC    = 10'h005;
    localparam A_VID_VSIZE    = 10'h006;
    localparam A_VID_VSYNC    = 10'h007;
    localparam A_FOV_CTRL     = 10'h008;
    localparam A_FOV_GAZE     = 10'h009;
    localparam A_FOV_RAD_FOV  = 10'h00A;
    localparam A_FOV_RAD_MID  = 10'h00B;
    localparam A_PHY_CTRL     = 10'h00C;
    localparam A_PHY_TMR1     = 10'h00D;
    localparam A_PHY_TMR2     = 10'h00E;
    localparam A_PHY_CLK_TMR  = 10'h00F;
    localparam A_PKT_CTRL     = 10'h010;
    localparam A_DBG_FIFO     = 10'h020;
    localparam A_DBG_PKT_CNT  = 10'h021;
    localparam A_DBG_BYTE_CNT = 10'h022;
    localparam A_DBG_FRAME_CNT= 10'h023;
    localparam A_PWR_CTRL     = 10'h024;
    localparam A_PWR_STATUS   = 10'h025;

    wire [9:0] addr_word = PADDR[11:2];  // Word address

    // R/W registers
    reg [31:0] r_ctrl;
    reg [31:0] r_int_status;
    reg [31:0] r_int_enable;
    reg [31:0] r_vid_hsize;
    reg [31:0] r_vid_hsync;
    reg [31:0] r_vid_vsize;
    reg [31:0] r_vid_vsync;
    reg [31:0] r_fov_ctrl;
    reg [31:0] r_fov_gaze;
    reg [31:0] r_fov_rad_fov;
    reg [31:0] r_fov_rad_mid;
    reg [31:0] r_phy_ctrl;
    reg [31:0] r_phy_tmr1;
    reg [31:0] r_phy_tmr2;
    reg [31:0] r_phy_clk_tmr;
    reg [31:0] r_pkt_ctrl;
    reg [31:0] r_pwr_ctrl;

    //=========================================================================
    // Write logic
    //=========================================================================
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            r_ctrl        <= 32'h0000_0000;
            r_int_status  <= 32'h0000_0000;
            r_int_enable  <= 32'h0000_0000;
            r_vid_hsize   <= 32'h0000_0000;
            r_vid_hsync   <= 32'h0000_0000;
            r_vid_vsize   <= 32'h0000_0000;
            r_vid_vsync   <= 32'h0000_0000;
            r_fov_ctrl    <= 32'h0000_0000;
            r_fov_gaze    <= 32'h0000_0000;
            r_fov_rad_fov <= 32'h0000_2710;  // Default: 10000
            r_fov_rad_mid <= 32'h0001_5F90;  // Default: 90000
            r_phy_ctrl    <= 32'h0000_0002;  // Default: 4-lane, non-continuous
            r_phy_tmr1    <= 32'h0016_0A09;  // t_hs_zero=22, t_hs_prepare=10, t_lpx=9
            r_phy_tmr2    <= 32'h0014_0D00;  // t_hs_exit=20, t_hs_trail=13
            r_phy_clk_tmr <= 32'h0C10_3608;  // t_clk_trail=12, t_clk_post=16, t_clk_zero=54, t_clk_prepare=8
            r_pkt_ctrl    <= 32'h0000_0000;
            r_pwr_ctrl    <= 32'h0000_0000;
        end else if (apb_wr) begin
            case (addr_word)
                A_CTRL:        r_ctrl        <= PWDATA;
                A_INT_STATUS:  r_int_status  <= r_int_status & ~PWDATA; // W1C
                A_INT_ENABLE:  r_int_enable  <= PWDATA;
                A_VID_HSIZE:   r_vid_hsize   <= PWDATA;
                A_VID_HSYNC:   r_vid_hsync   <= PWDATA;
                A_VID_VSIZE:   r_vid_vsize   <= PWDATA;
                A_VID_VSYNC:   r_vid_vsync   <= PWDATA;
                A_FOV_CTRL:    r_fov_ctrl    <= PWDATA;
                A_FOV_GAZE:    r_fov_gaze    <= PWDATA;
                A_FOV_RAD_FOV: r_fov_rad_fov <= PWDATA;
                A_FOV_RAD_MID: r_fov_rad_mid <= PWDATA;
                A_PHY_CTRL:    r_phy_ctrl    <= PWDATA;
                A_PHY_TMR1:    r_phy_tmr1    <= PWDATA;
                A_PHY_TMR2:    r_phy_tmr2    <= PWDATA;
                A_PHY_CLK_TMR: r_phy_clk_tmr <= PWDATA;
                A_PKT_CTRL:    r_pkt_ctrl    <= PWDATA;
                A_PWR_CTRL:    r_pwr_ctrl    <= PWDATA;
                default: ;  // Ignore writes to read-only or invalid addresses
            endcase

            // Soft reset: self-clearing bit
            if (addr_word == A_CTRL && PWDATA[8])
                r_ctrl[8] <= 1'b0;
        end
    end

    //=========================================================================
    // Read logic
    //=========================================================================
    always @(*) begin
        PRDATA = 32'h0000_0000;
        if (apb_rd) begin
            case (addr_word)
                A_CTRL:         PRDATA = r_ctrl;
                A_STATUS:       PRDATA = {27'd0, sts_in_vsync, sts_video_active,
                                          sts_fifo_full, sts_fifo_empty, sts_phy_ready};
                A_INT_STATUS:   PRDATA = r_int_status;
                A_INT_ENABLE:   PRDATA = r_int_enable;
                A_VID_HSIZE:    PRDATA = r_vid_hsize;
                A_VID_HSYNC:    PRDATA = r_vid_hsync;
                A_VID_VSIZE:    PRDATA = r_vid_vsize;
                A_VID_VSYNC:    PRDATA = r_vid_vsync;
                A_FOV_CTRL:     PRDATA = r_fov_ctrl;
                A_FOV_GAZE:     PRDATA = r_fov_gaze;
                A_FOV_RAD_FOV:  PRDATA = r_fov_rad_fov;
                A_FOV_RAD_MID:  PRDATA = r_fov_rad_mid;
                A_PHY_CTRL:     PRDATA = r_phy_ctrl;
                A_PHY_TMR1:     PRDATA = r_phy_tmr1;
                A_PHY_TMR2:     PRDATA = r_phy_tmr2;
                A_PHY_CLK_TMR:  PRDATA = r_phy_clk_tmr;
                A_PKT_CTRL:     PRDATA = r_pkt_ctrl;
                A_DBG_PKT_CNT:  PRDATA = sts_pkt_cnt;
                A_DBG_BYTE_CNT: PRDATA = sts_byte_cnt;
                A_DBG_FRAME_CNT:PRDATA = sts_frame_cnt;
                A_PWR_CTRL:     PRDATA = r_pwr_ctrl;
                A_PWR_STATUS:   PRDATA = {30'd0, r_pwr_ctrl[1], r_pwr_ctrl[0]};
                default:        PRDATA = 32'h0000_0000;
            endcase
        end
    end

    //=========================================================================
    // Configuration output assignments
    //=========================================================================
    assign cfg_dsi_en        = r_ctrl[0];
    assign cfg_video_en      = r_ctrl[1];
    assign cfg_vtc_mode      = r_ctrl[3];

    assign cfg_h_active      = r_vid_hsize[11:0];
    assign cfg_h_bp          = r_vid_hsize[27:16];
    assign cfg_h_sync        = r_vid_hsync[11:0];
    assign cfg_h_fp          = r_vid_hsync[27:16];
    assign cfg_v_active      = r_vid_vsize[11:0];
    assign cfg_v_bp          = r_vid_vsize[27:16];
    assign cfg_v_sync        = r_vid_vsync[11:0];
    assign cfg_v_fp          = r_vid_vsync[27:16];

    assign cfg_fov_enable    = r_fov_ctrl[0];
    assign cfg_gaze_x        = r_fov_gaze[11:0];
    assign cfg_gaze_y        = r_fov_gaze[27:16];
    assign cfg_radius_foveal_sq = r_fov_rad_fov[25:0];
    assign cfg_radius_mid_sq   = r_fov_rad_mid[25:0];

    assign cfg_lane_count    = r_phy_ctrl[1:0];
    assign cfg_cont_clk      = r_phy_ctrl[2];
    assign cfg_t_lpx         = r_phy_tmr1[7:0];
    assign cfg_t_hs_prepare  = r_phy_tmr1[15:8];
    assign cfg_t_hs_zero     = r_phy_tmr1[23:16];
    assign cfg_t_hs_trail    = r_phy_tmr2[7:0];
    assign cfg_t_hs_exit     = r_phy_tmr2[15:8];
    assign cfg_t_clk_prepare = r_phy_clk_tmr[7:0];
    assign cfg_t_clk_zero    = r_phy_clk_tmr[15:8];
    assign cfg_t_clk_post    = r_phy_clk_tmr[23:16];
    assign cfg_t_clk_trail   = r_phy_clk_tmr[31:24];

    assign cfg_vc            = r_pkt_ctrl[3:2];
    assign cfg_video_mode    = r_pkt_ctrl[1:0];

    assign pwr_sw_core_en    = r_pwr_ctrl[0];
    assign pwr_sw_fov_en     = r_pwr_ctrl[1];

    // Interrupt: OR of (status & enable) bits
    assign dsi_irq = |(r_int_status & r_int_enable);

endmodule
