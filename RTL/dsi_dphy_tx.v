//=============================================================================
// Module:      dsi_dphy_tx
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_dphy_tx.v
//
// Description:
//   D-PHY digital TX controller at the PPI (PHY-Protocol Interface) level.
//   Manages LP↔HS state transitions for all data lanes and the clock lane.
//   Does NOT implement analog drivers — outputs PPI-level control signals
//   that connect to an analog D-PHY macro in a real SoC.
//
//   DATA LANE FSM (shared across all active lanes — they transition together):
//     LP_STOP → HS_RQST → HS_PRPR → HS_ZERO → HS_DATA → HS_TRAIL → HS_EXIT → LP_STOP
//
//   CLOCK LANE FSM (continuous clock mode):
//     CLK_LP11 → CLK_RQST → CLK_PRPR → CLK_ZERO → CLK_RUN → CLK_TRAIL → CLK_EXIT → CLK_LP11
//     In continuous mode, clock stays in CLK_RUN as long as controller is active.
//
//   LP STATE ENCODING:
//     LP-11 = {Dp=1, Dn=1} = Stop state (idle)
//     LP-01 = {Dp=0, Dn=1} = HS Request
//     LP-00 = {Dp=0, Dn=0} = HS Prepare / Bridge
//     LP-10 = {Dp=1, Dn=0} = (Escape mode entry — not used in this impl)
//
//   TIMING: All timing parameters are configurable via registers in
//   byte_clk cycles. Default values correspond to 800 Mbps data rate.
//
// Power Domain: PD_DSI_CORE
//=============================================================================

module dsi_dphy_tx (
    input  wire        byte_clk,
    input  wire        rst_n,

    // Per-lane data from Lane Manager
    input  wire [7:0]  lane0_data,
    input  wire        lane0_valid,
    input  wire [7:0]  lane1_data,
    input  wire        lane1_valid,
    input  wire [7:0]  lane2_data,
    input  wire        lane2_valid,
    input  wire [7:0]  lane3_data,
    input  wire        lane3_valid,

    // HS handshake
    input  wire        hs_request,
    output reg         hs_active,

    // PHY timing configuration (in byte_clk cycles)
    input  wire [7:0]  cfg_t_lpx,
    input  wire [7:0]  cfg_t_hs_prepare,
    input  wire [7:0]  cfg_t_hs_zero,
    input  wire [7:0]  cfg_t_hs_trail,
    input  wire [7:0]  cfg_t_hs_exit,
    input  wire [7:0]  cfg_t_clk_prepare,
    input  wire [7:0]  cfg_t_clk_zero,
    input  wire [7:0]  cfg_t_clk_post,
    input  wire [7:0]  cfg_t_clk_trail,
    input  wire        cfg_cont_clk,
    input  wire [1:0]  cfg_lane_count,

    // PPI Output: Data lanes
    output reg  [7:0]  TxDataHS_0,
    output reg  [7:0]  TxDataHS_1,
    output reg  [7:0]  TxDataHS_2,
    output reg  [7:0]  TxDataHS_3,
    output reg  [3:0]  TxRequestHS,   // Per-lane HS request
    output reg  [1:0]  TxDataLP_0,    // {Dp, Dn} for lane 0
    output reg  [1:0]  TxDataLP_1,
    output reg  [1:0]  TxDataLP_2,
    output reg  [1:0]  TxDataLP_3,

    // PPI Output: Clock lane
    output reg         TxClkHS,       // Clock lane HS enable
    output reg  [1:0]  TxClkLP        // Clock lane LP state {Dp, Dn}
);

    //=========================================================================
    // LP State constants
    //=========================================================================
    localparam [1:0] LP_11 = 2'b11;  // Stop (idle)
    localparam [1:0] LP_01 = 2'b01;  // HS Request
    localparam [1:0] LP_00 = 2'b00;  // HS Prepare

    //=========================================================================
    // Data Lane FSM States
    //=========================================================================
    localparam [2:0] DL_LP_STOP  = 3'd0;
    localparam [2:0] DL_HS_RQST  = 3'd1;
    localparam [2:0] DL_HS_PRPR  = 3'd2;
    localparam [2:0] DL_HS_ZERO  = 3'd3;
    localparam [2:0] DL_HS_DATA  = 3'd4;
    localparam [2:0] DL_HS_TRAIL = 3'd5;
    localparam [2:0] DL_HS_EXIT  = 3'd6;

    reg [2:0]  dl_state;
    reg [7:0]  dl_timer;

    //=========================================================================
    // Clock Lane FSM States
    //=========================================================================
    localparam [2:0] CL_LP11     = 3'd0;
    localparam [2:0] CL_HS_RQST  = 3'd1;
    localparam [2:0] CL_HS_PRPR  = 3'd2;
    localparam [2:0] CL_HS_ZERO  = 3'd3;
    localparam [2:0] CL_HS_RUN   = 3'd4;
    localparam [2:0] CL_HS_TRAIL = 3'd5;
    localparam [2:0] CL_HS_EXIT  = 3'd6;

    reg [2:0]  cl_state;
    reg [7:0]  cl_timer;
    reg        clk_hs_active;  // Clock lane is in HS mode

    //=========================================================================
    // Lane enable mask
    //=========================================================================
    wire lane0_en = 1'b1;  // Lane 0 always active
    wire lane1_en = (cfg_lane_count >= 2'b01);
    wire lane2_en = (cfg_lane_count >= 2'b10);
    wire lane3_en = (cfg_lane_count >= 2'b10);

    //=========================================================================
    // Clock Lane FSM
    //=========================================================================
    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            cl_state      <= CL_LP11;
            cl_timer      <= 8'd0;
            clk_hs_active <= 1'b0;
            TxClkHS       <= 1'b0;
            TxClkLP       <= LP_11;
        end else begin
            case (cl_state)
                CL_LP11: begin
                    TxClkLP <= LP_11;
                    TxClkHS <= 1'b0;
                    clk_hs_active <= 1'b0;
                    if (hs_request) begin
                        cl_state <= CL_HS_RQST;
                        cl_timer <= cfg_t_lpx;
                        TxClkLP  <= LP_01;
                    end
                end

                CL_HS_RQST: begin
                    TxClkLP <= LP_01;
                    if (cl_timer == 8'd1) begin
                        cl_state <= CL_HS_PRPR;
                        cl_timer <= cfg_t_clk_prepare;
                        TxClkLP  <= LP_00;
                    end else begin
                        cl_timer <= cl_timer - 8'd1;
                    end
                end

                CL_HS_PRPR: begin
                    TxClkLP <= LP_00;
                    if (cl_timer == 8'd1) begin
                        cl_state <= CL_HS_ZERO;
                        cl_timer <= cfg_t_clk_zero;
                        TxClkHS  <= 1'b1;  // Start HS clock
                        TxClkLP  <= LP_00;
                    end else begin
                        cl_timer <= cl_timer - 8'd1;
                    end
                end

                CL_HS_ZERO: begin
                    TxClkHS <= 1'b1;
                    if (cl_timer == 8'd1) begin
                        cl_state      <= CL_HS_RUN;
                        clk_hs_active <= 1'b1;
                    end else begin
                        cl_timer <= cl_timer - 8'd1;
                    end
                end

                CL_HS_RUN: begin
                    TxClkHS       <= 1'b1;
                    clk_hs_active <= 1'b1;
                    // In continuous mode, stay here until hs_request drops
                    // AND data lanes are back in LP
                    if (!hs_request && !cfg_cont_clk && dl_state == DL_LP_STOP) begin
                        cl_state <= CL_HS_TRAIL;
                        cl_timer <= cfg_t_clk_trail;
                    end
                end

                CL_HS_TRAIL: begin
                    TxClkHS       <= 1'b1;
                    clk_hs_active <= 1'b0;
                    if (cl_timer == 8'd1) begin
                        cl_state <= CL_HS_EXIT;
                        cl_timer <= cfg_t_hs_exit;
                        TxClkHS  <= 1'b0;
                        TxClkLP  <= LP_11;
                    end else begin
                        cl_timer <= cl_timer - 8'd1;
                    end
                end

                CL_HS_EXIT: begin
                    TxClkLP <= LP_11;
                    TxClkHS <= 1'b0;
                    if (cl_timer == 8'd1) begin
                        cl_state <= CL_LP11;
                    end else begin
                        cl_timer <= cl_timer - 8'd1;
                    end
                end

                default: cl_state <= CL_LP11;
            endcase
        end
    end

    //=========================================================================
    // Data Lane FSM (shared — all active lanes transition together)
    //=========================================================================
    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            dl_state   <= DL_LP_STOP;
            dl_timer   <= 8'd0;
            hs_active  <= 1'b0;
            TxRequestHS<= 4'b0000;
            TxDataHS_0 <= 8'd0;
            TxDataHS_1 <= 8'd0;
            TxDataHS_2 <= 8'd0;
            TxDataHS_3 <= 8'd0;
            TxDataLP_0 <= LP_11;
            TxDataLP_1 <= LP_11;
            TxDataLP_2 <= LP_11;
            TxDataLP_3 <= LP_11;
        end else begin
            case (dl_state)
                DL_LP_STOP: begin
                    TxDataLP_0 <= LP_11;
                    TxDataLP_1 <= LP_11;
                    TxDataLP_2 <= LP_11;
                    TxDataLP_3 <= LP_11;
                    TxRequestHS<= 4'b0000;
                    hs_active  <= 1'b0;

                    // Wait for HS request AND clock lane to be in HS mode
                    if (hs_request && clk_hs_active) begin
                        dl_state <= DL_HS_RQST;
                        dl_timer <= cfg_t_lpx;
                        // Set LP-01 on active lanes
                        TxDataLP_0 <= LP_01;
                        if (lane1_en) TxDataLP_1 <= LP_01;
                        if (lane2_en) TxDataLP_2 <= LP_01;
                        if (lane3_en) TxDataLP_3 <= LP_01;
                    end
                end

                DL_HS_RQST: begin
                    if (dl_timer == 8'd1) begin
                        dl_state <= DL_HS_PRPR;
                        dl_timer <= cfg_t_hs_prepare;
                        TxDataLP_0 <= LP_00;
                        if (lane1_en) TxDataLP_1 <= LP_00;
                        if (lane2_en) TxDataLP_2 <= LP_00;
                        if (lane3_en) TxDataLP_3 <= LP_00;
                    end else begin
                        dl_timer <= dl_timer - 8'd1;
                    end
                end

                DL_HS_PRPR: begin
                    if (dl_timer == 8'd1) begin
                        dl_state    <= DL_HS_ZERO;
                        dl_timer    <= cfg_t_hs_zero;
                        TxRequestHS <= {lane3_en, lane2_en, lane1_en, lane0_en};
                        // Drive HS-0 (all zeros) on data lanes
                        TxDataHS_0 <= 8'h00;
                        TxDataHS_1 <= 8'h00;
                        TxDataHS_2 <= 8'h00;
                        TxDataHS_3 <= 8'h00;
                    end else begin
                        dl_timer <= dl_timer - 8'd1;
                    end
                end

                DL_HS_ZERO: begin
                    TxDataHS_0 <= 8'h00;
                    TxDataHS_1 <= 8'h00;
                    TxDataHS_2 <= 8'h00;
                    TxDataHS_3 <= 8'h00;
                    if (dl_timer == 8'd1) begin
                        dl_state  <= DL_HS_DATA;
                        hs_active <= 1'b1;
                    end else begin
                        dl_timer <= dl_timer - 8'd1;
                    end
                end

                DL_HS_DATA: begin
                    hs_active <= 1'b1;
                    // Forward lane data from Lane Manager
                    TxDataHS_0 <= lane0_valid ? lane0_data : 8'h00;
                    TxDataHS_1 <= lane1_valid ? lane1_data : 8'h00;
                    TxDataHS_2 <= lane2_valid ? lane2_data : 8'h00;
                    TxDataHS_3 <= lane3_valid ? lane3_data : 8'h00;

                    // Transition to trail when hs_request drops
                    if (!hs_request) begin
                        dl_state  <= DL_HS_TRAIL;
                        dl_timer  <= cfg_t_hs_trail;
                        hs_active <= 1'b0;
                    end
                end

                DL_HS_TRAIL: begin
                    // Drive known pattern during trail
                    TxDataHS_0 <= 8'hFF;  // Alternating for receiver sync
                    TxDataHS_1 <= 8'hFF;
                    TxDataHS_2 <= 8'hFF;
                    TxDataHS_3 <= 8'hFF;
                    hs_active  <= 1'b0;

                    if (dl_timer == 8'd1) begin
                        dl_state    <= DL_HS_EXIT;
                        dl_timer    <= cfg_t_hs_exit;
                        TxRequestHS <= 4'b0000;
                        TxDataLP_0  <= LP_11;
                        if (lane1_en) TxDataLP_1 <= LP_11;
                        if (lane2_en) TxDataLP_2 <= LP_11;
                        if (lane3_en) TxDataLP_3 <= LP_11;
                    end else begin
                        dl_timer <= dl_timer - 8'd1;
                    end
                end

                DL_HS_EXIT: begin
                    TxRequestHS <= 4'b0000;
                    TxDataLP_0  <= LP_11;
                    TxDataLP_1  <= LP_11;
                    TxDataLP_2  <= LP_11;
                    TxDataLP_3  <= LP_11;

                    if (dl_timer == 8'd1) begin
                        dl_state <= DL_LP_STOP;
                    end else begin
                        dl_timer <= dl_timer - 8'd1;
                    end
                end

                default: dl_state <= DL_LP_STOP;
            endcase
        end
    end

    //=========================================================================
    // SVA Assertions
    //=========================================================================
    `ifdef SIMULATION
    // hs_active should only be asserted in HS_DATA state
    property p_hs_active_only_in_data;
        @(posedge byte_clk) disable iff (!rst_n)
        hs_active |-> (dl_state == DL_HS_DATA);
    endproperty
    assert property (p_hs_active_only_in_data)
        else $error("DPHY: hs_active asserted outside HS_DATA state");

    // LP lines should be LP-11 in LP_STOP state
    property p_lp_stop_is_lp11;
        @(posedge byte_clk) disable iff (!rst_n)
        (dl_state == DL_LP_STOP) |-> (TxDataLP_0 == LP_11);
    endproperty
    assert property (p_lp_stop_is_lp11)
        else $error("DPHY: LP_STOP but TxDataLP_0 != LP-11");
    `endif

endmodule
