//=============================================================================
// Module:      dsi_lane_mgr
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        dsi_lane_mgr.v
//
// Description:
//   Distributes packet bytes from the Packetizer across 1, 2, or 4 data
//   lanes in round-robin order per the MIPI DSI specification.
//
//   LANE DISTRIBUTION:
//     1-lane: All bytes → Lane 0
//     2-lane: Byte 0→L0, Byte 1→L1, Byte 2→L0, Byte 3→L1, ...
//     4-lane: Byte 0→L0, Byte 1→L1, Byte 2→L2, Byte 3→L3, ...
//
//   The lane counter resets to 0 at pkt_sop (start of each packet).
//   This ensures the first byte of every packet always goes to Lane 0.
//
//   HS MODE HANDSHAKE:
//   When the first pkt_valid arrives, the Lane Manager asserts hs_request.
//   The D-PHY TX goes through LP→HS transition and asserts hs_active.
//   Data is forwarded to lanes only when hs_active is high.
//   When no more packets are pending, hs_request is de-asserted, and
//   the D-PHY TX performs HS→LP transition.
//
// Power Domain: PD_DSI_CORE
//=============================================================================

module dsi_lane_mgr (
    input  wire        byte_clk,
    input  wire        rst_n,

    // Packet input from Packetizer
    input  wire [7:0]  pkt_data,
    input  wire        pkt_valid,
    input  wire        pkt_sop,
    input  wire        pkt_eop,

    // Configuration
    input  wire [1:0]  cfg_lane_count,   // 00=1 lane, 01=2 lanes, 10=4 lanes

    // Per-lane output to D-PHY TX
    output reg  [7:0]  lane0_data,
    output reg         lane0_valid,
    output reg  [7:0]  lane1_data,
    output reg         lane1_valid,
    output reg  [7:0]  lane2_data,
    output reg         lane2_valid,
    output reg  [7:0]  lane3_data,
    output reg         lane3_valid,

    // HS mode handshake with D-PHY TX
    output reg         hs_request,       // Request HS mode
    input  wire        hs_active         // D-PHY in HS mode, ready for data
);

    //=========================================================================
    // Lane counter: round-robin index, resets at SOP
    //=========================================================================
    reg [1:0] lane_sel;

    // Lane counter mask based on configuration
    wire [1:0] lane_mask = (cfg_lane_count == 2'b10) ? 2'b11 :  // 4-lane: mod 4
                           (cfg_lane_count == 2'b01) ? 2'b01 :  // 2-lane: mod 2
                                                       2'b00;   // 1-lane: always 0

    //=========================================================================
    // HS Request management
    //=========================================================================
    // Assert hs_request when data arrives, de-assert after last packet ends
    // and a brief idle period (to allow HS-Trail and HS-Exit)
    reg        in_packet;
    reg [3:0]  idle_cnt;

    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_request <= 1'b0;
            in_packet  <= 1'b0;
            idle_cnt   <= 4'd0;
        end else begin
            if (pkt_valid && pkt_sop) begin
                hs_request <= 1'b1;
                in_packet  <= 1'b1;
                idle_cnt   <= 4'd0;
            end else if (pkt_eop) begin
                in_packet <= 1'b0;
                idle_cnt  <= 4'd0;
            end else if (!in_packet && !pkt_valid) begin
                // Count idle cycles after packet end
                if (idle_cnt < 4'd15)
                    idle_cnt <= idle_cnt + 4'd1;
                if (idle_cnt == 4'd10)
                    hs_request <= 1'b0;  // De-assert after 10 idle cycles
            end
        end
    end

    //=========================================================================
    // Lane distribution logic
    //=========================================================================
    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_sel    <= 2'd0;
            lane0_data  <= 8'd0;
            lane0_valid <= 1'b0;
            lane1_data  <= 8'd0;
            lane1_valid <= 1'b0;
            lane2_data  <= 8'd0;
            lane2_valid <= 1'b0;
            lane3_data  <= 8'd0;
            lane3_valid <= 1'b0;
        end else begin
            // Default: no valid data
            lane0_valid <= 1'b0;
            lane1_valid <= 1'b0;
            lane2_valid <= 1'b0;
            lane3_valid <= 1'b0;

            // Reset lane counter at start of each packet
            if (pkt_sop && pkt_valid)
                lane_sel <= 2'd0;

            if (pkt_valid) begin
                // Distribute byte to selected lane
                case (lane_sel)
                    2'd0: begin lane0_data <= pkt_data; lane0_valid <= 1'b1; end
                    2'd1: begin lane1_data <= pkt_data; lane1_valid <= 1'b1; end
                    2'd2: begin lane2_data <= pkt_data; lane2_valid <= 1'b1; end
                    2'd3: begin lane3_data <= pkt_data; lane3_valid <= 1'b1; end
                endcase

                // Advance lane counter (wraps based on lane_mask)
                // For SOP: lane_sel was just set to 0, so first byte → Lane 0
                if (!pkt_sop)  // Don't advance on the SOP cycle itself
                    lane_sel <= (lane_sel + 2'd1) & lane_mask;
                else
                    lane_sel <= 2'd1 & lane_mask; // Next byte after SOP → Lane 1 (or wrap to 0)
            end
        end
    end

    //=========================================================================
    // SVA Assertions
    //=========================================================================
    `ifdef SIMULATION
    // In 1-lane mode, only lane0 should ever be valid
    property p_1lane_only_lane0;
        @(posedge byte_clk) disable iff (!rst_n)
        (cfg_lane_count == 2'b00) |->
            (!lane1_valid && !lane2_valid && !lane3_valid);
    endproperty
    assert property (p_1lane_only_lane0)
        else $error("LANE_MGR: Non-lane0 active in 1-lane mode");

    // In 2-lane mode, lanes 2 and 3 should never be valid
    property p_2lane_no_lane23;
        @(posedge byte_clk) disable iff (!rst_n)
        (cfg_lane_count == 2'b01) |->
            (!lane2_valid && !lane3_valid);
    endproperty
    assert property (p_2lane_no_lane23)
        else $error("LANE_MGR: Lane 2/3 active in 2-lane mode");
    `endif

endmodule
