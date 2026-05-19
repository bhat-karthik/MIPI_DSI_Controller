//=============================================================================
// Module:      reset_sync
// Project:     MIPI DSI Host Controller for Foveated Rendering Systems
// File:        reset_sync.v
// Author:      MTech VLSI Design Team
// Date:        April 2026
//
// Description:
//   Asynchronous reset synchronizer with parameterizable sync depth.
//   Implements the standard "async assert, sync de-assert" pattern that
//   prevents metastability when releasing reset.
//
//   WHY THIS EXISTS:
//   The raw rst_n input is an asynchronous signal — it can assert at any
//   time relative to the clock. Assertion (going low) is safe because all
//   flops have an async reset port that responds immediately. But
//   DE-ASSERTION (going high) is dangerous: if rst_n rises too close to a
//   clock edge, the reset-removal path violates setup/hold on the first
//   flop that sees it, causing metastability. The 2-FF synchronizer
//   ensures that rst_n de-assertion is aligned to a clock edge, and any
//   metastability on the first FF is resolved before the output is used.
//
//   USAGE:
//   Instantiate one per clock domain. In our design:
//     - One for pixel_clk domain (drives all pixel-domain modules)
//     - One for byte_clk domain (drives all byte-domain modules)
//
//   POWER DOMAIN NOTE:
//   In UPF, this module belongs to PD_ALWAYS_ON because reset control
//   must remain functional when switchable domains are powered off.
//
// Parameters:
//   SYNC_STAGES - Number of synchronizer flip-flops (default: 2)
//                 Increase to 3 for higher MTBF in safety-critical designs.
//                 Each stage adds one clock cycle of reset release latency.
//
// Timing:
//   Reset assertion:  Immediate (asynchronous, zero-cycle latency)
//   Reset de-assert:  SYNC_STAGES clock cycles after rst_n_async goes high
//
// Synthesis notes:
//   - Both FFs should be placed close together (same site row) to
//     minimize wire delay between stages. In Innovus, use:
//       set_dont_touch [get_cells sync_ff_reg*]
//     to prevent optimization from removing or merging these FFs.
//   - Genus will infer async reset on these FFs from the coding style.
//   - The STA tool (Tempus) should have a set_false_path on rst_n_async
//     since it's asynchronous by definition.
//
//=============================================================================

module reset_sync #(
    parameter SYNC_STAGES = 2    // Number of synchronizer stages (2 or 3)
) (
    input  wire clk,             // Clock to synchronize reset de-assertion to
    input  wire rst_n_async,     // Asynchronous active-low reset input
    output wire rst_n_sync       // Synchronized active-low reset output
);

    //=========================================================================
    // Synchronizer shift register
    //=========================================================================
    //
    // How it works:
    //   - When rst_n_async goes LOW (asserted): all FFs reset to 0
    //     immediately via their async reset port. rst_n_sync goes LOW
    //     in the same cycle. This is the "async assert" part.
    //
    //   - When rst_n_async goes HIGH (de-asserted): on the next rising
    //     edge of clk, sync_ff[0] captures 1'b1. If this capture causes
    //     metastability (because rst_n_async rose too close to the clock
    //     edge), sync_ff[0] may oscillate briefly but will settle to
    //     either 0 or 1 before the next clock edge.
    //
    //   - On the following clock edge, sync_ff[1] captures the now-stable
    //     value of sync_ff[0]. This is the "sync de-assert" part.
    //     rst_n_sync goes HIGH cleanly, aligned to the clock edge.
    //
    //   The input to the chain is tied to 1'b1 (not rst_n_async), because
    //   the async reset port handles assertion. The data path only carries
    //   the de-assertion event.
    //

    (* ASYNC_REG = "TRUE" *)    // Xilinx/Vivado attribute (ignored by Genus, harmless)
    reg [SYNC_STAGES-1:0] sync_ff;

    always @(posedge clk or negedge rst_n_async) begin
        if (!rst_n_async) begin
            // Async assert: all stages go to 0 immediately
            sync_ff <= {SYNC_STAGES{1'b0}};
        end else begin
            // Sync de-assert: shift 1'b1 through the chain
            sync_ff <= {sync_ff[SYNC_STAGES-2:0], 1'b1};
        end
    end

    // Output is the last stage of the synchronizer
    assign rst_n_sync = sync_ff[SYNC_STAGES-1];

    //=========================================================================
    // Parameter validation
    //=========================================================================
    initial begin
        if (SYNC_STAGES < 2) begin
            $fatal(1, "reset_sync: SYNC_STAGES must be >= 2 (got %0d)", SYNC_STAGES);
        end
    end

    //=========================================================================
    // SVA Assertions (for simulation — ignored by synthesis)
    //=========================================================================
    `ifdef SIMULATION
    // 1. When rst_n_async asserts (falls), rst_n_sync must assert within 1 cycle
    //    (actually 0 cycles due to async path, but we check next cycle to be safe)
    property p_async_assert;
        @(posedge clk) disable iff (1'b0)
        $fell(rst_n_async) |-> ##[0:1] !rst_n_sync;
    endproperty
    assert property (p_async_assert)
        else $error("ASSERT FAIL: rst_n_sync did not assert after rst_n_async fell");

    // 2. When rst_n_async de-asserts, rst_n_sync must follow after SYNC_STAGES cycles
    property p_sync_deassert;
        @(posedge clk) disable iff (!rst_n_async)
        $rose(rst_n_async) |-> ##SYNC_STAGES rst_n_sync;
    endproperty
    assert property (p_sync_deassert)
        else $error("ASSERT FAIL: rst_n_sync did not de-assert %0d cycles after rst_n_async", SYNC_STAGES);

    // 3. rst_n_sync should never be X during simulation (after initial time 0)
    property p_no_x;
        @(posedge clk)
        !$isunknown(rst_n_sync);
    endproperty
    assert property (p_no_x)
        else $error("ASSERT FAIL: rst_n_sync is X");
    `endif

endmodule
