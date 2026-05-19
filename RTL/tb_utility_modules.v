//=============================================================================
// Testbench: tb_utility_modules
// Project:   MIPI DSI Host Controller for Foveated Rendering Systems
// File:      tb_utility_modules.v
// Author:    MTech VLSI Design Team
// Date:      April 2026
//
// Description:
//   Self-checking testbench validating all three utility modules:
//     1. reset_sync   — async assert / sync de-assert behavior
//     2. dsi_ecc      — ECC generation and syndrome checking
//     3. dsi_crc16    — CRC-16 accumulation over byte sequences
//
//   This testbench is SELF-CHECKING: it compares outputs against
//   pre-computed golden values and reports PASS/FAIL with a summary.
//   Run with: xrun -v93 -sv tb_utility_modules.v reset_sync.v
//             dsi_ecc.v dsi_crc16.v +define+SIMULATION +access+r
//
//   Or with NCLaunch: add all 4 files, define SIMULATION macro.
//
//=============================================================================

`timescale 1ns / 1ps
`define SIMULATION

module tb_utility_modules;

    //=========================================================================
    // Test counters
    //=========================================================================
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;

    task check;
        input [255:0] test_name;  // Test identifier string
        input [31:0]  expected;
        input [31:0]  actual;
        input [31:0]  width;      // Number of valid bits to compare
        reg   [31:0]  mask;
        begin
            test_count = test_count + 1;
            mask = (1 << width) - 1;
            if ((actual & mask) === (expected & mask)) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0s: expected=0x%0h, got=0x%0h",
                         test_name, expected & mask, actual & mask);
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0s: expected=0x%0h, got=0x%0h  <<<",
                         test_name, expected & mask, actual & mask);
            end
        end
    endtask

    //=========================================================================
    // Clock generation
    //=========================================================================
    reg clk;
    initial clk = 0;
    always #2.5 clk = ~clk;  // 200 MHz (5 ns period)

    //=========================================================================
    //
    // TEST GROUP 1: reset_sync
    //
    //=========================================================================
    reg  rst_n_async;
    wire rst_n_sync;

    reset_sync #(
        .SYNC_STAGES(2)
    ) u_rst_sync (
        .clk         (clk),
        .rst_n_async (rst_n_async),
        .rst_n_sync  (rst_n_sync)
    );

    // Also test with 3-stage variant
    wire rst_n_sync_3;
    reset_sync #(
        .SYNC_STAGES(3)
    ) u_rst_sync_3 (
        .clk         (clk),
        .rst_n_async (rst_n_async),
        .rst_n_sync  (rst_n_sync_3)
    );

    //=========================================================================
    //
    // TEST GROUP 2: dsi_ecc
    //
    //=========================================================================
    reg  [23:0] ecc_header;
    reg  [7:0]  ecc_rx_in;
    wire [7:0]  ecc_generated;
    wire [7:0]  ecc_syndrome;
    wire        ecc_err_single;
    wire        ecc_err_double;

    dsi_ecc u_ecc (
        .header_in  (ecc_header),
        .ecc_out    (ecc_generated),
        .ecc_rx     (ecc_rx_in),
        .syndrome   (ecc_syndrome),
        .err_single (ecc_err_single),
        .err_double (ecc_err_double)
    );

    //=========================================================================
    //
    // TEST GROUP 3: dsi_crc16
    //
    //=========================================================================
    reg         crc_init;
    reg         crc_en;
    reg  [7:0]  crc_data;
    wire [15:0] crc_out;

    dsi_crc16 u_crc (
        .clk      (clk),
        .rst_n    (rst_n_async),
        .crc_init (crc_init),
        .crc_en   (crc_en),
        .data_in  (crc_data),
        .crc_out  (crc_out)
    );

    //=========================================================================
    // CRC helper task: feed one byte into CRC and wait one cycle
    //=========================================================================
    task crc_feed_byte;
        input [7:0] byte_val;
        begin
            @(posedge clk);
            crc_en   <= 1'b1;
            crc_data <= byte_val;
            @(posedge clk);
            crc_en   <= 1'b0;
        end
    endtask

    // CRC init task
    task crc_reset;
        begin
            @(posedge clk);
            crc_init <= 1'b1;
            @(posedge clk);
            crc_init <= 1'b0;
        end
    endtask

    //=========================================================================
    //
    // MAIN TEST SEQUENCE
    //
    //=========================================================================
    initial begin
        // Initialize signals
        rst_n_async = 1'b1;
        ecc_header  = 24'h000000;
        ecc_rx_in   = 8'h00;
        crc_init    = 1'b0;
        crc_en      = 1'b0;
        crc_data    = 8'h00;

        $display("");
        $display("==========================================================");
        $display("  MIPI DSI Utility Modules — Self-Checking Testbench");
        $display("==========================================================");
        $display("");

        //=====================================================================
        // GROUP 1: Reset Synchronizer Tests
        //=====================================================================
        $display("--- GROUP 1: Reset Synchronizer Tests ---");

        // Test 1.1: Async assert — rst_n_sync should go low immediately
        #10;
        rst_n_async = 1'b0;  // Assert reset
        #1;                   // Wait tiny amount (not a full clock cycle)
        check("RST 1.1: Async assert (immediate)", 0, rst_n_sync, 1);

        // Test 1.2: rst_n_sync should STAY low while rst_n_async is low
        @(posedge clk); @(posedge clk);
        check("RST 1.2: Held during assert", 0, rst_n_sync, 1);

        // Test 1.3: Sync de-assert — should take exactly 2 clocks for 2-stage
        @(posedge clk);
        rst_n_async = 1'b1;  // De-assert reset
        // After 0 clocks: rst_n_sync should still be 0
        #0.1;
        check("RST 1.3a: Not yet de-asserted (0 cyc)", 0, rst_n_sync, 1);

        // After 1 clock: still 0 (first FF captured, second hasn't)
        @(posedge clk); #0.1;
        check("RST 1.3b: Not yet de-asserted (1 cyc)", 0, rst_n_sync, 1);

        // After 2 clocks: should be 1 now
        @(posedge clk); #0.1;
        check("RST 1.3c: De-asserted after 2 cyc", 1, rst_n_sync, 1);

        // Test 1.4: 3-stage variant should take 3 clocks
        rst_n_async = 1'b0;
        #1;
        check("RST 1.4a: 3-stage async assert", 0, rst_n_sync_3, 1);
        @(posedge clk);
        rst_n_async = 1'b1;
        @(posedge clk); #0.1;
        check("RST 1.4b: 3-stage after 1 cyc", 0, rst_n_sync_3, 1);
        @(posedge clk); #0.1;
        check("RST 1.4c: 3-stage after 2 cyc", 0, rst_n_sync_3, 1);
        @(posedge clk); #0.1;
        check("RST 1.4d: 3-stage after 3 cyc", 1, rst_n_sync_3, 1);

        // Ensure reset is de-asserted for remaining tests
        rst_n_async = 1'b1;
        #20;

        $display("");

        //=====================================================================
        // GROUP 2: ECC Generator Tests
        //=====================================================================
        $display("--- GROUP 2: ECC Generator Tests ---");

        // Test 2.1: VSS packet (DI=0x01, Data0=0x00, Data1=0x00)
        // D[23:0] = 24'h000001. Only D[0]=1.
        // P0=1, P1=1, P2=1, P3=0, P4=0, P5=0 → ECC = 8'h07
        ecc_header = 24'h000001;
        ecc_rx_in  = 8'h00;    // Not checking syndrome yet
        #1;
        check("ECC 2.1: VSS (0x000001)", 32'h07, ecc_generated, 8);

        // Test 2.2: HSS packet (DI=0x21, Data0=0x00, Data1=0x00)
        // D[23:0] = 24'h000021. D[0]=1, D[5]=1.
        // P0=D[0]^D[5]=0, P1=D[0]=1, P2=D[0]^D[5]=0,
        // P3=0, P4=D[5]=1, P5=0 → ECC = 8'h12
        ecc_header = 24'h000021;
        #1;
        check("ECC 2.2: HSS (0x000021)", 32'h12, ecc_generated, 8);

        // Test 2.3: VSE packet (DI=0x11, Data0=0x00, Data1=0x00)
        // D[23:0] = 24'h000011. D[0]=1, D[4]=1.
        // P0=D[0]^D[4]=0, P1=D[0]^D[4]=0, P2=D[0]=1,
        // P3=0, P4=D[4]=1, P5=0 → ECC = 8'h14
        ecc_header = 24'h000011;
        #1;
        check("ECC 2.3: VSE (0x000011)", 32'h14, ecc_generated, 8);

        // Test 2.4: HSE packet (DI=0x31, Data0=0x00, Data1=0x00)
        // D[23:0] = 24'h000031. D[0]=1, D[4]=1, D[5]=1.
        // P0=D[0]^D[4]^D[5]=1, P1=D[0]^D[4]=0, P2=D[0]^D[5]=0,
        // P3=0, P4=D[4]^D[5]=0, P5=0 → ECC = 8'h01
        ecc_header = 24'h000031;
        #1;
        check("ECC 2.4: HSE (0x000031)", 32'h01, ecc_generated, 8);

        // Test 2.5: RGB888 long packet header (DI=0x3E, WC=5760=0x1680)
        // D[23:0] = {0x16, 0x80, 0x3E} = 24'h16803E
        // Set bits: D[1..5], D[15], D[17], D[18], D[20]
        // P0=D[1]^D[2]^D[4]^D[5]^D[20] = 5 → 1
        // P1=D[1]^D[3]^D[4]^D[17]^D[20] = 5 → 1
        // P2=D[2]^D[3]^D[5]^D[15]^D[18]^D[20] = 6 → 0
        // P3=D[1]^D[2]^D[3]^D[15]^D[20] = 5 → 1
        // P4=D[4]^D[5]^D[17]^D[18]^D[20] = 5 → 1
        // P5=D[15]^D[17]^D[18] = 3 → 1
        // ECC = {00, P5=1, P4=1, P3=1, P2=0, P1=1, P0=1} = 8'h3B
        ecc_header = 24'h16803E;
        #1;
        check("ECC 2.5: RGB888 WC=5760 (0x16803E)", 32'h3B, ecc_generated, 8);

        // Test 2.6: All zeros
        ecc_header = 24'h000000;
        #1;
        check("ECC 2.6: All zeros (0x000000)", 32'h00, ecc_generated, 8);

        // Test 2.7: All ones
        // D[23:0] = 24'hFFFFFF — all 24 bits set
        // Each parity bit XORs its subset; count 1s in each:
        // P0 has 14 terms → all 14 are 1 → XOR of 14 ones = 0
        // P1 has 14 terms → all 14 are 1 → 0
        // P2 has 13 terms → all 13 are 1 → 1
        // P3 has 13 terms → all 13 are 1 → 1
        // P4 has 13 terms → all 13 are 1 → 1
        // P5 has 13 terms → all 13 are 1 → 1
        // ECC = {00, 1, 1, 1, 1, 0, 0} = 8'h3C
        ecc_header = 24'hFFFFFF;
        #1;
        check("ECC 2.7: All ones (0xFFFFFF)", 32'h3C, ecc_generated, 8);

        // Test 2.8: Syndrome check — no error case
        // Generate ECC for VSS, then feed it back as ecc_rx
        ecc_header = 24'h000001;
        #1;
        ecc_rx_in = ecc_generated;  // Feed back generated ECC
        #1;
        check("ECC 2.8: Syndrome (no error)", 32'h00, ecc_syndrome, 8);
        check("ECC 2.8b: err_single=0", 0, ecc_err_single, 1);
        check("ECC 2.8c: err_double=0", 0, ecc_err_double, 1);

        // Test 2.9: Syndrome check — single-bit error in D[0]
        // Flip D[0] in header, keep original ECC
        ecc_header = 24'h000001;
        #1;
        ecc_rx_in = ecc_generated;
        ecc_header = 24'h000000;  // D[0] flipped from 1→0
        #1;
        check("ECC 2.9: err_single detected", 1, ecc_err_single, 1);
        check("ECC 2.9b: err_double=0", 0, ecc_err_double, 1);

        // Test 2.10: Syndrome check — double-bit error
        ecc_header = 24'h000001;
        #1;
        ecc_rx_in = ecc_generated;
        ecc_header = 24'h000004;  // D[0] flipped to 0, D[2] flipped to 1 (2 bits changed)
        #1;
        check("ECC 2.10: err_double detected", 1, ecc_err_double, 1);
        check("ECC 2.10b: err_single=0", 0, ecc_err_single, 1);

        $display("");

        //=====================================================================
        // GROUP 3: CRC-16 Generator Tests
        //=====================================================================
        $display("--- GROUP 3: CRC-16 Generator Tests ---");

        // Test 3.1: After reset, CRC should be seed (0xFFFF)
        @(posedge clk);
        check("CRC 3.1: Initial seed after reset", 32'hFFFF, crc_out, 16);

        // Test 3.2: After explicit init, CRC should be seed
        // First feed some junk to dirty the register
        crc_en = 1'b1; crc_data = 8'hAA;
        @(posedge clk);
        crc_en = 1'b0;
        @(posedge clk);
        // Now init
        crc_reset();
        @(posedge clk);
        check("CRC 3.2: Seed after crc_init", 32'hFFFF, crc_out, 16);

        // Test 3.3: Single byte 0x00
        // Pre-computed: CRC of [0x00] with seed 0xFFFF, poly 0x8408 = 0x0F87
        //
        // Derivation (step-by-step for documentation):
        //   seed = FFFF, byte = 00
        //   bit0: crc[0]=1 ^ d[0]=0 = 1 → (FFFF>>1) ^ 8408 = 7FFF ^ 8408 = FBF7
        //   bit1: crc[0]=1 ^ d[1]=0 = 1 → (FBF7>>1) ^ 8408 = 7DFB ^ 8408 = F9F3
        //   bit2: crc[0]=1 ^ d[2]=0 = 1 → (F9F3>>1) ^ 8408 = 7CF9 ^ 8408 = F8F1
        //   bit3: crc[0]=1 ^ d[3]=0 = 1 → (F8F1>>1) ^ 8408 = 7C78 ^ 8408 = F870
        //   bit4: crc[0]=0 ^ d[4]=0 = 0 → F870>>1 = 7C38
        //   bit5: crc[0]=0 ^ d[5]=0 = 0 → 7C38>>1 = 3E1C
        //   bit6: crc[0]=0 ^ d[6]=0 = 0 → 3E1C>>1 = 1F0E
        //   bit7: crc[0]=0 ^ d[7]=0 = 0 → 1F0E>>1 = 0F87
        //
        crc_reset();
        crc_feed_byte(8'h00);
        @(posedge clk);
        check("CRC 3.3: Single byte 0x00", 32'h0F87, crc_out, 16);

        // Test 3.4: Single byte 0xFF
        // Pre-computed: CRC of [0xFF] with seed 0xFFFF, poly 0x8408
        //   bit0: crc[0]=1 ^ d[0]=1 = 0 → FFFF>>1 = 7FFF
        //   bit1: crc[0]=1 ^ d[1]=1 = 0 → 7FFF>>1 = 3FFF
        //   bit2: crc[0]=1 ^ d[2]=1 = 0 → 3FFF>>1 = 1FFF
        //   bit3: crc[0]=1 ^ d[3]=1 = 0 → 1FFF>>1 = 0FFF
        //   bit4: crc[0]=1 ^ d[4]=1 = 0 → 0FFF>>1 = 07FF
        //   bit5: crc[0]=1 ^ d[5]=1 = 0 → 07FF>>1 = 03FF
        //   bit6: crc[0]=1 ^ d[6]=1 = 0 → 03FF>>1 = 01FF
        //   bit7: crc[0]=1 ^ d[7]=1 = 0 → 01FF>>1 = 00FF
        //
        crc_reset();
        crc_feed_byte(8'hFF);
        @(posedge clk);
        check("CRC 3.4: Single byte 0xFF", 32'h00FF, crc_out, 16);

        // Test 3.5: Two bytes [0x00, 0x00] — black pixel (R=G=0)
        // After first byte 0x00: CRC = 0x0F87 (from test 3.3)
        // Second byte 0x00 starting from CRC = 0x0F87:
        //   bit0: crc[0]=1 ^ d[0]=0 = 1 → (0F87>>1) ^ 8408 = 07C3 ^ 8408 = 834B
        //   ... (many steps, pre-computed result below)
        // Pre-computed with Python: CRC of [0x00, 0x00] = 0xF0B8
        crc_reset();
        crc_feed_byte(8'h00);
        crc_feed_byte(8'h00);
        @(posedge clk);
        check("CRC 3.5: Two bytes [0x00,0x00]", 32'hF0B8, crc_out, 16);

        // Test 3.6: Three bytes [0x00, 0x00, 0x00] — single black RGB888 pixel
        // Pre-computed with Python: CRC of [0x00, 0x00, 0x00] = 0x3933
        crc_reset();
        crc_feed_byte(8'h00);
        crc_feed_byte(8'h00);
        crc_feed_byte(8'h00);
        @(posedge clk);
        check("CRC 3.6: Three bytes [0x00,0x00,0x00]", 32'h3933, crc_out, 16);

        // Test 3.7: Three bytes [0xFF, 0xFF, 0xFF] — single white RGB888 pixel
        // Pre-computed with Python: CRC of [0xFF, 0xFF, 0xFF] = 0x0F78
        crc_reset();
        crc_feed_byte(8'hFF);
        crc_feed_byte(8'hFF);
        crc_feed_byte(8'hFF);
        @(posedge clk);
        check("CRC 3.7: Three bytes [0xFF,0xFF,0xFF]", 32'h0F78, crc_out, 16);

        // Test 3.8: Verify init works mid-computation
        // Start computing CRC, then re-init, should get back to seed
        crc_reset();
        crc_feed_byte(8'hAA);
        crc_feed_byte(8'h55);
        // CRC is now some non-seed value
        @(posedge clk);
        if (crc_out == 16'hFFFF)
            $display("[WARN] CRC 3.8: CRC happened to equal seed (unlikely, check test)");
        // Now re-init
        crc_reset();
        @(posedge clk);
        check("CRC 3.8: Re-init mid-computation", 32'hFFFF, crc_out, 16);

        // Test 3.9: CRC hold — value should not change when crc_en is low
        crc_reset();
        crc_feed_byte(8'h42);
        @(posedge clk);
        begin : crc_hold_test
            reg [15:0] saved_crc;
            saved_crc = crc_out;
            // Wait 5 cycles with crc_en=0
            repeat(5) @(posedge clk);
            check("CRC 3.9: Hold value (5 idle cycles)", saved_crc, crc_out, 16);
        end

        //=====================================================================
        // SUMMARY
        //=====================================================================
        $display("");
        $display("==========================================================");
        $display("  TEST SUMMARY");
        $display("==========================================================");
        $display("  Total tests:  %0d", test_count);
        $display("  Passed:       %0d", pass_count);
        $display("  Failed:       %0d", fail_count);
        $display("==========================================================");
        if (fail_count == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> %0d TESTS FAILED — FIX BEFORE PROCEEDING <<<", fail_count);
        $display("==========================================================");
        $display("");

        #100;
        $finish;
    end

    //=========================================================================
    // Timeout watchdog
    //=========================================================================
    initial begin
        #100000;
        $display("[TIMEOUT] Testbench timed out after 100 us");
        $finish;
    end

    //=========================================================================
    // Waveform dump (for debugging)
    //=========================================================================
    initial begin
        $dumpfile("tb_utility_modules.vcd");
        $dumpvars(0, tb_utility_modules);
    end

endmodule
