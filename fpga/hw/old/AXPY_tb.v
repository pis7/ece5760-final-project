`timescale 1ns/1ps

module AXPY_tb;

    parameter DATA_LEN = 32;

    reg clk;
    reg rst;
    reg [31:0] n;

    reg  [DATA_LEN-1:0] alpha;
    reg  [DATA_LEN-1:0] x_i;
    reg  [DATA_LEN-1:0] y_i;

    wire [DATA_LEN-1:0] z_i;
    wire [31:0] compute_counter;

    reg axpy_req_val;
    wire axpy_req_rdy;

    reg axpy_resp_rdy;
    wire axpy_resp_val;

    wire done;

    // DUT
    AXPY #(DATA_LEN) dut (
        .clk             (clk),
        .rst             (rst),
        .n               (n),
        .alpha           (alpha),
        .x_i             (x_i),
        .y_i             (y_i),
        .z_i             (z_i),
        .compute_counter (compute_counter),
        .done            (done),
        .axpy_req_val    (axpy_req_val),
        .axpy_req_rdy    (axpy_req_rdy),
        .axpy_resp_rdy   (axpy_resp_rdy),
        .axpy_resp_val   (axpy_resp_val)
    );

    // Clock
    always #5 clk = ~clk;

    integer TEST_CASE_NUM;

    // ------------------------------------------------------------
    // Global vectors (avoid task argument arrays)
    // ------------------------------------------------------------
    integer vx [0:31];
    integer vy [0:31];
    integer expected [0:31];

    // ------------------------------------------------------------
    // TASK: send one element (safe handshake)
    // ------------------------------------------------------------
    task send_element;
        input [31:0] a;
        input [31:0] b;
        input [31:0] al;
    begin
        x_i = a;
        y_i = b;
        alpha = al;
        axpy_req_val = 1;

        if (axpy_req_rdy) begin 
            // If already see an input handshake, just process it and advance to next set of vector entries 
            @(posedge clk); 
        end else begin 
            // If we do not see a handshake immediately, wait until this happens before moving on 
            while (!(axpy_req_rdy)) @(posedge clk); 
        end

        axpy_req_val = 0;
    end
    endtask

    // ------------------------------------------------------------
    // TASK: run full test (NO array arguments)
    // ------------------------------------------------------------
    task run_axpy;
        input integer size;
        integer i;
    begin
        $display("\n========== TEST %0d ==========", TEST_CASE_NUM);

        n = size;
        axpy_resp_rdy = 1;

        for (i = 0; i < size; i = i + 1) begin

            send_element(vx[i], vy[i], alpha);

            // Expect that at this point, the module is in UPDATE_ENTRY
            @(negedge clk);

            if (z_i !== expected[i]) begin
                $display("FAIL T%0d i=%0d expected=%0d got=%0d",
                         TEST_CASE_NUM, i, expected[i], z_i);
            end else begin
                $display("PASS T%0d i=%0d = %0d",
                         TEST_CASE_NUM, i, z_i);
            end

            // Backpressure test
            axpy_resp_rdy = 0;
            repeat (3) @(posedge clk);

            if (!axpy_resp_val)
                $display("FAIL (Test %0d): result not held under backpressure", TEST_CASE_NUM);

            axpy_resp_rdy = 1;
            @(posedge clk); // Return to COMPUTE (or DONE if last entry)
        end

        // Expect that at this point, the module is in DONE
        @(negedge clk);

        if (!done)
            $display("FAIL: done not asserted");

        TEST_CASE_NUM = TEST_CASE_NUM + 1;
    end
    endtask

    // ------------------------------------------------------------
    // TESTS
    // ------------------------------------------------------------
    integer i;

    initial begin
        clk = 0;
        rst = 1;

        axpy_req_val = 0;
        axpy_resp_rdy = 1;

        alpha = 0;
        x_i = 0;
        y_i = 0;
        n = 0;

        TEST_CASE_NUM = 1;

        repeat (5) @(posedge clk);
        rst = 0;

        // --------------------------------------------------------
        // TEST 1: z = y + 3x
        // --------------------------------------------------------
        alpha = 3;
        vx[0]=1; vy[0]=10; expected[0]=13;
        vx[1]=2; vy[1]=20; expected[1]=26;
        vx[2]=3; vy[2]=30; expected[2]=39;
        vx[3]=4; vy[3]=40; expected[3]=52;

        run_axpy(4);

        // --------------------------------------------------------
        // TEST 2: zeros
        // --------------------------------------------------------
        alpha = 1;
        vx[0]=0; vy[0]=5; expected[0]=5;
        vx[1]=0; vy[1]=6; expected[1]=6;
        vx[2]=0; vy[2]=7; expected[2]=7;

        run_axpy(3);

        // --------------------------------------------------------
        // TEST 3: small case
        // --------------------------------------------------------
        alpha = 4;
        vx[0]=1; vy[0]=1; expected[0]=5;
        vx[1]=2; vy[1]=2; expected[1]=10;

        run_axpy(2);

        $display("\nALL AXPY TESTS COMPLETE");
        $finish;
    end

endmodule