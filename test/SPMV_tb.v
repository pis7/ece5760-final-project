`timescale 1ns/1ps
`include "../hw/SPMV.sv"
`include "MockMem.sv"
`include "TestBench.sv"

module SPMV_tb;
  logic rst;
  logic clk;
  logic go;
  logic done;
  logic [31:0] num_rows;
  logic [31:0] num_non_zero;

  MemReq  q_mem_req;
  logic      q_mem_req_val;
  logic      q_mem_req_rdy;

  MemResp q_mem_resp;
  logic      q_mem_resp_val;
  logic      q_mem_resp_rdy;

  MemReq  x_mem_req;
  logic      x_mem_req_val;
  logic      x_mem_req_rdy;

  MemResp x_mem_resp;
  logic      x_mem_resp_val;
  logic      x_mem_resp_rdy;

  MemReq  c_mem_req;
  logic      c_mem_req_val;
  logic      c_mem_req_rdy;

  MemResp c_mem_resp;
  logic      c_mem_resp_val;
  logic      c_mem_resp_rdy;

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) q_mem (
    .rst         (rst),
    .clk         (clk),
    .mem_req     (q_mem_req),
    .mem_req_val (q_mem_req_val),
    .mem_req_rdy (q_mem_req_rdy),
    .mem_resp    (q_mem_resp),
    .mem_resp_val(q_mem_resp_val),
    .mem_resp_rdy(q_mem_resp_rdy)
  );

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) x_mem (
    .rst         (rst),
    .clk         (clk),
    .mem_req     (x_mem_req),
    .mem_req_val (x_mem_req_val),
    .mem_req_rdy (x_mem_req_rdy),
    .mem_resp    (x_mem_resp),
    .mem_resp_val(x_mem_resp_val),
    .mem_resp_rdy(x_mem_resp_rdy)
  );

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) c_mem (
    .rst         (rst),
    .clk         (clk),
    .mem_req     (c_mem_req),
    .mem_req_val (c_mem_req_val),
    .mem_req_rdy (c_mem_req_rdy),
    .mem_resp    (c_mem_resp),
    .mem_resp_val(c_mem_resp_val),
    .mem_resp_rdy(c_mem_resp_rdy)
  );

  SPMV dut (
    .rst             (rst),
    .clk             (clk),
    .go              (go),
    .done            (done),
    .num_rows        (num_rows),
    .num_non_zeros   (num_non_zero),
    .o_q_mem_req     (q_mem_req),
    .o_q_mem_req_val (q_mem_req_val),
    .i_q_mem_req_rdy (q_mem_req_rdy),
    .i_q_mem_resp    (q_mem_resp),
    .i_q_mem_resp_val(q_mem_resp_val),
    .o_q_mem_resp_rdy(q_mem_resp_rdy),
    .o_x_mem_req     (x_mem_req),
    .o_x_mem_req_val (x_mem_req_val),
    .i_x_mem_req_rdy (x_mem_req_rdy),
    .i_x_mem_resp    (x_mem_resp),
    .i_x_mem_resp_val(x_mem_resp_val),
    .o_x_mem_resp_rdy(x_mem_resp_rdy),
    .o_c_mem_req     (c_mem_req),
    .o_c_mem_req_val (c_mem_req_val),
    .i_c_mem_req_rdy (c_mem_req_rdy),
    .i_c_mem_resp    (c_mem_resp),
    .i_c_mem_resp_val(c_mem_resp_val),
    .o_c_mem_resp_rdy(c_mem_resp_rdy)
  );

  always #5 clk = ~clk;

  initial begin
    // Inital Testbench Setup
    TestBench tb;
    tb = new();

    // Reset the DUT
    clk = 0;
    rst = 1;
    go  = 0;
    num_rows = 0;
    num_non_zero = 0;

    repeat (5) @(negedge clk);

    // TEST: latch the values of num_rows and num_non_zero
    rst = 0;
    go = 1;
    num_rows = 10;
    num_non_zero = 8;
    @(negedge clk);
    tb.check_32b_eq(dut.num_rows, 10);
    tb.check_32b_eq(dut.num_non_zeros, 8);

    if (tb.all_checks_passed()) begin
      $display("ALL TESTS PASSED");
    end else begin
      $display("TESTS FAILED");
    end
    
    $finish;
  end

endmodule: SPMV_tb
