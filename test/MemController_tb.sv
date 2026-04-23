`timescale 1ns/1ps

`include "../hw/MemController.sv"
`include "../hw/MemTypes.sv"
`include "MockMem.sv"
`include "TestBench.sv"

module MemController_tb;

  logic clk;
  logic rst;

  MemReq  dut_req;
  logic   dut_req_val;

  MemResp dut_resp;
  logic   dut_resp_val;

  MemReq  interconnect_req;
  logic   interconnect_req_val;
  logic   interconnect_req_rdy;

  MemResp interconnect_resp;
  logic   interconnect_resp_val;
  logic   interconnect_resp_rdy;

  MockMem #(
    .WIDTH(32),
    .SIZE      (1024)
   ) mockMem (
    .rst         (rst),
    .clk         (clk),
    .mem_req     (interconnect_req),
    .mem_req_val (interconnect_req_val),
    .mem_req_rdy (interconnect_req_rdy),
    .mem_resp    (interconnect_resp),
    .mem_resp_val(interconnect_resp_val),
    .mem_resp_rdy(interconnect_resp_rdy)
  );
  
  MemController dut (
    .clk              (clk),
    .rst              (rst),
    .req              (dut_req),
    .req_val          (dut_req_val),
    .resp             (dut_resp),
    .resp_val         (dut_resp_val),
    .mem_req_cnct     (interconnect_req),
    .mem_req_cnct_val (interconnect_req_val),
    .mem_req_cnct_rdy (interconnect_req_rdy),
    .mem_resp_cnct    (interconnect_resp),
    .mem_resp_cnct_val(interconnect_resp_val),
    .mem_resp_cnct_rdy(interconnect_resp_rdy)
  );

  always #5 clk = ~clk;

  TestBench tb;
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0);

    // Initial Testbench Setup

    tb = new();

    // Reset the DUT and Memory
    clk = 0;
    rst = 1;

    repeat (2) @(negedge clk);

    // TEST: Basic Read Write
    rst = 0;

    dut_req = MemReq'{WR, 10, 'hDEADBEEF};
    dut_req_val = 1;

    do @(negedge clk); while (!dut_resp_val);

    dut_req = MemReq'{RD, 10, 0};
    dut_req_val = 1;

    do @(negedge clk); while (!dut_resp_val);

    tb.check_32b_eq(dut_resp.data, 'hDEADBEEF);

    if (tb.all_checks_passed()) begin
      $display("\033[32mALL TESTS PASSED\033[39m");
    end else begin
      $display("\033[31mTESTS FAILED\033[39m");
    end
    $finish;
  end

endmodule: MemController_tb
