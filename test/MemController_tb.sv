`timescale 1ns/1ps

`include "../hw/MemController.sv"
`include "MockMem.sv"
`include "../hw/SPMV.sv"
`include "TestBench.sv"

module MemController_tb;

  logic clk;
  logic rst;

  MemReq dut_req;
  logic  dut_req_val;

  MemResp dut_resp;
  logic   dut_resp_val;

  MemReq interconnect_req;
  logic  interconnect_req_val;
  logic  interconnect_req_rdy;

  MemResp interconnect_resp;
  logic  interconnect_resp_val;
  logic  interconnect_resp_rdy;

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) mockMem (
    .rst         (clk),
    .clk         (rst),
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

  initial begin
    // Initial Testbench Setup
    TestBench tb;
    tb = new();

    // Reset the DUT and Memory
    clk = 0;
    rst = 1;

    repeat (5) @(negedge clk);
  end

endmodule: MemController_tb
