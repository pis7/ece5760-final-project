`timescale 1ns/1ps
`include "../hw/SPMV.sv"

module MockMem #(
  parameter DATA_WIDTH=32, parameter SIZE
)(
  input  logic      i_rst,
  input  logic      i_clk,

  input  mem_req_t  i_mem_req,
  input  logic      i_mem_req_val,
  output logic      o_mem_req_rdy,

  output mem_resp_t o_mem_resp,
  output logic      o_mem_resp_val,
  input  logic      i_mem_resp_rdy
);
  typedef enum {
    IDLE, DONE
  } state_e;


  logic [DATA_WIDTH-1:0] data[SIZE];
  state_e state;
  state_e next_state;

  always_comb begin
    case (state)
      IDLE:    next_state = i_mem_req_val  ? DONE : IDLE;
      DONE:    next_state = i_mem_resp_rdy ? IDLE : DONE;
      default: next_state = IDLE;
    endcase
  end

  always_ff @(i_clk) begin
    if (i_rst) state <= IDLE;
    else       state <= next_state;
  end

  task handle_mem_req (
    input  mem_req_t  req,
    input  logic      req_val,

    output mem_resp_t resp
  );
    if (req_val && req.ty == WR) begin
      data[req.idx] <= req.data;
    end

    case (i_mem_req.ty)
      RD: resp <= req_val ? mem_resp_t'{RD, data[req.idx]} : 0;
      WR: resp <= req_val ? mem_resp_t'{WR, 0}             : 0;
      default:;
    endcase
  endtask: handle_mem_req

  always_ff @(i_clk) begin
    case (state)
      IDLE: handle_mem_req (i_mem_req, i_mem_req_val, o_mem_resp);
      DONE:;
      default:;
    endcase
  end

  always_comb begin
    o_mem_req_rdy  = state == IDLE;
    o_mem_resp_val = state == DONE;
  end

endmodule: MockMem

task check_32b_eq (
  input logic [31:0] expected,
  input logic [31:0] received
);
  if (expected !== received) begin
    $display("ERROR: expected %d but recieved %d", expected, received);
  end
endtask: check_32b_eq

module SPMV_tb;

  logic rst;
  logic clk;
  logic go;
  logic done;
  logic [31:0] num_rows;
  logic [31:0] num_non_zero;

  mem_req_t  q_mem_req;
  logic      q_mem_req_val;
  logic      q_mem_req_rdy;

  mem_resp_t q_mem_resp;
  logic      q_mem_resp_val;
  logic      q_mem_resp_rdy;

  mem_req_t  x_mem_req;
  logic      x_mem_req_val;
  logic      x_mem_req_rdy;

  mem_resp_t x_mem_resp;
  logic      x_mem_resp_val;
  logic      x_mem_resp_rdy;

  mem_req_t  c_mem_req;
  logic      c_mem_req_val;
  logic      c_mem_req_rdy;

  mem_resp_t c_mem_resp;
  logic      c_mem_resp_val;
  logic      c_mem_resp_rdy;

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) q_mem (
    .i_rst         (rst),
    .i_clk         (clk),
    .i_mem_req     (q_mem_req),
    .i_mem_req_val (q_mem_req_val),
    .o_mem_req_rdy (q_mem_req_rdy),
    .o_mem_resp    (q_mem_resp),
    .o_mem_resp_val(q_mem_resp_val),
    .i_mem_resp_rdy(q_mem_resp_rdy)
  );

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) x_mem (
    .i_rst         (rst),
    .i_clk         (clk),
    .i_mem_req     (x_mem_req),
    .i_mem_req_val (x_mem_req_val),
    .o_mem_req_rdy (x_mem_req_rdy),
    .o_mem_resp    (x_mem_resp),
    .o_mem_resp_val(x_mem_resp_val),
    .i_mem_resp_rdy(x_mem_resp_rdy)
  );

  MockMem #(
    .DATA_WIDTH(32),
    .SIZE      (1024)
   ) c_mem (
    .i_rst         (rst),
    .i_clk         (clk),
    .i_mem_req     (c_mem_req),
    .i_mem_req_val (c_mem_req_val),
    .o_mem_req_rdy (c_mem_req_rdy),
    .o_mem_resp    (c_mem_resp),
    .o_mem_resp_val(c_mem_resp_val),
    .i_mem_resp_rdy(c_mem_resp_rdy)
  );

  SPMV dut (
    .i_rst           (rst),
    .i_clk           (clk),
    .i_go            (go),
    .o_done          (done),
    .i_num_rows      (num_rows),
    .i_num_non_zeros (num_non_zero),
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
    // Reset the DUT
    clk = 0;
    rst = 1;
    go  = 0;
    num_rows = 0;
    num_non_zero = 0;

    repeat (5) @(posedge clk);

    // TEST: latch the values of num_rows and num_non_zero
    rst = 0;
    num_rows = 10;
    num_non_zero = 8;
    @(negedge clk);
    check_32b_eq(dut.num_rows, 10);
    check_32b_eq(dut.num_non_zeros, 8);

    $finish;
  end

endmodule: SPMV_tb
