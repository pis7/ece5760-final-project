`ifndef SPMV_SV_H
`define SPMV_SV_H

/* Harcode index widths because system verilog is dumb and doesn't support
 * paramaterized structs */
parameter IDX_WIDTH  = 32;
/* Harcode data widths because system verilog is dumb and doesn't support
 * paramaterized structs */
parameter DATA_WIDTH = 32;

typedef enum {
  RD, WR
} mem_cmd_ty_e;

typedef struct packed {
  mem_cmd_ty_e           ty;
  logic [IDX_WIDTH-1:0]  idx;
  logic [DATA_WIDTH-1:0] data;
} mem_req_t;

typedef struct packed {
  mem_cmd_ty_e           ty;
  logic [DATA_WIDTH-1:0] data;
} mem_resp_t;

/* SPMV performs a matrix muliplicaton Qx = c.
 * The module assumes Q is layed out using CSR format in the order data, column
 * index, and row index. Vectors are expected to be layed out with the lower
 * indexed vectors at lower memory addresses and the least memory address at
 * address 0.
 *
 * This module directly writes its result into the c memory, the lowest value
 * at address zero.
 *
 * The module begins reading from memory and computing the product the cycle
 * go is asserted. It assumes i_num_rows and i_num_non_zeros are valid that
 * cycle. Once the module finished writing to the c memory, it asserts done
 * for one cycle.
 */
module SPMV (
  input  logic        i_rst,
  input  logic        i_clk,

  input  logic        i_go,
  output logic        o_done,

  input  logic [31:0] i_num_rows,
  input  logic [31:0] i_num_non_zeros,

  output mem_req_t    o_q_mem_req,
  output logic        o_q_mem_req_val,
  input  logic        i_q_mem_req_rdy,

  input  mem_resp_t   i_q_mem_resp,
  input  logic        i_q_mem_resp_val,
  output logic        o_q_mem_resp_rdy,

  output mem_req_t    o_x_mem_req,
  output logic        o_x_mem_req_val,
  input  logic        i_x_mem_req_rdy,

  input  mem_resp_t   i_x_mem_resp,
  input  logic        i_x_mem_resp_val,
  output logic        o_x_mem_resp_rdy,

  output mem_req_t    o_c_mem_req,
  output logic        o_c_mem_req_val,
  input  logic        i_c_mem_req_rdy,

  input  mem_resp_t   i_c_mem_resp,
  input  logic        i_c_mem_resp_val,
  output logic        o_c_mem_resp_rdy
);
  typedef enum {
    IDLE
  } state_e;


  /* The module is implemented with a relatively straightforward state machine.
   */

  state_e state;
  state_e next_state;

  always_comb begin
    next_state = IDLE;
  end

  always_ff @(i_clk) begin
    if (i_rst) state <= IDLE;
    else       state <= next_state;
  end

  logic [31:0] num_rows;
  logic [31:0] num_non_zeros;

  always_ff @(i_clk) begin
    if (i_rst) begin
      num_rows      <= 0;
      num_non_zeros <= 0;
    end else if (i_go && state == IDLE) begin
      num_rows      <= i_num_rows;
      num_non_zeros <= i_num_non_zeros;
    end else begin
      num_rows      <= num_rows;
      num_non_zeros <= num_non_zeros;
    end
  end


endmodule: SPMV

`endif /* SPMV_SV_H */
