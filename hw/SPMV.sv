`ifndef SPMV_SV_H
`define SPMV_SV_H
`include "macros.sv"
`include "MemController.sv"

/* Harcode index widths because system verilog is dumb and doesn't support
 * paramaterized structs */
parameter IDX_WIDTH  = 32;
/* Harcode data widths because system verilog is dumb and doesn't support
 * paramaterized structs */
parameter DATA_WIDTH = 32;

typedef enum {
  RD, WR
} MemCmdTy;

typedef struct packed {
  MemCmdTy           ty;
  logic [IDX_WIDTH-1:0]  idx;
  logic [DATA_WIDTH-1:0] data;
} MemReq;

typedef struct packed {
  MemCmdTy           ty;
  logic [DATA_WIDTH-1:0] data;
} MemResp;

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
  input  logic        rst,
  input  logic        clk,

  input  logic        go,
  output logic        done,

  input  logic [31:0] num_rows,
  input  logic [31:0] num_non_zeros,

  output MemReq    o_q_mem_req,
  output logic        o_q_mem_req_val,
  input  logic        i_q_mem_req_rdy,

  input  MemResp   i_q_mem_resp,
  input  logic        i_q_mem_resp_val,
  output logic        o_q_mem_resp_rdy,

  output MemReq    o_x_mem_req,
  output logic        o_x_mem_req_val,
  input  logic        i_x_mem_req_rdy,

  input  MemResp   i_x_mem_resp,
  input  logic        i_x_mem_resp_val,
  output logic        o_x_mem_resp_rdy,

  output MemReq    o_c_mem_req,
  output logic        o_c_mem_req_val,
  input  logic        i_c_mem_req_rdy,

  input  MemResp   i_c_mem_resp,
  input  logic        i_c_mem_resp_val,
  output logic        o_c_mem_resp_rdy
);
  typedef enum {
    IDLE,
    READ
  } State;


  State state;
  State next_state;

  always_comb begin
    case (state)
      IDLE:    next_state = go ? READ : IDLE;
      READ:    next_state = READ;
      default: next_state = IDLE;
    endcase
  end

  always_ff @(clk) begin
    if (rst)   state <= IDLE;
    else       state <= next_state;
  end

  logic [31:0] num_rows_reg;
  logic [31:0] num_non_zeros_reg;

  always_ff @(clk) begin
    if (rst) begin
      num_rows_reg      <= 0;
      num_non_zeros_reg <= 0;
    end else if (go && state == IDLE) begin
      num_rows_reg      <= num_rows;
      num_non_zeros_reg <= num_non_zeros;
    end else begin
      num_rows_reg      <= num_rows_reg;
      num_non_zeros_reg <= num_non_zeros_reg;
    end
  end

  // logic [DATA_WIDTH-1:0] cur_x_val;
  // MemController #(
  //   .DATA_WIDTH(DATA_WIDTH),
  //   .IDX_WIDTH (IDX_WIDTH)
  //  ) memController (
  //   .clk              (clk),
  //   .rst              (rst),
  //   .req              (req),
  //   .req_val          (req_val),
  //   .resp             (resp),
  //   .resp_val         (resp_val),
  //   .mem_req_cnct     (mem_req_cnct),
  //   .mem_req_cnct_val (mem_req_cnct_val),
  //   .mem_req_cnct_rdy (mem_req_cnct_rdy),
  //   .mem_resp_cnct    (mem_resp_cnct),
  //   .mem_resp_cnct_val(mem_resp_cnct_val),
  //   .mem_resp_cnct_rdy(mem_resp_cnct_rdy)
  // );
  

  // logic [DATA_WIDTH-1:0] cur_x_val;
  // logic [IDX_WIDTH-1:0]  x_val_count;
  // logic [IDX_WIDTH-1:0]  next_x_val_count;

  // // TODO: Actually assign next_x_val_count
  // assign next_x_val_count = x_val_count;

  // always_ff @(clk) begin
  //   if (rst)                 cur_x_val <= 0;
  //   else if (i_x_mem_resp_val) cur_x_val <= i_x_mem_resp.data;
  //   else                       cur_x_val <= cur_x_val;
  // end

  // always_ff @(clk) begin
  //   if (rst) x_val_count <= 0;
  //   else       x_val_count <= next_x_val_count;
  // end

  // logic                  reqed_x_val;
  // logic                  read_x_val;
  // logic                  next_reqed_x_val;
  // logic                  next_read_x_val;

  // always_comb begin
  //   o_x_mem_req_val  = !reqed_x_val && !read_x_val && state == READ;
  //   next_reqed_x_val = !reqed_x_val && !read_x_val && i_x_mem_req_rdy && state == READ;

  //   o_x_mem_resp_rdy = reqed_x_val && !read_x_val && state == READ;
  //   next_read_x_val  = reqed_x_val && !read_x_val && i_x_mem_resp_val && state == READ;

  //   o_x_mem_req      = MemReq'{RD, x_val_count, 0};
  // end

  // always_ff @(clk) begin
  //   if (rst) begin
  //     reqed_x_val <= 0;
  //     read_x_val  <= 0;
  //   end else begin
  //     reqed_x_val <= next_reqed_x_val;
  //     read_x_val  <= next_read_x_val;
  //   end
  // end

endmodule: SPMV

`endif /* SPMV_SV_H */
