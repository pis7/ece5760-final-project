/* Harcode index widths because system verilog is dumb and doesn't support
 * paramaterized structs */
parameter IDX_WIDTH = 32;

typedef enum {
  RD, WR
} mem_cmd_ty_e;

typedef struct packed {
  mem_cmd_ty_e ty;
  logic [31:0] idx;
} mem_cmd_t;

/* SPMV performs a matrix muliplicaton Qv = x.
 * The module assumes Q is layed out using CSR format in the order data, column
 * index, and row index. Vectors are expected to be layed out with the lower
 * indexed vectors at lower memory addresses and the least memory address at
 * address 0.
 *
 * This module directly writes its result into the x memory, the lowest value
 * at address zero.
 *
 * The module begins reading from memory and computing the product the cycle
 * after go is asserted. Once the module finished writing to the x memory, it
 * asserts done for one cycle.
 */
module SPMV #(
  parameter DATA_WIDTH=32
)(
  input  logic                  i_rst,
  input  logic                  i_clk,

  input  logic                  i_go,
  output logic                  o_done,

  input  logic [31:0]           i_num_rows,
  input  logic [31:0]           i_num_non_zeros,

  output mem_cmd_t              o_qmem_req,
  output logic                  o_qmem_req_val,
  input  logic                  i_qmem_req_rdy,

  input  logic [DATA_WIDTH-1:0] i_qmem_resp,
  input  logic                  i_qmem_resp_val,
  output logic                  o_qmem_resp_rdy,

  output mem_cmd_t              o_vmem_req,
  output logic                  o_vmem_req_val,
  input  logic                  i_vmem_req_rdy,

  input  logic [DATA_WIDTH-1:0] i_vmem_resp,
  input  logic                  i_vmem_resp_val,
  output logic                  o_vmem_resp_rdy,

  output mem_cmd_t              o_xmem_req,
  output logic                  o_xmem_req_val,
  input  logic                  i_xmem_req_rdy,

  input  logic [DATA_WIDTH-1:0] i_xmem_resp,
  input  logic                  i_xmem_resp_val,
  output logic                  o_xmem_resp_rdy
);

  

endmodule: SPMV
