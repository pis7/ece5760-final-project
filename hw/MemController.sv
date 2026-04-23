`ifndef MEM_CONTROLLER_SV
`define MEM_CONTROLLER_SV

`include "MemTypes.sv"

/* The memory we interface with is latency insensitive. This replaces that
   interface with something latency sensitive. It allows queing a message, and
   and then it will hold a valid response for a single cycle at some point in
   the future. */
module MemController (
  input  logic   clk,
  input  logic   rst,

  input  MemReq  req,
  input  logic   req_val,

  output MemResp resp,
  output logic   resp_val,

  output MemReq  mem_req_cnct,
  output logic   mem_req_cnct_val,
  input  logic   mem_req_cnct_rdy,

  input  MemResp mem_resp_cnct,
  input  logic   mem_resp_cnct_val,
  output logic   mem_resp_cnct_rdy
);
  typedef enum {
    IDLE, FETCH, WAIT, HOLD
  } State;

  State state;
  State next_state;

  always_comb begin
    case (state)
      IDLE:    next_state = req_val           ? FETCH : IDLE;
      FETCH:   next_state = mem_req_cnct_rdy  ? WAIT  : FETCH;
      WAIT:    next_state = mem_resp_cnct_val ? HOLD  : WAIT;
      HOLD:    next_state = IDLE;
      default: next_state = IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) state <= IDLE;
    else     state <= next_state;
  end

  MemReq in_reg;
  always_ff @(posedge clk) begin
    if (rst) in_reg                <= 0;
    else if (state == IDLE) in_reg <= req;
    else                    in_reg <= in_reg;
  end

  assign mem_req_cnct = req;
  assign mem_req_cnct_val = state == FETCH;

  MemResp out_reg;
  always_ff @(posedge clk) begin
    if (rst)                out_reg <= 0;
    else if (state == WAIT) out_reg <= mem_resp_cnct;
    else                    out_reg <= out_reg;
  end
  assign mem_resp_cnct_rdy = state == WAIT;

  assign resp = out_reg;
  assign resp_val = state == HOLD;

endmodule: MemController

`endif /* MEM_CONTROLLER_SV */
