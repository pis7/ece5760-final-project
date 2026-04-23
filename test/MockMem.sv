`ifndef MOCK_MEM_SV
`define MOCK_MEM_SV

`include "../hw/MemTypes.sv"

module MockMem #(
  parameter WIDTH=32, parameter SIZE
)(
  input  logic      clk,
  input  logic      rst,

  input  MemReq     mem_req,
  input  logic      mem_req_val,
  output logic      mem_req_rdy,

  output MemResp    mem_resp,
  output logic      mem_resp_val,
  input  logic      mem_resp_rdy
);
  typedef enum {
    IDLE, DONE
  } State;


  logic [WIDTH-1:0] data[SIZE];
  State state;
  State next_state;

  always_comb begin
    case (state)
      IDLE:    next_state = mem_req_val  ? DONE : IDLE;
      DONE:    next_state = mem_resp_rdy ? IDLE : DONE;
      default: next_state = IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) state <= IDLE;
    else     state <= next_state;
  end

  assign mem_req_rdy = state == IDLE;

  always_ff @(posedge clk) begin
    for (integer i = 0; i < SIZE; i++) begin
      if      (rst)                                                                  data[i] <= 0;
      else if (state == IDLE && mem_req_val && mem_req.idx == i && mem_req.ty == WR) data[i] <= mem_req.data;
      else                                                                           data[i] <= data[i];
    end
  end

  assign mem_resp_val = state == DONE;

  always_ff @(posedge clk) begin
    if      (rst)                                              mem_resp <= 0;
    else if (state == IDLE && mem_req_val && mem_req.ty == WR) mem_resp <= MemResp'{WR, 0};
    else if (state == IDLE && mem_req_val && mem_req.ty == RD) mem_resp <= MemResp'{RD, data[mem_req.idx]};
    else if (state == DONE && !mem_resp_rdy)                   mem_resp <= mem_resp;
    else                                                       mem_resp <= 0;
  end

endmodule: MockMem

`endif /* MOCK_MEM_SV */
