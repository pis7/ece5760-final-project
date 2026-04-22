`ifndef MOCK_MEM_SV
`define MOCK_MEM_SV

`include "../hw/SPMV.sv"

module MockMem #(
  parameter DATA_WIDTH=32, parameter SIZE
)(
  input  logic      clk,
  input  logic      rst,

  input  MemReq  mem_req,
  input  logic      mem_req_val,
  output logic      mem_req_rdy,

  output MemResp mem_resp,
  output logic      mem_resp_val,
  input  logic      mem_resp_rdy
);
  typedef enum {
    IDLE, DONE
  } State;


  logic [DATA_WIDTH-1:0] data[SIZE];
  State state;
  State next_state;

  always_comb begin
    case (state)
      IDLE:    next_state = mem_req_val  ? DONE : IDLE;
      DONE:    next_state = mem_resp_rdy ? IDLE : DONE;
      default: next_state = IDLE;
    endcase
  end

  always_ff @(clk) begin
    if (rst)   state <= IDLE;
    else       state <= next_state;
  end

  task handle_mem_req (
    input  MemReq  req,
    input  logic      req_val,

    output MemResp resp
  );
    if (req_val && req.ty == WR) begin
      data[req.idx] <= req.data;
    end

    case (mem_req.ty)
      RD: resp <= req_val ? MemResp'{RD, data[req.idx]} : 0;
      WR: resp <= req_val ? MemResp'{WR, 0}             : 0;
      default:;
    endcase
  endtask: handle_mem_req

  always_ff @(clk) begin
    case (state)
      IDLE: handle_mem_req (mem_req, mem_req_val, mem_resp);
      DONE:;
      default:;
    endcase
  end

  always_comb begin
    mem_req_rdy  = state == IDLE;
    mem_resp_val = state == DONE;
  end

endmodule: MockMem

`endif /* MOCK_MEM_SV */
