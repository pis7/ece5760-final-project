`ifndef MEM_TYPES_SV
`define MEM_TYPES_SV

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
  MemCmdTy               ty;
  logic [IDX_WIDTH-1:0]  idx;
  logic [DATA_WIDTH-1:0] data;
} MemReq;

typedef struct packed {
  MemCmdTy               ty;
  logic [DATA_WIDTH-1:0] data;
} MemResp;

`endif
