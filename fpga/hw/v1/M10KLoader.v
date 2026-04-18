module M10KLoader #(
  parameter NUM_WORDS  = 16,
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = $clog2(NUM_WORDS),
  parameter BASE_ADDR  = 0
) (
  input  logic                  clk,
  input  logic                  rst,

  // Control
  input  logic                  go_read,
  input  logic                  go_write,
  output logic                  done,

  // Avalon M10K interface
  output logic [ADDR_WIDTH-1:0] on_chip_ram_address,
  output logic                  on_chip_ram_chipselect,
  output logic                  on_chip_ram_clken,
  output logic                  on_chip_ram_write,
  input  logic [DATA_WIDTH-1:0] on_chip_ram_readdata,
  output logic [DATA_WIDTH-1:0] on_chip_ram_writedata,
  output logic [3:0]            on_chip_ram_byteenable,

  // External register array interface (registers live in parent)
  output logic [ADDR_WIDTH-1:0] reg_addr,
  output logic                  reg_wr_en,
  output logic signed [DATA_WIDTH-1:0] reg_wr_data,
  input  logic signed [DATA_WIDTH-1:0] reg_rd_data
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------

  typedef enum logic [2:0] {
    IDLE,
    READ_SETUP,
    READ_WAIT,
    READ_CAPTURE,
    WRITE_SETUP,
    WRITE_HOLD,
    DONE_STATE
  } state_t;

  state_t state_reg;
  state_t state_next;
  logic [ADDR_WIDTH-1:0] idx;

  always_ff @( posedge clk ) begin
    if ( rst )
      state_reg <= IDLE;
    else
      state_reg <= state_next;
  end

  //----------------------------------------------------------------------
  // State Transitions
  //----------------------------------------------------------------------

  always_comb begin
    state_next = state_reg;
    case( state_reg )
      IDLE: begin
        if( go_read )       state_next = READ_SETUP;
        else if( go_write ) state_next = WRITE_SETUP;
      end
      READ_SETUP:  state_next = READ_WAIT;
      READ_WAIT:   state_next = READ_CAPTURE;
      READ_CAPTURE: begin
        if( idx == ADDR_WIDTH'(NUM_WORDS - 1) )
          state_next = DONE_STATE;
        else
          state_next = READ_SETUP;
      end
      WRITE_SETUP: state_next = WRITE_HOLD;
      WRITE_HOLD: begin
        if( idx == ADDR_WIDTH'(NUM_WORDS - 1) )
          state_next = DONE_STATE;
        else
          state_next = WRITE_SETUP;
      end
      DONE_STATE: begin
        if( !go_read && !go_write )
          state_next = IDLE;
      end
      default: ;
    endcase
  end

  //----------------------------------------------------------------------
  // State Outputs (combinational)
  //----------------------------------------------------------------------

  // Register array access: always point at current index
  assign reg_addr = idx;

  // Constant outputs
  assign on_chip_ram_address    = BASE_ADDR + idx;
  assign on_chip_ram_clken      = 1'b1;
  assign on_chip_ram_writedata  = reg_rd_data;
  assign on_chip_ram_byteenable = {(DATA_WIDTH/8){1'b1}};
  assign reg_wr_data            = on_chip_ram_readdata;

  //                              cs  wr  reg_wr_en
  task cs(
    input cs_chipselect,
    input cs_write,
    input cs_reg_wr_en
  );
    on_chip_ram_chipselect = cs_chipselect;
    on_chip_ram_write      = cs_write;
    reg_wr_en              = cs_reg_wr_en;
  endtask

  always_comb begin
    case( state_reg )
      IDLE:         cs(0, 0, 0);
      READ_SETUP:   cs(1, 0, 0);
      READ_WAIT:    cs(1, 0, 0);
      READ_CAPTURE: cs(1, 0, 1);
      WRITE_SETUP:  cs(1, 1, 0);
      WRITE_HOLD:   cs(1, 1, 0);
      DONE_STATE:   cs(0, 0, 0);
      default:      cs(0, 0, 0);
    endcase
  end

  //----------------------------------------------------------------------
  // State Outputs (sequential)
  //----------------------------------------------------------------------

  always_ff @( posedge clk ) begin
    if( rst ) begin
      idx  <= '0;
      done <= 1'b0;
    end else begin
      case( state_reg )
        IDLE: begin
          idx  <= '0;
          done <= 1'b0;
        end
        READ_CAPTURE: begin
          if( idx != ADDR_WIDTH'(NUM_WORDS - 1) )
            idx <= idx + 1;
        end
        WRITE_HOLD: begin
          if( idx != ADDR_WIDTH'(NUM_WORDS - 1) )
            idx <= idx + 1;
        end
        DONE_STATE: begin
          done <= 1'b1;
        end
        default: ;
      endcase
    end
  end

endmodule
