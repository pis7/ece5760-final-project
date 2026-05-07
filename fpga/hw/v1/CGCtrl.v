// v1 CG controller: top-level FSM coordinating M10K load/store and the
// combinational CG datapath. p_word_bits matches CGTop's Avalon width.

module CGCtrl #(
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  parameter p_acc_bits         = 48,
  parameter p_word_bits        = (p_total_bits <= 32) ? 32 : 64
) (
  input  logic clk,
  input  logic rst,

  // ARM
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // M10K loader
  input  logic m10k_loader_done,
  output logic m10k_rd_go,
  output logic m10k_wr_go,

  // Datapath
  input  logic [31:0] iter,
  input  logic [31:0] max_iter,
  input  logic signed [p_acc_bits-1:0] rr_new,
  input  logic signed [p_acc_bits-1:0] rr_old,
  input  logic [31:0] eps_sq,
  output logic do_init,
  output logic do_run,
  output logic sel_y
);

  //----------------------------------------------------------------------
  // State
  //----------------------------------------------------------------------

  typedef enum logic [3:0] {
    IDLE,
    RD_M10K,
    CG_INIT_X,
    CG_RUN_X,
    CG_INIT_Y,
    CG_RUN_Y,
    WR_M10K,
    CG_DONE
  } state_t;

  state_t state_reg;
  state_t state_next;

  always_ff @( posedge clk ) begin
    if ( rst )
      state_reg <= IDLE;
    else
      state_reg <= state_next;
  end

  //----------------------------------------------------------------------
  // State Transitions
  //----------------------------------------------------------------------

  // Sign-extend eps_sq to accumulator width for comparison. eps_sq is
  // the 32-bit PIO input; callers pre-clamp the upper bits so direct
  // sign-extend is safe at any p_total_bits in [2, 64].
  logic signed [p_acc_bits-1:0] eps_sq_wide;
  assign eps_sq_wide = p_acc_bits'($signed(eps_sq));

  always_comb begin
    state_next = state_reg;
    case( state_reg )
      IDLE:      if( sw_go )            state_next = RD_M10K;
      RD_M10K:   if( m10k_loader_done ) state_next = CG_INIT_X;
      CG_INIT_X:                        state_next = CG_RUN_X;
      CG_RUN_X:  if( iter >= max_iter || rr_new <= eps_sq_wide
                     || (iter > 1 && rr_new >= rr_old) )
                                        state_next = CG_INIT_Y;
      CG_INIT_Y:                        state_next = CG_RUN_Y;
      CG_RUN_Y:  if( iter >= max_iter || rr_new <= eps_sq_wide
                     || (iter > 1 && rr_new >= rr_old) )
                                        state_next = WR_M10K;
      WR_M10K:   if( m10k_loader_done ) state_next = CG_DONE;
      CG_DONE:   if( sw_done_ack )      state_next = IDLE;
      default: ;
    endcase
  end

  //----------------------------------------------------------------------
  // State Outputs
  //----------------------------------------------------------------------

  //                 sw_done  rd_go  wr_go  do_init  do_run  sel_y
  task cs (
    input cs_sw_done,
    input cs_m10k_rd_go,
    input cs_m10k_wr_go,
    input cs_do_init,
    input cs_do_run,
    input cs_sel_y
  );
    sw_done    = cs_sw_done;
    m10k_rd_go = cs_m10k_rd_go;
    m10k_wr_go = cs_m10k_wr_go;
    do_init    = cs_do_init;
    do_run     = cs_do_run;
    sel_y      = cs_sel_y;
  endtask

  always_comb begin
    case( state_reg )
      //                done rd  wr  init run sel_y
      IDLE:      cs(     0,  0,  0,  0,   0,  0);
      RD_M10K:   cs(     0,  1,  0,  0,   0,  0);
      CG_INIT_X: cs(     0,  0,  0,  1,   0,  0);
      CG_RUN_X:  cs(     0,  0,  0,  0,   1,  0);
      CG_INIT_Y: cs(     0,  0,  0,  1,   0,  1);
      CG_RUN_Y:  cs(     0,  0,  0,  0,   1,  1);
      WR_M10K:   cs(     0,  0,  1,  0,   0,  0);
      CG_DONE:   cs(     1,  0,  0,  0,   0,  0);
      default:   cs(     0,  0,  0,  0,   0,  0);
    endcase
  end

endmodule
