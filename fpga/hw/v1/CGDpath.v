module CGDpath #(
  parameter p_max_n            = 50,
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  parameter p_acc_bits         = 48,
  parameter p_q_val_base_addr  = 0,
  parameter p_q_col_base_addr  = p_max_n * p_max_n,
  parameter p_q_rowp_base_addr = 2 * p_max_n * p_max_n,
  parameter p_cx_x_base_addr   = 2 * p_max_n * p_max_n + p_max_n + 1,
  parameter p_cx_y_base_addr   = 2 * p_max_n * p_max_n + 2 * p_max_n + 1,
  parameter p_x_base_addr      = 2 * p_max_n * p_max_n + 3 * p_max_n + 1,
  parameter p_y_base_addr      = 2 * p_max_n * p_max_n + 4 * p_max_n + 1,
  parameter p_total_words      = 2 * p_max_n * p_max_n + 5 * p_max_n + 1
) (
  input  logic                      clk,
  input  logic                      rst,

  // Control
  input  logic                      do_init,
  input  logic                      do_run,
  input  logic                      sel_y,

  // CG solve parameters
  input  logic [31:0]               n,

  // Register array (read-only from datapath)
  input  logic signed [p_total_bits-1:0] cg_data [p_total_words],

  // Outputs to control (wide)
  output logic [31:0]                        iter,
  output logic signed [p_acc_bits-1:0]       rr_new,
  output logic signed [p_acc_bits-1:0]       rr_old,

  // Outputs to CGTop for writeback
  output logic signed [p_total_bits-1:0]     x_new [p_max_n],
  output logic signed [p_acc_bits-1:0]       dq
);

  localparam p_max_nnz = p_max_n * p_max_n;

  // Datapath registers
  logic signed [p_total_bits-1:0] d_reg [p_max_n];
  logic signed [p_total_bits-1:0] r_reg [p_max_n];
  logic signed [p_acc_bits-1:0]   rr_reg;

  assign rr_old = rr_reg;

  // Extract CSR arrays and vectors from cg_data for module interfaces
  logic signed [p_total_bits-1:0] q_vals    [p_max_nnz];
  logic signed [p_total_bits-1:0] q_col_idx [p_max_nnz];
  logic signed [p_total_bits-1:0] q_row_ptr [p_max_n+1];
  logic signed [p_total_bits-1:0] cx_vec    [p_max_n];
  logic signed [p_total_bits-1:0] x_vec     [p_max_n];

  always_comb begin
    for( int i = 0; i < p_max_nnz; i++ ) begin
      q_vals[i]    = cg_data[p_q_val_base_addr + i];
      q_col_idx[i] = cg_data[p_q_col_base_addr + i];
    end
    for( int i = 0; i <= p_max_n; i++ )
      q_row_ptr[i] = cg_data[p_q_rowp_base_addr + i];
    for( int i = 0; i < p_max_n; i++ ) begin
      cx_vec[i] = sel_y ? cg_data[p_cx_y_base_addr + i] : cg_data[p_cx_x_base_addr + i];
      x_vec[i]  = sel_y ? cg_data[p_y_base_addr + i]    : cg_data[p_x_base_addr + i];
    end
  end

  //----------------------------------------------------------------------
  // Initialization
  //----------------------------------------------------------------------

  // SPMV: Qx_x = Q * x
  logic signed [p_total_bits-1:0] Qx_x [p_max_n];

  SPMV #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) spmv_init (
    .n       (n),
    .vals    (q_vals),
    .col_idx (q_col_idx),
    .row_ptr (q_row_ptr),
    .vec     (x_vec),
    .result  (Qx_x)
  );

  // VecNegSub: r_new_init = -(cx + Qx_x)
  logic signed [p_total_bits-1:0] r_new_init [p_max_n];

  VecNegSub #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits)
  ) vec_neg_sub_init (
    .n      (n),
    .a      (cx_vec),
    .b      (Qx_x),
    .result (r_new_init)
  );

  // VecDot: rr_new_init = r_new_init . r_new_init (wide output)
  logic signed [p_acc_bits-1:0] rr_new_init;

  VecDot #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) dot_init (
    .n      (n),
    .a      (r_new_init),
    .b      (r_new_init),
    .result (rr_new_init)
  );

  //----------------------------------------------------------------------
  // Main Loop
  //----------------------------------------------------------------------

  // SPMV: q = Q * d_reg
  logic signed [p_total_bits-1:0] q [p_max_n];

  SPMV #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) spmv_loop (
    .n       (n),
    .vals    (q_vals),
    .col_idx (q_col_idx),
    .row_ptr (q_row_ptr),
    .vec     (d_reg),
    .result  (q)
  );

  // VecDot: dq = d_reg . q (wide output)
  VecDot #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) dot_dq (
    .n      (n),
    .a      (d_reg),
    .b      (q),
    .result (dq)
  );

  // FpDiv: alpha = rr_reg / dq (wide inputs, 27-bit output)
  logic signed [p_total_bits-1:0] alpha;

  FpDiv #(
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_wide_bits  (p_acc_bits)
  ) div_alpha (
    .a      (rr_reg),
    .b      (dq),
    .result (alpha)
  );

  // AXPY: x_new = x + alpha * d_reg
  AXPY #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits)
  ) axpy_x (
    .n      (n),
    .mode   (1'b0),  // add
    .a      (x_vec),
    .b      (d_reg),
    .coef   (alpha),
    .result (x_new)
  );

  // AXPY: r_new = r_reg - alpha * q
  logic signed [p_total_bits-1:0] r_new [p_max_n];

  AXPY #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits)
  ) axpy_r (
    .n      (n),
    .mode   (1'b1),  // sub
    .a      (r_reg),
    .b      (q),
    .coef   (alpha),
    .result (r_new)
  );

  // VecDot: rr_new = r_new . r_new (wide output)
  VecDot #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) dot_rr (
    .n      (n),
    .a      (r_new),
    .b      (r_new),
    .result (rr_new)
  );

  // FpDiv: beta = rr_new / rr_reg (wide inputs, 27-bit output)
  logic signed [p_total_bits-1:0] beta;

  FpDiv #(
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_wide_bits  (p_acc_bits)
  ) div_beta (
    .a      (rr_new),
    .b      (rr_reg),
    .result (beta)
  );

  // AXPY: d_new = r_new + beta * d_reg
  logic signed [p_total_bits-1:0] d_new [p_max_n];

  AXPY #(
    .p_max_n      (p_max_n),
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits)
  ) axpy_d (
    .n      (n),
    .mode   (1'b0),  // add
    .a      (r_new),
    .b      (d_reg),
    .coef   (beta),
    .result (d_new)
  );

  //----------------------------------------------------------------------
  // Register Update
  //----------------------------------------------------------------------

  always_ff @(posedge clk) begin
    for( int i = 0; i < p_max_n; i++ ) begin
      if( rst ) begin
        d_reg[i] <= '0;
        r_reg[i] <= '0;
        rr_reg   <= '0;
        iter     <= '0;
      end else if( do_init ) begin
        d_reg[i] <= r_new_init[i];
        r_reg[i] <= r_new_init[i];
        rr_reg   <= rr_new_init;
        iter     <= '0;
      end else if( do_run ) begin
        iter <= iter + 1;
        if( dq != 0 ) begin
          d_reg[i] <= d_new[i];
          r_reg[i] <= r_new[i];
          rr_reg   <= rr_new;
        end
      end
    end
  end

endmodule
