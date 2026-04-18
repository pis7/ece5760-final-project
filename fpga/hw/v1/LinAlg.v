// Sparse matrix-vector multiply (CSR format)
// result = A * vec
// Accumulates full-precision products in a wide accumulator,
// truncates only the final per-row sum to p_total_bits.

module SPMV #(
  parameter p_max_n      = 50,
  parameter p_max_nnz    = p_max_n * p_max_n,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_acc_bits   = 48
) (
  input  logic [31:0] n,

  // CSR matrix A
  input  logic signed [p_total_bits-1:0] vals    [p_max_nnz],
  input  logic signed [p_total_bits-1:0] col_idx [p_max_nnz],
  input  logic signed [p_total_bits-1:0] row_ptr [p_max_n+1],

  // Input vector
  input  logic signed [p_total_bits-1:0] vec [p_max_n],

  // Output vector
  output logic signed [p_total_bits-1:0] result [p_max_n]
);

  // Full-precision multiply: returns ~40-bit shifted product (no truncation).
  // On FPGA: DSP gives 54-bit product, shift right by frac_bits.
  function automatic signed [p_acc_bits-1:0] fp_mul_wide(
    input signed [p_total_bits-1:0] a,
    input signed [p_total_bits-1:0] b
  );
    logic signed [2*p_total_bits-1:0] full;
    full = a * b;
    fp_mul_wide = p_acc_bits'(full >>> p_frac_bits);
  endfunction

  logic signed [p_acc_bits-1:0] sum;

  always_comb begin
    for( int i = 0; i < p_max_n; i++ ) begin
      sum = '0;
      for( int j = 0; j < p_max_nnz; j++ ) begin
        if( i < int'(n) ) begin
          if( j >= row_ptr[i] && j < row_ptr[i + 1] ) begin
            sum = sum + fp_mul_wide(vals[j], vec[col_idx[j]]);
          end
        end
      end
      result[i] = p_total_bits'(sum);
    end
  end

endmodule

// Vector negate-subtract: result[i] = -(a[i] + b[i])

module VecNegSub #(
  parameter p_max_n      = 50,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic [31:0] n,

  input  logic signed [p_total_bits-1:0] a [p_max_n],
  input  logic signed [p_total_bits-1:0] b [p_max_n],
  output logic signed [p_total_bits-1:0] result [p_max_n]
);

  always_comb begin
    for( int i = 0; i < p_max_n; i++ ) begin
      if( i < int'(n) )
        result[i] = -(a[i] + b[i]);
      else
        result[i] = '0;
    end
  end

endmodule

// Vector dot product: result = sum(a[i] * b[i])
// Uses wide accumulator with full-precision products.
// Output is wide (p_acc_bits) — used for rr and dq.

module VecDot #(
  parameter p_max_n      = 50,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_acc_bits   = 48
) (
  input  logic [31:0] n,

  input  logic signed [p_total_bits-1:0] a [p_max_n],
  input  logic signed [p_total_bits-1:0] b [p_max_n],
  output logic signed [p_acc_bits-1:0]   result
);

  // Full-precision multiply: returns ~40-bit shifted product (no truncation).
  function automatic signed [p_acc_bits-1:0] fp_mul_wide(
    input signed [p_total_bits-1:0] x,
    input signed [p_total_bits-1:0] y
  );
    logic signed [2*p_total_bits-1:0] full;
    full = x * y;
    fp_mul_wide = p_acc_bits'(full >>> p_frac_bits);
  endfunction

  always_comb begin
    result = '0;
    for( int i = 0; i < p_max_n; i++ ) begin
      if( i < int'(n) )
        result = result + fp_mul_wide(a[i], b[i]);
    end
  end

endmodule

// AXPY: result[i] = a[i] +/- coef * b[i]
// mode: 0 = add, 1 = sub
// Uses truncated 27-bit multiply (coef is a scalar like alpha/beta).

module AXPY #(
  parameter p_max_n      = 50,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic [31:0] n,
  input  logic        mode,  // 0 = add, 1 = sub

  input  logic signed [p_total_bits-1:0] a [p_max_n],
  input  logic signed [p_total_bits-1:0] b [p_max_n],
  input  logic signed [p_total_bits-1:0] coef,
  output logic signed [p_total_bits-1:0] result [p_max_n]
);

  // Truncated 27-bit multiply (matches DSP block output width).
  function automatic signed [p_total_bits-1:0] fp_mul(
    input signed [p_total_bits-1:0] x,
    input signed [p_total_bits-1:0] y
  );
    logic signed [2*p_total_bits-1:0] full;
    full = x * y;
    fp_mul = $signed({full[2*p_total_bits-1],
              full[2*(p_total_bits-1)-p_int_bits : p_frac_bits]});
  endfunction

  always_comb begin
    for( int i = 0; i < p_max_n; i++ ) begin
      if( i < int'(n) ) begin
        if( mode == 1'b0 )
          result[i] = a[i] + fp_mul(coef, b[i]);
        else
          result[i] = a[i] - fp_mul(coef, b[i]);
      end else begin
        result[i] = '0;
      end
    end
  end

endmodule
