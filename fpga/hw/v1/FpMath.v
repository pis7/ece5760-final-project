// Fixed-point signed multiply
// Uses MandelbrotSignedMult pattern: {sign, high bits}

module FpMul #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic signed [p_total_bits-1:0] a,
  input  logic signed [p_total_bits-1:0] b,
  output logic signed [p_total_bits-1:0] result
);

  logic signed [2*p_total_bits-1:0] full;

  always_comb begin
    full   = a * b;
    result = $signed({full[2*p_total_bits-1],
              full[2*(p_total_bits-1)-p_int_bits : p_frac_bits]});
  end

endmodule

// Fixed-point signed divide with wide inputs
// Accepts p_wide_bits-wide numerator/denominator (for rr, dq from dot products).
// Produces p_total_bits-wide result (27-bit).
// On FPGA this is LUT-based (not DSP), so input width is flexible.

module FpDiv #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_wide_bits  = 48
) (
  input  logic signed [p_wide_bits-1:0]  a,
  input  logic signed [p_wide_bits-1:0]  b,
  output logic signed [p_total_bits-1:0] result
);

  logic signed [2*p_wide_bits-1:0] num;

  always_comb begin
    num    = (2*p_wide_bits)'($signed(a)) <<< p_frac_bits;
    result = p_total_bits'(num / (2*p_wide_bits)'($signed(b)));
  end

endmodule
