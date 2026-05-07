// Combinational fixed-point signed multiply.

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

// Combinational fixed-point signed divide with wide inputs. Accepts
// p_wide_bits-wide a/b (for rr, dq from dot products); returns a
// p_total_bits-wide quotient.

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
