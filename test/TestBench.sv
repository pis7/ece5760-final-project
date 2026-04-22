`ifndef TEST_BENCH_SV
`define TEST_BENCH_SV

class TestBench;
  bit all_asserts_passed;

  function new;
    this.all_asserts_passed = 1;
  endfunction

  task check_32b_eq (
    input logic [31:0] received,
    input logic [31:0] expected
  );
    if (expected !== received) begin
      $display("ERROR: expected %d but recieved %d", expected, received);
      this.all_asserts_passed = 0;
    end
  endtask: check_32b_eq

  function bit all_checks_passed();
    all_checks_passed = this.all_asserts_passed;
  endfunction
endclass

`endif /* TEST_BENCH_SV */
