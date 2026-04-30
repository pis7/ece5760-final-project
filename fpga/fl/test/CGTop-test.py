# Testbench for CGTop FL model
# Compares CGTop against scipy.sparse.linalg.cg reference

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from typing import List, Tuple
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import cg as scipy_cg

from fpga.fl.CGTop import CGTop

# ===========================================================================
# Helpers
# ===========================================================================

def dense_to_csr_lists(
    M : np.ndarray
  ) -> List[List]:
  """Convert a dense numpy matrix to the CSR list-of-lists format
  used by LinAlg.py:  [vals, col_idxs, row_ptrs]."""

  sp  = csr_matrix(M)
  vals     = sp.data.tolist()
  col_idxs = sp.indices.tolist()
  row_ptrs = sp.indptr.tolist()

  return [vals, col_idxs, row_ptrs]

def make_spd(
    n    : int,
    seed : int = 0
  ) -> np.ndarray:
  """Return a random n x n symmetric positive-definite matrix."""

  rng = np.random.default_rng(seed)
  A   = rng.standard_normal((n, n))
  return A.T @ A + n * np.eye(n)        # A'A + nI guarantees SPD

def run_one_test(
    name   : str,
    Q_dense: np.ndarray,
    c_vec  : np.ndarray,
    x0     : np.ndarray,
    tol    : float = 1e-4
  ) -> bool:
  """Run CGTop and scipy_cg on the same system, compare results."""

  n = Q_dense.shape[0]

  # -- Reference (scipy) -----------------------------------------------------

  Q_sp        = csr_matrix(Q_dense)
  b_ref       = (-c_vec).copy()
  x_ref, info = scipy_cg(Q_sp, b_ref, x0=x0.copy(), rtol=1e-5, maxiter=1000)

  # -- DUT (CGTop) -----------------------------------------------------------

  Qx_csr = dense_to_csr_lists(Q_dense)
  x_dut  = CGTop(
    Qx       = Qx_csr,
    x        = x0.tolist(),
    cx       = c_vec.tolist(),
    n        = n,
    max_iter = 1000,
    eps_sq   = (1e-5 ** 2)
  )

  # -- Compare ---------------------------------------------------------------

  x_dut_np = np.array(x_dut)
  err      = np.linalg.norm(x_dut_np - x_ref) / (np.linalg.norm(x_ref) + 1e-30)

  status = "PASS" if err < tol else "FAIL"
  print(f"  [{status}] {name:40s}  rel_err = {err:.6e}")

  return err < tol

# ===========================================================================
# Test cases
# ===========================================================================

def test_identity_2x2() -> bool:
  """Qx = -c with Q = I  =>  x = -c."""

  Q = np.eye(2)
  c = np.array([3.0, -7.0])
  x0 = np.zeros(2)

  return run_one_test("identity_2x2", Q, c, x0)

def test_diagonal_3x3() -> bool:
  """Diagonal system, known closed-form solution."""

  Q = np.diag([2.0, 5.0, 10.0])
  c = np.array([4.0, -10.0, 30.0])
  x0 = np.zeros(3)

  return run_one_test("diagonal_3x3", Q, c, x0)

def test_dense_spd_4x4() -> bool:
  """Small dense SPD matrix."""

  Q = make_spd(4, seed=42)
  c = np.array([1.0, -2.0, 3.0, -4.0])
  x0 = np.zeros(4)

  return run_one_test("dense_spd_4x4", Q, c, x0)

def test_nonzero_initial_guess() -> bool:
  """Start from a non-zero x0."""

  Q = make_spd(5, seed=7)
  c = np.ones(5) * 2.0
  x0 = np.array([1.0, -1.0, 0.5, -0.5, 0.0])

  return run_one_test("nonzero_initial_guess", Q, c, x0)

def test_tridiagonal_8x8() -> bool:
  """Tri-diagonal SPD matrix (like 1-D Laplacian + shift)."""

  n = 8
  Q = np.zeros((n, n))
  for i in range(n):
    Q[i, i] = 4.0
    if i > 0:
      Q[i, i-1] = -1.0
    if i < n - 1:
      Q[i, i+1] = -1.0
  c = np.arange(1, n + 1, dtype=float)
  x0 = np.zeros(n)

  return run_one_test("tridiagonal_8x8", Q, c, x0)

def test_random_sparse_16() -> bool:
  """Larger random SPD system, n = 16."""

  Q = make_spd(16, seed=123)
  rng = np.random.default_rng(456)
  c  = rng.standard_normal(16)
  x0 = np.zeros(16)

  return run_one_test("random_sparse_16", Q, c, x0)

def test_random_sparse_32() -> bool:
  """Larger random SPD system, n = 32."""

  Q = make_spd(32, seed=789)
  rng = np.random.default_rng(101)
  c  = rng.standard_normal(32)
  x0 = rng.standard_normal(32)

  return run_one_test("random_sparse_32", Q, c, x0)

def test_single_element() -> bool:
  """Degenerate 1x1 system."""

  Q = np.array([[5.0]])
  c = np.array([10.0])
  x0 = np.zeros(1)

  return run_one_test("single_element", Q, c, x0)

def test_large_condition_number() -> bool:
  """SPD matrix with a spread of eigenvalues."""

  n = 6
  eigvals = np.array([0.01, 0.1, 1.0, 10.0, 100.0, 1000.0])
  rng = np.random.default_rng(55)
  V, _ = np.linalg.qr(rng.standard_normal((n, n)))
  Q = V @ np.diag(eigvals) @ V.T
  c = np.ones(n)
  x0 = np.zeros(n)

  return run_one_test("large_condition_number", Q, c, x0, tol=1e-2)

def test_negative_rhs() -> bool:
  """All-negative c vector."""

  Q = make_spd(5, seed=99)
  c = -np.arange(1, 6, dtype=float)
  x0 = np.zeros(5)

  return run_one_test("negative_rhs", Q, c, x0)

def test_nearly_solved() -> bool:
  """x0 is very close to solution — should converge in 1-2 iterations."""

  Q = np.diag([3.0, 6.0])
  c = np.array([9.0, -12.0])
  # Exact solution: x = -Q^{-1} c = [-3, 2], perturb slightly
  x0 = np.array([-3.0 + 1e-6, 2.0 - 1e-6])

  return run_one_test("nearly_solved", Q, c, x0)

def test_scaled_identity_10() -> bool:
  """Scaled identity, n = 10."""

  n = 10
  Q = 7.0 * np.eye(n)
  c = np.arange(1, n + 1, dtype=float)
  x0 = np.zeros(n)

  return run_one_test("scaled_identity_10", Q, c, x0)

def test_arrow_matrix_6x6() -> bool:
  """Arrow (bordered-diagonal) SPD matrix."""

  n = 6
  Q = np.diag([10.0] * n)
  for i in range(n):
    Q[0, i] += 1.0
    Q[i, 0] += 1.0
  Q[0, 0] += n  # keep it well-conditioned
  rng = np.random.default_rng(33)
  c  = rng.standard_normal(n)
  x0 = np.zeros(n)

  return run_one_test("arrow_matrix_6x6", Q, c, x0)

def test_near_singular() -> bool:
  """SPD matrix with one very small eigenvalue."""

  n = 4
  eigvals = np.array([1e-4, 1.0, 2.0, 3.0])
  rng = np.random.default_rng(77)
  V, _ = np.linalg.qr(rng.standard_normal((n, n)))
  Q = V @ np.diag(eigvals) @ V.T
  c = np.array([1.0, 2.0, 3.0, 4.0])
  x0 = np.zeros(n)

  return run_one_test("near_singular", Q, c, x0, tol=1e-2)

def test_large_rhs_values() -> bool:
  """Large magnitude c vector."""

  Q = make_spd(5, seed=200)
  c = np.array([1e6, -1e6, 5e5, -5e5, 1e7])
  x0 = np.zeros(5)

  return run_one_test("large_rhs_values", Q, c, x0)

def test_close_initial_guess() -> bool:
  """x0 is close to solution — should converge in very few iterations."""

  Q = make_spd(4, seed=300)
  c = np.array([2.0, -3.0, 1.0, -1.0])
  # Compute exact solution, then perturb
  x_exact = np.linalg.solve(Q, -c)
  x0 = x_exact + 1e-3 * np.array([1, -1, 1, -1])

  return run_one_test("close_initial_guess", Q, c, x0)

def test_random_sparse_64() -> bool:
  """Larger system, n = 64."""

  Q = make_spd(64, seed=500)
  rng = np.random.default_rng(501)
  c  = rng.standard_normal(64)
  x0 = np.zeros(64)

  return run_one_test("random_sparse_64", Q, c, x0)

def test_block_diagonal_8x8() -> bool:
  """Block-diagonal SPD matrix (two 4x4 blocks)."""

  A = make_spd(4, seed=60)
  B = make_spd(4, seed=61)
  Q = np.zeros((8, 8))
  Q[:4, :4] = A
  Q[4:, 4:] = B
  rng = np.random.default_rng(62)
  c  = rng.standard_normal(8)
  x0 = np.zeros(8)

  return run_one_test("block_diagonal_8x8", Q, c, x0)

def test_uniform_rhs() -> bool:
  """All c entries are the same value."""

  Q = make_spd(6, seed=400)
  c = np.full(6, 5.0)
  x0 = np.zeros(6)

  return run_one_test("uniform_rhs", Q, c, x0)

# ===========================================================================
# Main — run all tests, report summary
# ===========================================================================

if __name__ == "__main__":

  # Auto-collect every top-level callable named test_* (in source order, since
  # Python 3.7+ dicts preserve insertion order). Avoids hand-maintaining a
  # registry that drifts out of sync with the test definitions above.
  tests = [v for k, v in globals().items()
           if k.startswith("test_") and callable(v)]

  print("=" * 72)
  print("CGTop FL Testbench")
  print("=" * 72)

  results = [t() for t in tests]

  print("=" * 72)
  passed = sum(results)
  total  = len(results)
  print(f"Result: {passed}/{total} tests passed")

  if passed < total:
    print("*** SOME TESTS FAILED ***")
    sys.exit(1)
  else:
    print("All tests passed.")
    sys.exit(0)
