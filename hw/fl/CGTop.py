# Toplevel FL model

from typing import List
from fpga.fl.LinAlg import *

def CGTop(
    Qx : List[List], 
    x : List[float], 
    cx : List[float],
    n : int,
    max_iter=1000, 
    eps_sq=(1e-5 ** 2)
  ) -> List[float]:

  # Register load step ---------------------------------------------------------

  Qx_reg = Qx
  x_reg  = x
  cx_reg = cx
  d_reg  = [0.0] * n
  r_reg  = [0.0] * n
  rr_reg = 0.0

  # Initialization step --------------------------------------------------------

  Qx_x = SPMV(
    A = Qx_reg,
    b = x_reg,
    n = n
  )

  r_new = VecNegSub(
    a = cx_reg,
    b = Qx_x,
    n = n
  )

  rr_new = VecDot(
    a = r_new,
    b = r_new,
    n = n
  )

  # Register update

  d_reg  = r_new
  r_reg  = r_new
  rr_reg = rr_new

  # Main solve loop ------------------------------------------------------------

  for it in range(max_iter):

    q = SPMV(
      A = Qx_reg,
      b = d_reg,
      n = n
    )

    dq = VecDot(
      a = d_reg,
      b = q,
      n = n
    )
    
    alpha = rr_reg / dq

    x_new = AXPY(
      a = x_reg,
      b = d_reg,
      coef = alpha,
      n = n
    )

    r_new = AXPY(
      a = r_reg,
      b = q,
      coef = alpha,
      n = n,
      mode = "sub"
    )

    rr_new = VecDot(
      a = r_new,
      b = r_new,
      n = n
    )

    beta = rr_new / rr_reg

    d_new = AXPY(
      a = r_new,
      b = d_reg,
      coef = beta,
      n = n
    )

    # Register update

    d_reg  = d_new
    r_reg  = r_new
    rr_reg = rr_new
    if dq != 0.0:
      x_reg = x_new

    # Done condition

    if dq == 0.0 or rr_new < eps_sq:
      break

  return x_reg