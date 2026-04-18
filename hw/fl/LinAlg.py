# Common Lin Alg Modules

from typing import List

CSR_VALS = 0
CSR_COL_IDXS = 1
CSR_ROW_PTRS = 2

def SPMV(
    A : List[List], 
    b : List[float],
    n : int
  ) -> List[float]:
  
  ret = [0.0] * n

  for i in range(n):
    s = 0
    for j in range(A[CSR_ROW_PTRS][i], A[CSR_ROW_PTRS][i+1]):
      s += A[CSR_VALS][j] * b[A[CSR_COL_IDXS][j]]
    ret[i] = s
  
  return ret

def VecNegSub(
    a : List[float],
    b : List[float],
    n : int
  ) -> List[float]:

  ret = [0.0] * n

  for i in range(n):
    ret[i] = -a[i] - b[i]
  
  return ret

def VecDot(
    a : List[float],
    b : List[float],
    n : int
  ) -> float:

  ret = 0.0

  for i in range(n):
    ret += a[i] * b[i]
  
  return ret

def AXPY(
    a : List[float],
    b : List[float],
    coef : float,
    n : int,
    mode : str = "add"
  ) -> List[float]:
  
  ret = [0.0] * n

  for i in range(n):
    if mode == "add":
      ret[i] = a[i] + coef * b[i]
    elif mode == "sub":
      ret[i] = a[i] - coef * b[i]
    else:
      ret[i] = 0.0

  return ret
