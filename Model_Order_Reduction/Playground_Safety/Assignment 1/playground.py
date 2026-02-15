from IPython.display import clear_output as clc
import numpy as np
from dlroms import*

## Construction of the geometry
domain = fe.rectangle((0, 0), (4.2, 1))
for j in range(10):
  domain = domain - fe.rectangle((0.2+0.4*j, 0.2), (0.4+0.4*j, 0.4))
for j in range(5):
  domain = domain - fe.rectangle((0.2+0.8*j, 0.6), (0.8+0.8*j, 0.8))
    
mesh = fe.mesh(domain, structured = True, stepsize = 0.05)
Vh = fe.space(mesh, 'CG', 1, vector_valued = True) # FE space for the displacement vector field
V_scalar = fe.space(mesh, 'CG', 1) # Auxiliary FE space for scalar-valued maps
clc()


## Definition of the Full Order Model (FOM)
def FOMsolver(rho, lambda_, mu, mass):
    
  from ufl_legacy import nabla_div
  from scipy.sparse.linalg import spsolve
  from scipy.sparse import csr_matrix
  from fenics import nabla_grad, inner, dot, dx, Identity, ds

  # Boundary conditions
  tol = mesh.hmax()/2.0
  def clamped_boundary(x):
      return x[1] < tol
  bc = fe.DirichletBC(clamped_boundary, lambda x: [0.0, 0.0])

  # Auxiliary definitions
  def epsilon(u):
      return 0.5*(nabla_grad(u) + nabla_grad(u).T)

  def sigma(u):
      #return lambda_*nabla_div(u)*Identity(2) + 2*mu*epsilon(u)
      return lambda_*nabla_div(u)*Identity(2) + 2*mu*epsilon(u)
  
  # Variational problem
  f = fe.interpolate(lambda x: [0, -rho*9.81], Vh)
  T = fe.interpolate(lambda x: [0.0, -mass*9.81*(x[1] > 1-tol)/4.2], Vh)

  a = lambda u, v: inner(sigma(u), epsilon(v))*dx
  F = lambda v: dot(f, v)*dx + dot(T, v)*ds

  # Assembling and adjusting
  Ah = fe.assemble(a, Vh)
  Fh = fe.assemble(F, Vh)

  Ah = fe.applyBCs(Ah, Vh, bc)
  Fh = fe.applyBCs(Fh, Vh, bc)

  # Solving
  u = spsolve(Ah, Fh)
  clc()

  return u