import numpy as np
from dlroms import* 

from playground import mesh, Vh
from ufl_legacy import nabla_div
from scipy.sparse import csr_matrix
from fenics import nabla_grad, inner, dot, dx, ds, Identity

# Boundary conditions
tol = mesh.hmax()/2.0
def clamped_boundary(x):
    return x[1] < tol
bc = fe.DirichletBC(clamped_boundary, lambda x: [0.0, 0.0])

# Auxiliary definitions
# def epsilon(u):
#     return 0.5*(nabla_grad(u) + nabla_grad(u).T)

# def sigma(u):
#     return lambda_*nabla_div(u)*Identity(2) + 2*mu*epsilon(u)

# Variational problem
f_rho = fe.interpolate(lambda x: [0, -9.81], Vh)
f_mass = fe.interpolate(lambda x: [0.0, -9.81*(x[1] > 1-tol)/4.2], Vh)

    
a_lambda = lambda u, v: 0.5 * inner(nabla_div(u)*Identity(2), nabla_grad(v) + nabla_grad(v).T)*dx
a_mu = lambda u, v: 0.5 * inner(nabla_grad(u) + nabla_grad(u).T, nabla_grad(v) + nabla_grad(v).T)*dx
F_rho = lambda v: dot(f_rho, v)*dx 
F_mass = lambda v: dot(f_mass, v)*ds

# Assembling and adjusting
Ah_lambda = fe.assemble(a_lambda, Vh)
Ah_mu = fe.assemble(a_mu, Vh)
Fh_rho = fe.assemble(F_rho, Vh)
Fh_mass = fe.assemble(F_mass, Vh)

print(Ah_lambda)
print(Ah_mu)
print(Fh_rho)
print(Fh_mass)

Alambda = fe.applyBCs(Ah_lambda, Vh, bc)
Amu = fe.applyBCs(Ah_mu, Vh, bc)
Frho = fe.applyBCs(Fh_rho, Vh, bc)
Fmass = fe.applyBCs(Fh_mass, Vh, bc)


