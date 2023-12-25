import numpy as np
import numpy as np
import time
import numpy as np
from scipy.optimize import linprog
from cvxopt import matrix, solvers, sqrt
import cvxopt
import cvxopt.solvers
from scipy.integrate import odeint
from cvxopt import matrix,sparse
import math
from scipy.optimize import minimize
from scipy.optimize import linprog
from scipy.optimize import fmin_slsqp
import numpy as np
#from scipy.optimize import odeint
import numpy as np
from cvxopt import matrix

# Given arrays
H = np.array([[1.,0. ],[0.,0.1]])

f = np.array([[3.00e+00],
              [0.00e+00]])

A = np.array([[  1.,0.],[-1.,0.],[-20.31348321,-1.],[  0.03533069,0.],[  1.,0.],[ -1.,0.]])

b = np.array([4.,6.,-1031.59400069 ,-2.00019157 ,5.15674161, 4.84325839])

# Create matrices for cvxopt.solvers.qp
P = matrix(H)
q = matrix(f)
G = matrix(A)  # cvxopt uses <= inequality, so multiply by -1
h = matrix(b)

# No equality constraints, so A and b matrices are empty
options = {'show_progress': False}
# Solve the quadratic programming problem
sol = cvxopt.solvers.qp(P, q, G, h, options=options)
print(sol['x'])  # This will contain the solution to the QP problem


