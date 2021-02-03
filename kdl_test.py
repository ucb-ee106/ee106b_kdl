import ee106b_baxter_kdl
import numpy as np 
import time

q = np.zeros(7)

# t0 = time.time()
# J = ee106b_baxter_kdl.baxter_jacobian(q)
# t = time.time() - t0
# print("Jacobian time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_gravity_vector(q)
t = time.time() - t0
print("Gravity time:", t)