import ee106b_baxter_kdl
import numpy as np 
import time

q = np.zeros(7)

# t0 = time.time()
# J = ee106b_baxter_kdl.baxter_jacobian(q)
# t = time.time() - t0
# print("Jacobian time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_gravity_vector("left", q)
t = time.time() - t0
print("Left Gravity time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_gravity_vector("right", q)
t = time.time() - t0
print("Right Gravity time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_gravity_vector("blah", q)
t = time.time() - t0
print("Incorrect Arm Name Gravity time:", t)


t0 = time.time()
G = ee106b_baxter_kdl.baxter_jacobian("left", q)
t = time.time() - t0
print("Left Jacobian time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_jacobian("right", q)
t = time.time() - t0
print("Right Jacobian time:", t)

t0 = time.time()
G = ee106b_baxter_kdl.baxter_jacobian("blah", q)
t = time.time() - t0
print("Incorrect Arm Name Jacobian time:", t)
