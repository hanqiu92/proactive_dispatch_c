import numpy as np
import pickle

# import files
od_dist = pickle.load( open( "od_dist_finer.p", "rb" ) )
t_dist = pickle.load( open( "t_dist_finer.p", "rb" ) )

od_dist_c = np.zeros((48,10000))
t_dist_c = np.zeros((48,100))

for i in xrange(48):
    for j in xrange(10000):
        od_dist_c[i,j] = od_dist[i,j%100,j/100]
    for j in xrange(100):
        t_dist_c[i,j] = t_dist[i,j%10,j/10]

np.savetxt("od_d_c_finer.csv", od_dist_c, fmt="%.6f", delimiter=",")
np.savetxt("t_d_c_finer.csv", t_dist_c, fmt="%.6f", delimiter=",")
