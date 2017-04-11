import numpy as np
import pickle

# import files
o_dist_s = pickle.load( open( "o_dist_smooth.p", "rb" ) )
d_dist_s = pickle.load( open( "d_dist_smooth.p", "rb" ) )
t_dist = pickle.load( open( "t_dist.p", "rb" ) )

o_dist_c = np.zeros((1440,100))
d_dist_c = np.zeros((1440,100))
t_dist_c = np.zeros((1440,100))

for i in xrange(1440):
    for j in xrange(100):
        o_dist_c[i,j] = o_dist_s[i,j%10,j/10]
        d_dist_c[i,j] = d_dist_s[i,j%10,j/10]
        t_dist_c[i,j] = t_dist[i,j%10,j/10]
        
pickle.dump(o_dist_c, open( "o_dist_c.p", "wb" ))
pickle.dump(d_dist_c, open( "d_dist_c.p", "wb" ))
pickle.dump(t_dist_c, open( "t_dist_c.p", "wb" ))

np.savetxt("o_d_c.csv", o_dist_c, fmt="%.6f", delimiter=",")
np.savetxt("d_d_c.csv", d_dist_c, fmt="%.6f", delimiter=",")
np.savetxt("t_d_c.csv", t_dist_c, fmt="%.6f", delimiter=",")