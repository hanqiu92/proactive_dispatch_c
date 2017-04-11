import pandas as pd
import numpy as np
from scipy.ndimage.filters import uniform_filter
import pickle

OD = pd.read_table('OD_raw.csv',sep=',')
loc = pd.read_table('loc_scale_10.csv',sep=',')

OD_ori = OD.merge(loc,left_on='loc_ori',right_on='index',how='left')
OD_ori = OD_ori.rename(columns = {'x':'o_x','y':'o_y'})
OD_ori.drop(['loc_ori','index'],axis=1, inplace=True)

OD_ori_des = OD_ori.merge(loc,left_on='loc_des',right_on='index',how='left')
OD_ori_des = OD_ori_des.rename(columns = {'x':'d_x','y':'d_y'})
OD_ori_des.drop(['loc_des','index'],axis=1, inplace=True)

od = OD_ori_des.as_matrix()
o_dist = np.zeros((1440,10,10))
d_dist = np.zeros((1440,10,10))
for i in xrange(od.shape[0]):
    o_t = od[i,0]
    d_t = od[i,1]
    o_x = od[i,2]
    o_y = od[i,3]
    d_x = od[i,4]
    d_y = od[i,5]
    o_dist[o_t-1,o_x-1,o_y-1] += 1.0
    d_dist[d_t-1,d_x-1,d_y-1] += 1.0

o_dist = o_dist / 30.0
d_dist = d_dist / 30.0

for i in xrange(1440):
    o_dist[i,:,:] = uniform_filter(o_dist[i,:,:], size = 3, mode = 'constant')
    o_dist[i,:,:] = np.maximum(o_dist[i,:,:],0)
    d_dist[i,:,:] = uniform_filter(d_dist[i,:,:], size = 3, mode = 'constant')
    d_dist[i,:,:] = np.maximum(d_dist[i,:,:],0)

pickle.dump(o_dist, open( "o_dist_smooth.p", "wb" ))
pickle.dump(d_dist, open( "d_dist_smooth.p", "wb" ))
