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
od_dist = np.zeros((48,100,100))
for i in xrange(od.shape[0]):
    o_t = od[i,0]
    d_t = od[i,1]
    o_x = od[i,2]
    o_y = od[i,3]
    d_x = od[i,4]
    d_y = od[i,5]
    for o_col in xrange(3):
        for o_row in xrange(3):
            o_loc = 10*(o_x+o_col-2)+(o_y+o_row-2)
            if (o_loc >= 0) and (o_loc < 100):
                for d_col in xrange(3):
                    for d_row in xrange(3):
                        d_loc = 10*(d_x+d_col-2)+(d_y+d_row-2)
                        if (d_loc >= 0) and (d_loc < 100):
                            od_dist[(o_t-1)/30,o_loc,d_loc] += 1.0 / 9.0

od_dist = od_dist / 30.0

pickle.dump(od_dist, open( "od_dist_finer.p", "wb" ))
