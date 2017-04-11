import pandas as pd
import numpy as np
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
OD = np.zeros((100,100))
for i in xrange(od.shape[0]):
    o_t = od[i,0]
    d_t = od[i,1]
    o_x = od[i,2]
    o_y = od[i,3]
    d_x = od[i,4]
    d_y = od[i,5]
    OD[10*(o_x-1)+(o_y-1),10*(d_x-1)+(d_y-1)] += 1.0

pickle.dump(OD, open( "OD.p", "wb" ))
