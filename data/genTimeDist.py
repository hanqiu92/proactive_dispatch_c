import pandas as pd
import numpy as np
from scipy.ndimage.filters import uniform_filter
import pickle

pair = pd.read_table('pair_raw.csv',sep=',')
loc_raw = pd.read_table('loc_xy_raw.tsv',sep='\t',names=['index','x','y'])
loc_ind = pd.read_table('loc_scale_10.csv',sep=',')

pair = pair.merge(loc_ind,left_on='loc_ori',right_on='index',how='left')
pair = pair.rename(columns = {'x':'o_x','y':'o_y'})
pair.drop(['index'],axis=1, inplace=True)

pair = pair.merge(loc_ind,left_on='loc_des',right_on='index',how='left')
pair = pair.rename(columns = {'x':'d_x','y':'d_y'})
pair.drop(['index'],axis=1, inplace=True)

pair = pair.merge(loc_raw,left_on='loc_ori',right_on='index',how='left')
pair = pair.rename(columns = {'x':'o_x_raw','y':'o_y_raw'})
pair.drop(['index'],axis=1, inplace=True)

pair = pair.merge(loc_raw,left_on='loc_des',right_on='index',how='left')
pair = pair.rename(columns = {'x':'d_x_raw','y':'d_y_raw'})
pair.drop(['index'],axis=1, inplace=True)

pair.drop(['loc_ori','loc_des'],axis=1, inplace=True)

dt = pair['time_des'] - pair['time_ori']
tt = pair['time_des'] + pair['time_ori']
pair['t'] = np.around((tt * (dt > 0) + (1440 + tt) * (dt < 0)) / 2.0).astype(int) % 1440
pair['time'] = (dt * (dt > 0) + (1440 + dt) * (dt < 0)) / np.sqrt((pair['o_x_raw'] - pair['d_x_raw']) ** 2 + (pair['o_y_raw'] - pair['d_y_raw']) ** 2)

pair.drop(['time_des','time_ori','o_x_raw','o_y_raw','d_x_raw','d_y_raw'],axis=1, inplace=True)

pair = pair.as_matrix()
t_dist = np.zeros((1440,10,10))
n_dist = np.zeros((1440,10,10))
for i in xrange(pair.shape[0]):
    t = int(pair[i,4])
    dt = pair[i,5]
    o_x = int(pair[i,0])
    o_y = int(pair[i,1])
    d_x = int(pair[i,2])
    d_y = int(pair[i,3])
    t_dist[t-1,o_x-1,o_y-1] += dt
    t_dist[t-1,d_x-1,d_y-1] += dt
    n_dist[t-1,o_x-1,o_y-1] += 1.0
    n_dist[t-1,d_x-1,d_y-1] += 1.0

t_dist = t_dist / np.maximum(0.5,n_dist)

def fill(seq):
    if sum(seq) < 1.0:
        seq = np.ones((1440))
        return seq
    else:
        ind = np.argwhere(seq > 0)[:,0]
        curr_ind = 0
        for i in xrange(ind[0],ind[-1]):
            if i == ind[curr_ind]:
                curr_ind += 1
            else:
                a = ind[curr_ind-1]
                b = ind[curr_ind]
                seq[i] = (seq[a] * (b-i) + seq[b] * (i-a)) / float(b-a)

        if ((ind[-1] - ind[0]) < 1439):
            a = ind[-1]
            b = ind[0] + 1440
            x = seq[ind[-1]]
            y = seq[ind[0]]
            for i in xrange(ind[-1],1440+ind[0]):
                seq[i%1440] = (x * (b-i) + y * (i-a)) / float(b-a)

    return seq

t_dist = np.apply_along_axis(fill,0,t_dist)
t_dist = uniform_filter(t_dist, size = [10,3,3], mode = 'constant')
factor = 1.0

def convert_time_by_factor_model1(time,factor):
    d1 = 1.0
    d2 = 6.0
    density = time * d1 * (time <= 1.0) + (d2 - (d2 - d1) / time) * (time > 1.0)
    density = density * factor
    new_time = 1.0 * (density <= d1) + 30.0 * (density > d2 * 0.9) + np.minimum((d2 - d1) / (d2 - density), 30.0) * (density <= d2 * 0.9) * (density > d1)
    return new_time

def convert_time_by_factor_model2(time,factor):
    d1 = 1.0
    d2 = 6.0
    density = time * d1 * (time <= 1.0) + (time * d2) / (time + (d2 - d1) / d1) * (time > 1.0)
    density = density * factor
    new_time = 1.0 * (density <= d1) + 30.0 * (density > d2 * 0.9) + np.minimum((d2 - d1) / d1 * density / (d2 - density), 30.0) * (density <= d2 * 0.9) * (density > d1)
    return new_time

t_dist = convert_time_by_factor_model1(t_dist,factor)
t_dist = np.maximum(t_dist,1.0)

pickle.dump(t_dist, open( "t_dist.p", "wb" ))
