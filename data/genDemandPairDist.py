import pandas as pd
import numpy as np
from scipy.ndimage.filters import uniform_filter,convolve1d
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

pair.drop(['loc_ori','loc_des'],axis=1, inplace=True)

# 0 central, 1 left, 2 right, 3 up, 4 down
def get_direction_index(o_x,o_y,d_x,d_y):
    if ((abs(o_x-d_x) + abs(o_y-d_y)) == 0):
        return 0;
    elif (o_x-d_x == 1):
        return 1;
    elif (o_x-d_x == -1):
        return 2;
    elif (o_y-d_y == -1):
        return 3;
    elif (o_y-d_y == 1):
        return 4;
    else:
        return -1;

pair = pair.as_matrix()
pd_dist = np.zeros((1440,10,10,5))
for i in xrange(pair.shape[0]):
    t_o = pair[i,0]
    t_d = pair[i,1]
    o_x = int(pair[i,2])
    o_y = int(pair[i,3])
    d_x = int(pair[i,4])
    d_y = int(pair[i,5])
    route_size = abs(o_x-d_x) + abs(o_y-d_y)
    if (route_size <= 1):
        pd_dist[(int(t_o - 1)) % 1440,o_x-1,o_y-1,get_direction_index(o_x,o_y,d_x,d_y)] += 1.0
    else:
        # do x axis first; index += n
        if (o_x < d_x):
            route_x = range(o_x,d_x + 1) + [d_x] * abs(o_y-d_y)
        elif (o_x > d_x):
            route_x = range(o_x,d_x - 1, -1) + [d_x] * abs(o_y-d_y)
        else:
            route_x = [d_x] * (abs(o_y-d_y) + 1)
        # then do y axis; index += 1
        if (o_y < d_y):
            route_y = [o_y] * abs(o_x - d_x) + range(o_y, d_y + 1)
        elif (o_y > d_y):
            route_y = [o_y] * abs(o_x - d_x) + range(o_y, d_y - 1, -1)
        else:
            route_y = [o_y] * (abs(o_x - d_x) + 1)

        u_t = (t_d - t_o) / float(route_size)
        for i in xrange(route_size):
            pd_dist[(int(t_o + u_t * i - 1)) % 1440,route_x[i]-1,route_y[i]-1,
                    get_direction_index(route_x[i]-1,route_y[i]-1,route_x[i+1]-1,route_y[i+1]-1)] += 1.0

for i in xrange(5):
    pd_dist[:,:,:,i] = uniform_filter(pd_dist[:,:,:,i], size = [10,3,3], mode = 'constant')
pd_dist[:,:,:,1:] = convolve1d(pd_dist[:,:,:,1:], weights = [0.2,0.6,0.2], axis = 3, mode = 'wrap')
pd_dist = np.maximum(pd_dist,0.0)

pd_dist = pd_dist / 30.0

# post processing to reduce manipulation difficulty
pd_dist_graph = {}
for i in xrange(10):
    for j in range(10):
        n = 10 * i + j;
        pd_dist_graph[(n,n)] = pd_dist[:,i,j,0]
        if (i > 0):
            # left
            m = 10 * (i-1) + j;
            pd_dist_graph[(n,m)] = pd_dist[:,i,j,1]
        if (i < 9):
            # right
            m = 10 * (i+1) + j;
            pd_dist_graph[(n,m)] = pd_dist[:,i,j,2]
        if (j < 9):
            # up
            m = 10 * i + (j+1);
            pd_dist_graph[(n,m)] = pd_dist[:,i,j,3]
        if (j > 0):
            # down
            m = 10 * i + (j-1);
            pd_dist_graph[(n,m)] = pd_dist[:,i,j,4]

pickle.dump(pd_dist_graph, open( "pd_dist_graph.p", "wb" ))
