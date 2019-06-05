import numpy as np
from scipy.sparse import diags

def row_filter(data,fil_length,fil_type):
    insert_len = int((fil_length-1)/2)
    head = data[0].reshape(1,-1)
    tail = data[-1].reshape(1,-1)
    for i in range(insert_len):
        data = np.insert(np.append(data,tail,axis=0),0,head,axis=0)
    data = np.transpose(data)

    fil_row = data.shape[1]
    fil_col = data.shape[1]-fil_length+1
    if fil_type == 'average':
        element = np.ones((fil_length))
        diag = -np.arange(fil_length)
    if fil_type == 'differenz':
        element = np.zeros((fil_length))
        element[0]=-1
        element[-1]=1
        diag = -np.arange(fil_length)
    fil = diags(element,diag,shape=(fil_row,fil_col)).toarray()
    re = np.dot(data,fil)
    return re

a = np.array([[1,1],[2,2],[3,3],[4,4],[5,5],[6,6]])
# fil_length = 3
# fil_type ='average'
# re = row_filter(a,fil_length,fil_type)
# print(a,'\n',re)

from scipy import signal
fil = np.array([[1],[0],[-1]])
re = signal.convolve2d(a,fil,mode = 'same')
print(a,re)


