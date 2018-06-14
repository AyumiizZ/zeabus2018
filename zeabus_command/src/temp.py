import sklearn
import numpy as np
import pickle

def linear(area):
        return -38.69222*(area) + 3.76897

pick_in = open('linearregression.pickle', 'rb')
clf = pickle.load(pick_in)

print clf.predict(np.array([[0, 0]]))
print clf.predict(np.array([[0, 1]]))


print linear(0.02)
