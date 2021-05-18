import numpy as np
from sklearn.metrics import jaccard_score

evaluation_folder = '/home/katanachan/catkin_ws/src/code/data/semantickitti_04/evaluations/'

gt_all = np.array([])
pred_all = np.array([])
for i in range(11):
    print(i)
    
    result = np.loadtxt(evaluation_folder + str(i).zfill(6) + '.txt', dtype=np.uint32)
    gt = result[:,0]
    gt = gt & 0xFFFF
    pred = result[:,1]
    gt_all = np.concatenate((gt_all, gt))
    pred_all = np.concatenate((pred_all, pred))
    
# Ignore background and sky label
pred_all = pred_all[gt_all != 0]
gt_all = gt_all[gt_all != 0]
    
print(np.unique(np.concatenate((gt_all, pred_all), axis=0)) )
classes = ['Free Space', 'Car', 'Bicycle', 'Motorcycle', 'Truck', 'Other Vehicle', 'Person', 'Bicyclist', 'Motorcyclist', 'Road', 'Parking', 'Sidewalk', 'Other Ground','Building', 'Fence', 'Vegetation', 'Trunk', 'Terrain', 'Pole', 'Traffic Sign']
scores = 100 * jaccard_score(gt_all, pred_all, average=None)

print (list(zip (classes, np.round(scores,2))))
