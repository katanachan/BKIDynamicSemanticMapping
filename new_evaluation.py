import numpy as np
import IPython
from sklearn.metrics import jaccard_score
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-s', type=int)
parser.add_argument('-n', type=int)
parser.add_argument('-i', type=bool, default=False)
parser.add_argument('-t', type=float, default=0.05)
parser.add_argument('-o', type=bool, default=False)
results = parser.parse_args()

seq = str(results.s).zfill(2)
evaluation_folder = '/media/sdd1/shwarya/catkin_ws/src/code/data/semantickitti_' + seq + '/evaluations/'
data_folder = '/media/sdd1/shwarya/catkin_ws/src/code/data/semantickitti_' + seq + '/predictions/'
#cylinder_folder = '/media/sdd1/shwarya/catkin_ws/src/code/data/semantickitti_' + seq + '/
total_scans = results.n
gen = results.i
thresh = results.t
ours = results.o #get evaluation for your results only

gt_all = np.array([])
pred_all = np.array([])
cyl_all = np.array([])
for i in range(total_scans):
    print(i)
    
    result = np.loadtxt(evaluation_folder + str(i).zfill(6) + '.txt', dtype=np.uint32)
    if False:
        velocities = np.loadtxt(evaluation_folder + str(i).zfill(6) + 'flow.txt')
        gt_vels = np.fromfile(data_folder + str(i).zfill(6) + '.bin', dtype=np.float32).reshape(-1,3)
        gt_vels = np.apply_along_axis(np.linalg.norm, 1, gt_vels)
    cylinder = np.fromfile(data_folder + str(i).zfill(6) + '.label', dtype=np.uint32)

    #print (result.shape, velocities.shape, gt_vels.shape)
    #IPython.embed()
    ## make 4 figures
    ## make velocities from gt and KDE
    if gen:
        fig, axs = plt.subplots(4, sharex=True)
        axs[0].hist(velocities[result[:,0] == 1, 1], label='Static car KDE')
        axs[1].hist(velocities[result[:,0] == 20, 1], label='Moving car KDE')
        axs[0].hist(velocities[result[:,0] == 6, 1], label = 'Static person KDE')
        axs[1].hist(velocities[result[:,0] == 22,1], label = 'Moving person KDE')
        axs[2].hist(gt_vels[result[:,0] == 1], label= 'Static car NN')
        axs[3].hist(gt_vels[result[:,0] == 20], label='Moving car NN')


        for j in range(4):
            axs[j].legend()
            axs[j].set_xlabel("Flow norm")
            axs[j].set_ylabel("Number of points")
        fig.savefig(str(i) + '.png')
        plt.close()
    
    gt = result[:,0]
    gt = gt & 0xFFFF
    pred = result[:,1]
    print (gt.shape, cylinder.shape)
    #cars = pred == 1
    #hoomanz = pred == 6
    #othervehicle = pred == 5
    #motorcyclist = pred == 8
    # truck = pred == 4
    #bicyclist = pred == 7
    #pred[np.logical_and(velocities[:,1] > 0.2, cars)] = 20
    # #pred[np.logical_and(velocities[:,1] > thresh, othervehicle)] = 24
    #pred[np.logical_and(velocities[:,1] > 0.05, motorcyclist)] = 23
    #pred[np.logical_and(velocities[:,1] > 0.1, hoomanz)] = 22
    # pred[np.logical_and(velocities[:,1] > thresh, truck)] = 25
    #pred[np.logical_and(velocities[:,1] > 0.01, bicyclist)] = 21
    # static_hoomanz = velocities[result[:,0]==6]
    # dynamic_hoomanz = velocities[result[:,0]==22]
    # static_cars = velocities[result[:,0] == 1]
    # dynamic_cars = velocities[result[:,0] == 20]
    # static_vehicle = velocities[result[:,0] == 5]
    # dynamic_vehicle = velocities[result[:,0] == 24]
    # print ("Static hoomanz", static_hoomanz.shape[0], static_hoomanz.mean(0))
    # print ("Dynamic hoomanz", dynamic_hoomanz.shape[0], dynamic_hoomanz.mean(0) )
    # print ("Static cars", static_cars.shape[0], static_cars.mean(0))
    # print ("Dynamic cars", dynamic_cars.shape[0], dynamic_cars.mean(0))

    # print ("Static vehicle", static_vehicle.shape[0], static_vehicle.mean(0))
    # print ("Dynamic vehicle", dynamic_vehicle.shape[0], dynamic_vehicle.mean(0))
    # if (dynamic_vehicle.shape[0] != 0):
    #     print (pred[result[:,0]==24])
    #pred[np.logical_and(velocities[:,1] > 0.01, hoomanz)] = 22
    gt_all = np.concatenate((gt_all, gt))
    pred_all = np.concatenate((pred_all, pred))
    cyl_all = np.concatenate((cyl_all, cylinder))
# Ignore background and sky label
pred_all = pred_all[gt_all != 0]
cyl_all = cyl_all[gt_all != 0]
gt_all = gt_all[gt_all != 0]

print(np.unique(np.concatenate((gt_all, pred_all), axis=0), return_counts=True) )
classes = ['Free Space', 'Car', 'Bicycle', 'Motorcycle', 'Truck', 'Other Vehicle', 'Person', 'Bicyclist', 'Motorcyclist', 'Road', 'Parking', 'Sidewalk', 'Other Ground','Building', 'Fence', 'Vegetation', 'Trunk', 'Terrain', 'Pole', 'Traffic Sign']
mine = 100 * jaccard_score(gt_all, pred_all, average=None)
theirs = 100 * jaccard_score(gt_all, cyl_all, average=None)
print (list(zip (classes, np.round(mine,2))))
print ("Mine", mine)
print ("Theirs", theirs)
IPython.embed()
