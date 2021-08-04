# D-BKI
Dynamic Semantic Mapping using Closed Form Bayesian Inference & Scene Flow

<!-- <img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki.png" width="300">
<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm_variance.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki_variance.png" width="300"> -->


### Data

**Gazebo Simulator**: I store the data collected from a Gazebo simulation environment in the form of a custom point cloud structure with 7 fields: X, Y, Z, VX, VY, VZ & L, where the first three are positional information with respect to the robot frame, the next three fields are the egomotion-compensated scene flow fand the last field is the semantic label. Data is collected with a block laser scanner and filtered with passthrough filters to remove NaNs. Please download data from this link(TODO) to get pointclouds in **".pcd"** format.

**SemanticKITTI dataset**: The SemanticKITTI dataset can be downloaded from https://semantickitti.org. The data is provided in the correct format and can be downloaded from this link(TODO). Additionally, in the **predictions** folder, you can also find the corresponding scene flow information for each scan in **".bin"** format.

You can drop all this data into the **data/** folder.

### Compiling the package

```bash
catkin_ws/src$ git clone https://github.com/katanachan/BKIDynamicSemanticMapping
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Running Gazebo Simulation Mapping

```bash
$ roslaunch semantic_bki multi_tb3_home.launch
```

<!-- ## Semantic Mapping using KITTI dataset

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_05.png" width=320><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/kitti_15.png" width=540>

### Download Data
Please download [data_kitti_15](https://drive.google.com/file/d/1dIHRrsA7rZSRJ6M9Uz_75ZxcHHY96Gmb/view?usp=sharing) and uncompress it into the data folder. -->

### Running SemanticKITTI Mapping
```bash
$ roslaunch semantic_bki semantickitti_node.launch
```
You will see semantic map in RViz. It also queries each ground truth point for evaluation, stored at data/semantickitti_01/evaluations.

### Using Custom Sequences
If you are experimenting with other sequences in SemanticKITTI, make sure to open the [launch file](https://github.com/katanachan/BKIDynamicSemanticMapping/blob/master/launch/semantickitti_node.launch) and modify the following: <img src="https://raw.githubusercontent.com/katanachan/BKIDynamicSemanticMapping/master/github/launchfile.jpg" width=320> 

and then, the config file as described in the next section.

### Tuning Parameters
All parameters can be tuned from the **.config** file located in **config/**.
<ol>
  <li><strong>sequence_no</strong>: Enter an integer here specifying sequence number if you're modifying the SemanticKITTI configuration file</li>
  <li><strong>resolution</strong>: the resolution of the map that is built</li>
<li><strong>spatiotemporal</strong>: set to true if you have scene flow data, else false (for static mapping)</li>
<li><strong>query</strong>: set to false if you don't want to do a quantitative evaluation every scan</li>
<li><strong>visualize</strong>: set to true if you want to visualize the map in RViz</li>
<li><strong>moving_classes</strong>: a list of the class IDs that are considered "dynamic." If you wish, you can set movable static objects to dynamic. Note: if your free space sampling resolution is high (eg. 1-10), put 0 in the list of dynamic classes.</li>
<li><strong>free_resolution</strong>: set free space sampling resolution (if you set to a high value like 100, there will be no free space sampling resolution)</li>
<li><strong>ds_resolution</strong>: downsampling resolution for any incoming point cloud. Up to you.</li>
<li><strong>ell & flow_ell</strong>: ell is for the original map. Keep it about 2.5x the map resolution and flow_ell around the same.
<li><strong>flow_sf2</strong>: increase this if traces aren't disappearing.
</ol>

### Evaluation
Evaluation code is provided in TODO. You may modify the directory names to run it. Check out [semantic-kitti-api](https://github.com/PRBonn/semantic-kitti-api) for documentation about evaluation.

## Relevant Publications

If you found this code useful, please cite the following:

TODO

Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping ([PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8954837))
```
@ARTICLE{gan2019bayesian,
author={L. {Gan} and R. {Zhang} and J. W. {Grizzle} and R. M. {Eustice} and M. {Ghaffari}},
journal={IEEE Robotics and Automation Letters},
title={Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping},
year={2020},
volume={5},
number={2},
pages={790-797},
keywords={Mapping;semantic scene understanding;range sensing;RGB-D perception},
doi={10.1109/LRA.2020.2965390},
ISSN={2377-3774},
month={April},}
```

Learning-Aided 3-D Occupancy Mapping with Bayesian Generalized Kernel Inference ([PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8713569))
```
@article{Doherty2019,
  doi = {10.1109/tro.2019.2912487},
  url = {https://doi.org/10.1109/tro.2019.2912487},
  year = {2019},
  publisher = {Institute of Electrical and Electronics Engineers ({IEEE})},
  pages = {1--14},
  author = {Kevin Doherty and Tixiao Shan and Jinkun Wang and Brendan Englot},
  title = {Learning-Aided 3-D Occupancy Mapping With Bayesian Generalized Kernel Inference},
  journal = {{IEEE} Transactions on Robotics}
}
```
