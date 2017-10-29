## Project: Perception Pick & Place

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
This project is to detect the objects on the table, generate the instructions like which box it has to be dropped, and the position of the box for three different world scenarios in the specified order.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
First converted the point cloud to voxel ( "voxel" is short for "volume element") with a size of 0.01 to get more points.

![VOXEL](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/voxel.png)

Then applied the passthrough filter along z axis with a range of (0.6 to 1.1) to get only the tabletop with objects on it and remove the bottom of the table and it looks like below.
![Passthrough](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/passthrough-filter.png)

Then used the RANSAC - Random sample consensus algorithm  to get only the objects we are concerned on the table and remove the table top as well 

The RANSAC algorithm assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers can be defined by a particular model with a specific set of parameters, while outliers do not fit that model and hence can be discarded. Like in the example below, we can extract the outliners that are not good fits for the model.
![RANSAC](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/ransac-img.png)

After the RANSAC is applied it looks like this:
![my-ransac](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/my-ransac.png)

Refer to the code in this python script : [RANSAC-Exercise-Solution](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/RANSAC.py)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

In this exercise, now as we got the objects on the table top, we need to segment the point cloud to invidudal objects for this we use the Euclidean Clustering algorithm.

This Euclidean clustering algorithm uses the k-d tree, to reduce the computational burden of searching for neighboring points.
Inorder to get the k-d tree, we need to convert the XYZRGB point cloud to XYX as the PCL's Euclidean Clustering algorithm requires a point cloud with only spatial information ( for this used the functions from pcl_helper.py). 

Get the individual clusters by applying the cluster tolerance to 0.04 and cluster size range (50,2500).

Inorder to visualize the cluster better assigned a color to each individaul object cluster.

Then published those pcl cluster by converting to ros's pointcloud2 and it looks like this:

![Ecludiean Clustering](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/ecludiean-cluster.PNG)

Refer to the code in this python script : [Segmentation-Exercise2-Solution](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/segmentation.py)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

In this exercise, we need to recognize each object and label them.
We use the following algorithms to get the object.
1) Apply the historgram using the  hsv color to get the exact object points
2) Then apply the Surface Normals to get the exact shape of the objects.
3) Then apply the Support Vector Machine or "SVM" a supervised machine learning algorithm that allows to characterize the parameter space of the dataset into discrete classes
Utilized the Scikit-Learn/sklearn package in Python for the SVM.
After this we can use the trained dataset to recognize the objects in our scene.
It looks like this:

![Object Recognition](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/final-object-recog-svm.PNG)

Refer to the code in this python script :

  [Features](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/features.py)

  [Capture Features](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/capture_features.py)

  [Object Recognition-Exercise3-Solution](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/object_recognition.py)


### Pick and Place Setup

## Steps:

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
    $ catkin_make

 Get the Project Frame work from github:

      $ cd ~/catkin_ws/src  
      $ git clone https://github.com/udacity/RoboND-Perception-Project.git

Next install missing dependencies using rosdep install:

    $ cd ~/catkin_ws
    $ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

Build the project:

    $ cd ~/catkin_ws
    $ catkin_make

Add the following line to your .bashrc file:

    export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
  
After coding to verify the code do the following steps:
 
    $ roslaunch pr2_robot pick_place_project.launch

    $ rosrun pr2_robot project_template.py
    
    
#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

To get the output_*.yaml for three worlds, applied all the functions created in the Exercise-1, Exercise-2, Exerise-3, followed the same  process for training objects in three worlds, then created the output instructions by matching the detected objects  with the paramter servers information from picklist and dropbox list to get the exact location in which box to drop and what arm should be moved.

The generated files are as below:

1) [output_1.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world1/output_1.yaml)

2) [output_2.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world2/output_2.yaml)

3) [output_3.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world3/output_3.yaml)


### Discuss the algorithms used in the code.

I faced with noise and the side edges of the boxes in the view of the robot, which were causing problem to detect the exact number of objects from the table.

## Noise 

Inorder to remove the noise from the point cloud, applied the StatisticalOutlierRemoval filter  with a mean of 1 and standard deviation of 1.

Refer to the section in [project script](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/project_template.py) :   Statistical Outlier Filtering  

## Box edges

Inorder to remove the edges of the boxes from the view of the pr2-robot camera, applied the passthrough filter along z axis, then x axis and then y axis. By this process, the robot was able to find the exact number of objects on the table and below are screen shots.

| Before x,y passthrough filter   |   After  x,y passthrough filter      |
|---------------------------------|:------------------------------------:|
| ![Before filter applied ](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/pr-run2.PNG)| ![After filter applied ](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/pr3-run.PNG) |


Refer to the section in [project script](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/scripts/project_template.py) :   PassThrough Filter - #for X-Axis, #for Y-Axis  

As a result following steps are applied to get the output instructions for robot to pick and place the objects in the boxes.
1) Convert ros messages to pcl

2) Apply the Statistical outlier filter to remove the noise

3) Apply the Voxel Grid Downsampling to get the accurate volume points to detect the objects

4) Apply the Passthrough filters 

    i) Along Z - Axis to get the Table with Objects and remove the bottom of the table with a rangle of 0.6 to 0.8.
    
    ii) Along X - Axis with a range of 0.3 to 1.0 and Y - Axis to get the with a range of -0.5 to 0.5

5) Applied the RANSAC - Random sample consensus algorithm  to get only the objects we are concerned on the table and remove the table top as well

6) Extract the inliers ( the objects) and the outliers (the table top)

7) Apply the Euclidean Clustering algorithm Get the individual clusters by applying the cluster tolerance to 0.04 and cluster size range (50,2500).

8) Apply the colours to the invidivual cluster to have a better visual.

9) Prediction:
   Inorder to predict the object in the scene, we need to train the system, here the captured the different features of the expected objects with Histograms using hsv colors and surface normals to get the shape of the objects from the cluster.
   Then used the SVM algorithm, which allows to characterize the parameter space of the dataset into discrete classes (refer to train_svm.py)
   As a result we get the trained dataset, which is passed to the prediction algorigthm to match the objects in the scene and label them with the approriate names and publish.
   
 10) Pass the detected objects to the pr2_mover function where it will generate the instructions(output_*.yaml) for the robot to pick and place the object as expected.
 
 11) pr2_mover:
    For each detected objects:
      match the object's  label with the pick_list from parameter server, if matched take the group ( red/blue) which denote in which box the object has to be dropped, then match the group with dropbox_lot from paramter server to get the exact positions of the destination box and which arm to be used, at the same time, get the location(pick_position) of the object from the table from the object's point cloud.
      
      Then pass the pick position, object name, the arm to be used, the place postion  to function to generate the instructions in .yaml file and save the file .
      
 
 Pictures of World 1, 2 and 3 objects after the detection:
 
| World 1  |   World 2    | World 3   |
|----------|:------------:|-----------|
| ![World 1 ](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/pr3-run.PNG) | ![World 2](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/world2-objects.PNG) | ![World 3](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/world3-objects.PNG) |
 
 
 
### Improvement:
  
  1) I need to improve the ranges in the algorithm to detect the exact numbers in the world 3.
  2) I want to complete the next challenge of pick and place in the boxes after all the projects are done.
  
