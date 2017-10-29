## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
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
Followed the instructions specified in this lesson:
https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/e3e5fd8e-2f76-4169-a5bc-5a128d380155/concepts/474ba039-cc5c-4fa0-b8d6-3f5173abe513

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

To get the output_*.yaml for three worlds, applied all the functions created in the Exercise-1, Exercise-2, Exerise-3, followed the same  process for training objects in three worlds, then created the output instructions by matching the detected objects  with the paramter servers information from picklist and dropbox list to get the exact location in which box to drop and what arm should be moved.

The generated files are as below:

1) [output_1.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world1/output_1.yaml)

2) [output_2.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world2/output_2.yaml)

3) [output_3.yaml](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/world3/output_3.yaml)


*** 
Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  
***
I faced with noise and the side edges of the boxes in the view of the robot, which were causing problem to detect the exact number of objects from the table.

**noise 

Inorder to remove the noise from the point cloud, applied the StatisticalOutlierRemoval filter  with a mean of 1 and standard deviation of 1.

Refer to the section in project.py :   Statistical Outlier Filtering  

**Box edges

Inorder to remove the edges of the boxes from the view of the pr2-robot camera, applied the passthrough filter along z axis, then x axis and then y axis. By this process, the robot was able to find the exact number of objects on the table and below are screen shots.

| Before x,y passthrough filter   |   After  x,y passthrough filter      |
|---------------------------------|:------------------------------------:|
| ![Before filter applied ](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/pr-run2.PNG)| ![After filter applied ](https://github.com/perfalcon/RoboND-Perception-P2-Project/blob/master/images/pr3-run.PNG) |


Refer to the section in project.py :   PassThrough Filter - #for X-Axis, #for Y-Axis  

