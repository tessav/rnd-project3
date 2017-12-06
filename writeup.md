## Project: Perception Pick & Place

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

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
First, I passed the point cloud through a statistical outlier filter (k-mean: 15, threshold factor: 0.3) to reduce noise, and then a voxel downsampling filter (leaf size: 0.01) to reduce cloud size. 
<br><br>
Next, I applied a passthrough filter (axis: z, axis min: 0.6, axis max: 1.1) to remove irrelevant points in the cloud. In order to identify and separate points that belong to a certain model (SACMODEL_PLANE), I ran the RANSAC plane segmentation method (max distance: 0.01) and extracted the outliers (the objects). The resulting point cloud can be seen below. 
<br><br>
<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/cloud_objects.png" width=500 />
<br><br>
#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
I first created a kd-tree with the white cloud and then called the euclidean clustering method (tolerance: 0.03, min cluster size: 10, max cluster size: 2000). In order to visualize each identified cluster separately, I created cluster-mask point clouds of different colors. The result can be seen below.
<br><br>
<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/cluster_cloud.png" width=500 />
<br><br>

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
For the models: biscuits, soap, soap2, glue, sticky_notes, snacks, eraser and book (those present in the pick list), I took 100 samples each (of different poses) and extracted histogram features (using hsv). These captured features are then used to train the SVM which resulted in ~95% accuracy. The confusion matrix of the above models are shown below. 
<br><br>
<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/confusion_matrix.png" width=600 />
<br><br>

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.
#### Pick up list 1 detection result:

<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/pickup_list_1.png" width=500 />
<br>

#### Pick up list 2 detection result:

<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/pickup_list_2.png" width=500 />
<br>

#### Pick up list 3 detection result:

<img src="https://raw.githubusercontent.com/tessav/rnd-project3/master/images/pickup_list_3.png" width=500 />
<br>

In order to construct the PickPlace request, I obtained the centroid of the object to be picked so that I can create the pick pose, used the drop box group to create the place pose, and based on the particular drop box, select whether to use the left or right arm.  

### For improvement: 
1. Currently, edges of the red / green boxes are detected as objects (sticky notes) so total number of objects is always 2 more than the real number. In order to remove the false detection, I should tweak the passthrough filter to include other filter axes and values.
2. Implement pick and place using collison map when I have time.


