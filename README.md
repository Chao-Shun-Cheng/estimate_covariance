# Cross Covariance

NCKU Meclab package for estimating the covariance of pose from RSU.

## Introduction
This package make MKZ ndt pose regarded as ground truth, and receive the data from RSU to estimate the covariance. 

### Requirement
* `ndt_pose` is the good accuracy.
* **MKZ** is static.
* **RSU** is able to detect MKZ including position, label ,and dimensions. 

### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`/ndt_pose`|`geometry_msgs::PoseStamped`|the pose from the result of ndt_matching.|
|`/RSU_topic_name`|`autoware_msgs::DetectedObjectArray`|Tracking objects list from lilee RSU.|

### Parameter 
* ***distance_threshold*** : this parameter is to choose which object is MKZ.
* ***RSU_topic_name*** : this parameter is subscribed by this package. 
* ***save_path*** : raw data are saved in this path. **Notice** the path needs to create before you start to run.

