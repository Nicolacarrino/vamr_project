# VAMR miniproject
miniproject for vision algorithm and mobile robot course at ETH Zurich
author: Nicola Carrino, Sizhe Tian

## Run
**File Structure** 
> 	- kitti
	- malaga-urban-dataset-extract-07
	- parking
	- recording
	- vamr_project
		- main.m
		- processFrame.m
		- bundleAdjustment
		- keypoint
		- localization
		- parameters
		- plotting
		- pose
		- poseRefinement
		- projection

To run the pipeline, put dataset in corresponding folder, and run `vamr_project/main.m` in matlab
To change the dataset, change the variable `ds` of `vamr_project/main.m`
The parameters and settings for VO pipeline can be changed in `vamr_project/parameters`
To enable/disable pose refinement, change the parameter `poseRefinementFlag` at parameter file `vamr_project/parameters` for each dataset
To enable/disable bundle Adjustment, change the parameter `BA_flag` at beginning of file `vamr_project/main.m`



## Specifications
The specifications of the machine on which all the datasets have been run is:
maximum number of threads: 1
CPU frequency: 2.6 GHz
RAM: 8GB

## Dependencies
Matlab version : '9.6.0.1472908 (R2019a) Update 9'
**Toolbox used**
- image_toolbox
- matlab
- optimization_toolbox
- robotics_system_toolbox
- statistics_toolbox
- video_and_image_blockset