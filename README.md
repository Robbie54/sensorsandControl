# sensorsandControl

## Code structure 
The code is structered in one script with a few supporting functions following the code standard with proper code structure and syntax. 
### Variable setup 
Variables are initialised first with the creation and extraction of the relevant rosbag topics before moving on to setting up the camera intrinsics taken from the ros bag topics and various other relevant empty variables.
### Point Cloud Selection 
The key point cloud frames are manually selected and the region of interested points are allocated.
The code then sets up the point clouds and cuts out points outside of the region of interest to remove artefacts from the depth image data.
### Point Cloud Alignment and display
Next the point clouds are aligned and overlayed on top of each other with a supporting function create point cloud to help change the variable type.
Finally each point cloud is displyed and overlayed ontop of each other to create the final point cloud. 

## Individual contributions 
Contributions were often done as a group with individual contributions being heavily research and experimental based to bring to our meetings. Individually there was many failed attempts that did not make it into the final assignments script.

Robert focused on the initial phases of the project learning how to set up topics and initial point cloud functions, as well as attempting various methods for rgb overlay and alignement using the rot and trans in the tsv and auroura topic. 33%

Milo focused on researching how to remove interferance in the point cloud images with the region of interferance. He also spent time on various methods of aligning the point clouds and learning     about the em sensor and how to calibrate it. 33%

Max focused on going through the point clouds and removing unusable ones. He also worked on aligning the point clouds properly, and spent time on looking into the availabe rosbag topics. 33%

# Authors: 

Milo Boyd = miloboyd

Max Deda = zum78

Robert Howe = Robbie54

