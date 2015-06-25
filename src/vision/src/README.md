#License
This project uses CppMT
CppMT is freely available under the [Simplified BSD license][1],
meaning that you can basically do with the code whatever you want.
If you use our algorithm in scientific work, please cite our publication
```
@inproceedings{Nebehay2015CVPR,
    author = {Nebehay, Georg and Pflugfelder, Roman},
    booktitle = {Computer Vision and Pattern Recognition},
    month = jun,
    publisher = {IEEE},
    title = {Clustering of {Static-Adaptive} Correspondences for Deformable Object Tracking},
    year = {2015}
}
```

# Dependencies
* OpenCV (>= 2.4.8, < 3)
* ROS (indigo)

# Building
In order to build the project you need both ROS and OpenCV installed.
It is also assumed that catkin is installed.
First clone the repo
then change the directory
```
cd QuadControl/
```
then make it using
```
catkin_make install
```
It should then be added to ROS as a package
# Usage
```
usage: rosrun CMT cmt [--remember-last]
```
## Optional arguments
* `--remember-last` remember the last target

## Target Selection
Press any key to stop the preview stream. Left click to select the
top left bounding box corner and left click again to select the bottom right corner.

## Abort
Right click on the image feed to land the drone

## Reselection of target
Press 'k' to start the reselection of a target

## Screenshot
Press 'c' to capture a screenshot
[1]: http://en.wikipedia.org/wiki/BSD_licenses#2-clause_license_.28.22Simplified_BSD_License.22_or_.22FreeBSD_License.22.29
