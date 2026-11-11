<div align="center">
  <h1>ReFeree-Radar-SLAM</h1>
  <a href=""><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href=""><img src="https://img.shields.io/badge/-Linux-grey?logo=linux" /></a>
  <a href="https://sites.google.com/view/referee-radar"><img src="https://github.com/sparolab/Joint_ID/blob/main/fig/badges/badge-website.svg" alt="Project" /></a>
  <a href="https://ieeexplore.ieee.org/document/10705066"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2410.01325"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://www.alphaxiv.org/abs/2410.01325"><img src="https://img.shields.io/badge/alphaXiv-2408.07330-darkred" alt="alphaXiv" /></a>
  <a href="https://www.youtube.com/watch?v=aQ0OlHYJCYI"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />
  
  **[IEEE RA-L]** This repository is the official code for **ReFeree**: Radar-Based Lightweight and Robust Localization Using Feature and Free space.

  <a href="https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko" target="_blank">Hogyun Kim*</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=JCJAwgIAAAAJ&hl=ko" target="_blank">Byunghee Choi*</a><sup></sup>,
  <a href="https://scholar.google.co.kr/citations?view_op=list_works&hl=ko&user=rcp7sWAAAAAJ" target="_blank">Euncheol Choi</a><sup></sup>,
  <a href="https://scholar.google.com/citations?user=W5MOKWIAAAAJ&hl=ko" target="_blank">Younggun Cho</a><sup>†</sup>

  (* represents equal contribution.)

  **[Spatial AI and Robotics Lab (SPARO)](https://sites.google.com/view/sparo/%ED%99%88?authuser=0&pli=1)**
    
  <p align="center">
    <img src="fig/referee_radar_slam.gif" alt="animated" width="40%" />
  </p>

</div>

## Organization
- This repository is using the structure of [Navtech-Radar-SLAM](https://github.com/gisbi-kim/navtech-radar-slam). The odometry and pose-graph optimization part is [Yeti-Odometry](https://github.com/keenan-burnett/yeti_radar_odometry) and iSAM2, respectively.
- The Radar Place Recognition parts of this repository are [referee.hpp](yeti_radar_odometry/include/tf_eigen_manager.h) and [referee.cpp](yeti_radar_odometry/src/referee.cpp), and matching part in [odometry.cpp](yeti_radar_odometry/src/odometry.cpp).

## Preliminary
This repository built and complied in environment **Ubuntu 20.04, ROS Noetic(desktop-full)**.
The required `apt install` packges are below.
```
$ sudo apt-get install python3-catkin-tools
$ sudo apt-get install ros-noetic-gtsam
```

## How to use
First, clone and build the repository in your ros workspace.
```
$ cd your_ros_workspace/src
$ git clone https://github.com/sparolab/ReFeree-Radar-SLAM.git
$ cd your_ros_workspace
$ catkin build nano_gicp && catkin build
```

Second, prepare dataset in following format:  
```
(dataset_name)/  
├── polar/  
│ ├── (timestamp_start).png  
│ ├── ...  
│ └── (timestamp_start).png
```
The timstamp-named png radar image files should be located in (dataset_name)/polar directory.
And revise `seq_dir` and `save_directory` parameters in [yeti_radar_odometry.launch](yeti_radar_odometry/launch/yeti_radar_odometry.launch).
```
<param name="seq_dir" type="string" value="(path_of_your_dataset)/(dataset_name)/" />
...
<param name="save_directory" type="string" value="(path_of_your_results)/results/"/>
```

Third, launch the [yeti_radar_odometry.launch](yeti_radar_odometry/launch/yeti_radar_odometry.launch).
```
$ source devel/setup.bash
$ roslaunch yeti yeti_radar_odometry.launch 
```

## Special Thanks
* [Yeti-Odometry](https://github.com/keenan-burnett/yeti_radar_odometry)
* [Nano-GICP](https://github.com/engcang/nano_gicp)
* [Navtech-Radar-SLAM](https://github.com/gisbi-kim/navtech-radar-slam)
  

## Citation
```
@article{kim2024referee,
  title={ReFeree: Radar-Based Lightweight and Robust Localization using Feature and Free space},
  author={Kim, Hogyun and Choi, Byunghee and Choi, Euncheol and Cho, Younggun},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}
```

## Contact
* Hogyun Kim (hg.kim@inha.edu)
* Byunghee Choi (bhbhchoi@inha.edu)

