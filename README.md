# BDIS-SLAM: A lightweight CPU-based dense stereo SLAM for surgery #

This is the CPU level dense stereo SLAM software for minimally invasive surgery.

 
  
## Compiling ##

The program was only tested under a 64-bit Linux distribution.
SSE instructions from built-in X86 functions for GNU GCC were used.


```
Compile:
chmod +x build.sh
./build.sh
```

The code depends on Eigen3 and OpenCV.
      

## Usage ##
For testing:      
chmod +x build.sh
./build.sh
./run_stereo_6.sh
./run_ColonSyn.sh
      

Citation:      
@article{song2022bdis,      
  title={{BDIS}: Bayesian Dense Inverse Searching Method for Real-Time Stereo Surgical Image Matching},      
  author={Song, Jingwei and Zhu, Qiuchen and Lin, Jianyu and Ghaffari, Maani},      
  journal={IEEE Transactions on Robotics},      
  volume={39},      
  number={2},      
  pages={1388--1406},      
  year={2022},      
  publisher={IEEE}      
}      
@inproceedings{song2022bayesian,      
  title={Bayesian dense inverse searching algorithm for real-time stereo matching in minimally invasive surgery},      
  author={Song, Jingwei and Zhu, Qiuchen and Lin, Jianyu and Ghaffari, Maani},      
  booktitle={International Conference on Medical Image Computing and Computer-Assisted Intervention},      
  pages={333--344},      
  year={2022},      
  organization={Springer}      
}      

## Usage ##
The code is mainly based on ORB-SLAM2(https://github.com/raulmur/ORB_SLAM2) and BDIS(https://github.com/JingweiSong/BDIS-v2).

## LICENCE CONDITIONS ##

This work is released under GPLv3 license. For commercial purposes, please contact the authors: jingweisong@yahoo.com, jingweisong.eng@outlook.com