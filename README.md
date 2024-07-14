# ToySlam

Under developing slam repository. This repository is for testing slam algorithms.
Currently works with stereo cameras. IMU will be supported in future.

# Tested algorithms

* **Square Root Marginalization for Sliding-Window Bundle Adjustment**, N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, In 2021 International Conference on Computer Vision (ICCV), [[arXiv:2109.02182]](https://arxiv.org/abs/2109.02182)

## Build - Tested in Ubuntu22.04, Windows

## Ubuntu 
+ ### update the Ubuntu package
```
sudo apt update
sudo apt install -y unzip cmake build-essential ninja-build libx11-dev xorg-dev libglu1-mesa-dev pkg-config libgtk2.0-dev qtbase5-dev libxcb-xinput0 libxcb-xinerama0 mesa-vulkan-drivers  vulkan-tools 

```
+ ### libs and source installation
```
git clone https://github.com/yunsuk-jeung/lib.git
bash lib/install_libs_ubuntu2204_release.sh

git clone https://github.com/yunsuk-jeung/toy_slam.git
cd toy_slam
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release
make -j8
```
+ ### run euroc sample
``` 
bash ./euroc_vo
```
+ ### directory
```
workspace
  |_lib
  |_toy_slam
  |_EUROC/V1_01_easy

```

## Windows
to be described