# ToySlam

Under developing slam repository. This repository is for testing slam algorithms.
Currently works with stereo cameras. IMU will be supported in future.

# Tested algorithms

- **Square Root Marginalization for Sliding-Window Bundle Adjustment**, N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, In 2021 International Conference on Computer Vision (ICCV), [[arXiv:2109.02182]](https://arxiv.org/abs/2109.02182)

## Build - Tested in Ubuntu22.04, Windows

## Ubuntu

- ### update the Ubuntu package

```
sudo apt update
sudo apt install -y unzip cmake build-essential ninja-build libx11-dev xorg-dev libglu1-mesa-dev pkg-config libgtk2.0-dev qtbase5-dev libxcb-xinput0 libxcb-xinerama0

```

- ### build

```
cmake --build build -j
```

- ### note
  Fetch options can be toggled (defaults are ON):

```
cmake -DTOYSLAM_FETCH_OPENCV=ON -DTOYSLAM_FETCH_TBB=ON ..
```

- ### run euroc sample

```
in build folder
./euroc_vo
```

- ### directory

```
workspace
  |_ toy_slam
  |_ EUROC/V1_01_easy

```

## Windows

to be described
