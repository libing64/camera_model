# camera_model

相机内参标定和双目标定

支持多种相机模型:
* pinhole model
* kannala-brandt model
* mei model
* scaramuzza model

支持多种标定板
* chessboard
* circles grid
* asymmetric circles grid
* aruco makers
* charuco makrers

## 编译 & 安装
依赖项
* OpenCV (with opencv_contrib)
* Ceres Solver
* boost

```
cd camera_model
mkdir build
cd build
cmake ..
make
sudo make install
```

## 调用
编译安装之后， camera_model会作为共享库被安装在系统目录, 使用时无需添加源码，只添加依赖关系即可
```
cmake_minimum_required(VERSION 3.0)
project(camera_model_example)

add_executable(example example.cc)
target_link_libraries(example camera_model)
```

# 参考 & 感谢
part of [camodocal](https://github.com/hengli/camodocal)

感谢原作者 [Lionel Heng](https://github.com/hengli)

感谢 [YZF](https://github.com/dvorak0)


# 用法
## 相机内参标定 intrinsic_calib

Use [intrinsic_calib.cc](https://github.com/libing64/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.


## 双目标定 stereo_calib

Use [stereo_calib.cc](https://github.com/libing64/camera_model/blob/master/src/stereo_calib.cc) to calibrate your camera.

## Undistortion:

See [Camera.h](https://github.com/libing64/camera_model/blob/master/include/camera_model/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)

# 案例
# 相机内参标定 intrinsic calib

## mei model
```
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model mei --camera-name fisheye_mei -v
```

## pinhole model
```
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model pinhole --camera-name fisheye_pinhole -v
```

## kannala-brandt
```
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model kannala-brandt --camera-name fisheye_kannala-brandt -v
```
## scaramuzza

```
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model scaramuzza --camera-name fisheye_scaramuzza -v --view-results
```

## asymmetric cirlces grid
```
./intrinsic_calib -w 4 -h 11 -i ../data/asymmetric_circles_grid -p asymmetric_circles_grid -e jpg --pattern asymmetric_circles_grid --camera-model pinhole --camera-name webcam -v --view-results
```

注意，这里一定要4x11, 设置为11x4就不行

## circles grid
```
./intrinsic_calib -w 9 -h 6 -i ../data/circles_grid -p circles_grid -e jpg --pattern circles_grid --camera-model pinhole --camera-name circles_grid_cam -v --view-results
```

## charuco marker
```
./intrinsic_calib -w 5 -h 7 -s 0.04 --marker-size 0.024 -i ../data/charuco -p charuco -e jpg --pattern charuco --dp ../data/pattern/detector_params.yml -d 10 --camera-model pinhole --camera-name charuco_cam -v --view-results
```


## stereo calibration
```
./stereo_calib -i ../data/stereo_images/ -e jpg --prefix-l left --prefix-r right --camera-model mei -v --view-results
```

# TODO
- [ ] add aruco marker
