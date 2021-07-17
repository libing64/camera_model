part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use [intrinsic_calib.cc](https://github.com/dvorak0/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)


# intrinsic calibration

## mei model
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model mei --camera-name fisheye_mei -v


## pinhole model
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model pinhole --camera-name fisheye_pinhole -v


## kannala-brandt
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model kannala-brandt --camera-name fisheye_kannala-brandt -v

## scaramuzza
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model scaramuzza --camera-name fisheye_scaramuzza -v


## example osmo pocket
## mei model
./Calibration -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model mei --camera-name osmo_mei -v


## pinhole model
./Calibration -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model pinhole --camera-name osmo_pinhole -v


## kannala-brandt
./Calibration -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model kannala-brandt --camera-name osmo_kannala-brandt -v

## scaramuzza
./Calibration -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model scaramuzza --camera-name osmo_scaramuzza -v


## cirlces grid

./Calibration -w 4 -h 11 -i ../data/circles_grid -p circles_grid -e jpg --pattern asymmetric_circles_grid --camera-model pinhole --camera-name webcam -v
注意，这里为何一定要4x11, 设置为11x4就不行



# stereo calibration
```
./stereo_calib -i ../data/stereo_images/ -e jpg --prefix-l left --prefix-r right --camera-model mei
```

# TODO
* add detector, square, circle, charuco
明天把支持不同pattern类型的代码给加上
