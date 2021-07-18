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
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model mei --camera-name fisheye_mei -v


## pinhole model
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model pinhole --camera-name fisheye_pinhole -v


## kannala-brandt
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model kannala-brandt --camera-name fisheye_kannala-brandt -v

## scaramuzza
./intrinsic_calib -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model scaramuzza --camera-name fisheye_scaramuzza -v


## example osmo pocket
## mei model
./intrinsic_calib -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model mei --camera-name osmo_mei -v


## pinhole model
./intrinsic_calib -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model pinhole --camera-name osmo_pinhole -v


## kannala-brandt
./intrinsic_calib -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model kannala-brandt --camera-name osmo_kannala-brandt -v

## scaramuzza
./intrinsic_calib -w 9 -h 6 -i ../data/osmo -p osmo_ -e jpg --camera-model scaramuzza --camera-name osmo_scaramuzza -v


## asymmetric cirlces grid
```
./intrinsic_calib -w 4 -h 11 -i ../data/asymmetric_circles_grid -p asymmetric_circles_grid -e jpg --pattern asymmetric_circles_grid --camera-model pinhole --camera-name webcam -v --view-results
```

注意，这里为何一定要4x11, 设置为11x4就不行

## circles grid
```
./intrinsic_calib -w 9 -h 6 -i ../data/circles_grid -p circles_grid -e jpg --pattern circles_grid --camera-model pinhole --camera-name circles_grid_cam -v --view-results
```

## charuco
```
./intrinsic_calib -w 5 -h 7 -s 0.04 --marker-size 0.024 -i ../data/charuco -p charuco -e jpg --pattern charuco --dp ../data/pattern/detector_params.yml -d 10 --camera-model pinhole --camera-name charuco_cam -v --view-results
```


# stereo calibration
```
./stereo_calib -i ../data/stereo_images/ -e jpg --prefix-l left --prefix-r right --camera-model mei -v --view-results
```

# TODO
* add detector, square, circle, charuco
明天把支持不同pattern类型的代码给加上
