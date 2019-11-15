# ArucoGroundTruthLibrary

C++ library abstracting ArUco markers and boards as [BayesFilters](https://github.com/robotology/bayes-filters-lib) measurement models.

<p align="center"><img src="https://github.com/xenvre/aruco-ground-truth-library/blob/master/misc/example.png"/></p>

### Dependencies

- [`BayesFilters`](https://github.com/robotology/bayes-filters-lib)
- [`Eigen 3`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`OpenCV`](https://opencv.org/)
- [`RobotsIO`](https://github.com/xenvre/robots-io)

#### Installation

```
git clone https://github.com/xenvre/aruco-ground-truth-library.git
cd aruco-ground-truth-library
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=<installation_path> ../
make install
```

In order to use the library within a `CMake` project
```
find_package(ArucoGroundTruthLibrary REQUIRED)
(...)
target_link_libraries(... ArucoGroundTruthLibrary::ArucoGroundTruthLibrary ...)
```

### Classes

- `ArucoMeasurement`: base class implementing a `BayesFilters::MeasurementModel` that provides the pose of the camera and the pose of the marker/board with respect to the camera;
- `ArucoMarkerMeasurement`: an `ArucoMeasurement` from images containing a marker;
- `ArucoBoardMeasurement`: an `ArucoMeasurement` from images containing a board (offsets in the ArUco dictionary are supported in order to detect multiple boards at the same time);
- `ReverseLinkMeasurement`: a base class implementing a `BayesFilters::MeasurementModel` and accepting an `ArucoMeasurement` that relates the pose of the marker with the pose of a link of interest;
- `ThreePointReverseLinkMeasurement`: a class inheriting from `ReverseLinkMeasurement` that accepts three points (origin, tip of x and y axis) describing the plane of the marker/board;

