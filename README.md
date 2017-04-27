# SenseData
Get sensor Data from Google Tango Device, including IMU, Color Camera and Depth Camera data.

And handle the data with [ROVIO](https://github.com/ethz-asl/rovio).

Now it is still in development mode.

## Dependencies
1. `OpenCV-android-sdk`: download OpenCV for android SDK, unzip it to `app` folder.
2. `Eigen3`: download [eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) and compile it with cmake [android cross compile](https://github.com/taka-no-me/android-cmake), then install it with prefix `/opt/android`
3. `yaml-cpp`: download [yaml-cpp](https://github.com/jbeder/yaml-cpp) and compile it with cmake android cross compile, then install it with prefix `/opt/android`.
4. `kindr`: download [kindr](https://github.com/ethz-asl/kindr), and compile it with cmake, then install it with prefix `/opt/android`. 
5. `boost`: come from [rtabmap](https://github.com/introlab/rtabmap)
    ```
    wget -nv https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz
    tar -xzf boost_1_59_0.tar.gz
    cd boost_1_59_0
    wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/489ff2869eccd6f8d03ffb9090ef839108762741/BoostConfig.cmake.in
    wget -nv https://gist.github.com/matlabbe/0bce8feeb73a499a76afbbcc5c687221/raw/e7fbf0e301cfea417a7aa69989a761a4de08b8c3/CMakeLists.txt
    ```
    then compile it with cmake and install it with prefix `/opt/android`.
6. handle the `std::to_string` not found error in `kindr/common/source_file_pos.hpp`, add following code to the file.
    ```
    namespace std {
        template <typename T>
        string to_string(T value) {
          std::ostringstream os ;
          os << value ;
          return os.str() ;
        }
    }
    ```

## Platform
Mac OS X 10.12.4 with Android Studio 2.3.1.
Ubuntu 14.04 with Android Studio 2.3.1.

