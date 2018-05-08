# zeabus vision 2018

## Hardware

* uEye Industrial Cameras [**UI-3260CP Rev.2**](https://en.ids-imaging.com/store/ui-3260cp-rev-2.html)
* Arduino Nano

## Software

* Robot Operating System [**ROS**](http://www.ros.org) 
* MATLAB
* Arduino IDE

## Libraries 

* OpenCV
* Numpy
* Matplot

## Camera calibration
    
* Prepare chessboard where one side contains an even number of squares, both black and white, and the other contains an odd number of squares and define size of square that use for calibration.  

* Set up stereo camera have a same clock for capture the images that use arduino send the trigger to two cameras.
    
* You have 2 ways to calibration
    
1. The first way is collect 10 - 20 images for accurate calibration in MATLAB, After you have cameraParameters to remove lens distortion from the image. 
   
2. (Recommanded) The second way is ROS stereo camera calibration [**camera_calibration**](https://wiki.ros.org/camera_calibration/)
    
    *  Run the cameracalibrator.py node for calibrate a stereo camera:
        
        ```
        rosrun camera_calibration cameracalibrator.py --size 8x7 --approximate=0.01 --square 0.90 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw right_camera:=/my_stereo/right left_camera:=/my_stereo/left _approximate_sync:=True _queue_size:=1
        ```
    * After finished the calibration, click `SAVE`. The images and calibration parameter are compressed and save to  `/tmp/calibrationdata.tar.gz`.
 
    * Then extract `calibrationdata.tar.gz` that have `left.yaml` and `right.yaml`, copy them into directory `camera_info/` in your camera parameter directory (e.g. zeabus_vision/camera_info)
    
    * Convert file `.yaml` to `.ini` by
        
        ```
        rosrun camera_calibration_parsers convert *.yaml *.ini
        ```
    * Move `*.ini` into directory `camera_conf/`
    
3. Create `.launch` file for open stereo camera.

    * Use stereo_image_proc pacakge
    
    ```
    <node pkg="stereo_image_proc" name="stereo_image_proc" type="stereo_image_proc" output="screen" ns="stereo" args="_approximate_sync:=True _queue_size:=10"></node>
    ```
