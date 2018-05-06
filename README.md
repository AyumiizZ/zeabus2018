# zeabus vision 2018

## Hardware

* uEye Industrial Cameras [**UI-3260CP Rev.2**](https://en.ids-imaging.com/store/ui-3260cp-rev-2.html)
* Arduino Nano

## Software

* Robot Operating System [**ROS**](http://www.ros.org) 
* MATLAB
* Arduino IDE

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
    * After calibration use have two file `left.yaml` and `right.yaml`
    
