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
    
* The first way is collect 10 - 20 images for accurate calibration in MATLAB, After you have cameraParameters to remove lens distortion from the image. 
   
* (Recommanded) The second way is ROS stereo camera calibration [**camera_calibration**](https://wiki.ros.org/camera_calibration/)
    
