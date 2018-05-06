# zeabus vision 2018

## Camera calibration
    
    1. Prepare chessboard where one side contains an even number of squares, both black and white, 
    and the other contains an odd number of squares and define size of square that use for calibration.  

    2. Set up stereo camera have a same clock for capture the images
    
    3. You have 2 ways to calibration
    
    4. The first way is collect 10 - 20 images for accurate calibration in MATLAB, After you have cameraParameters
    to remove lens distortion from the image. 
    
    5. (Recommanded) The second way is ROS stereo camera calibration [**camera_calibration**](http://wiki.ros.org/camera_calibration)
    
