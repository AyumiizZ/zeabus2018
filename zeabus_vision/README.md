# zeabus vision 2018

## Table of Contents
**[Hardware](#hardware)**<br>
**[Software](#software)**<br>
**[Libraries](#libraries)**<br>
**[Auto Exposure](#auto-exposure)**<br>
**[Color Range](#color-range)**<br>
**[Bag2JPG](#bag2jpg)**<br>
**[Bag2mp4](#bag2mp4)**<br>
**[Camera calibration](#camera-calibration)**<br>
**[Description](#description)**<br>
**[CameraInfo Message](#camerainfo-message)**<br>

## Hardware

* uEye Industrial Cameras [**UI-3260CP Rev.2**](https://en.ids-imaging.com/store/ui-3260cp-rev-2.html)
* Kowa C-Mount 6mm
* Arduino Nano


## Software

* Robot Operating System [**ROS**](http://www.ros.org) 
* MATLAB
* Arduino IDE


## Libraries 

* OpenCV
* Numpy
* Matplot


## Auto Exposure

* ZEABUS's auto exposure (*ZEABUS's AE*) is changed by mode of V that is channel of HSV Color in frame.
* The main program contain in `auto_exposure.py` that have a `AutoExposure` Class, You can use it with camera which you want to use ZEABUS's AE by create a new file and insert `AutoExposure` Class. See [*auto_exposure_front.py*](https://github.com/zeabusTeam/zeabus2018/blob/vision/zeabus_vision/src/auto_exposure_front.py).
* ZEABUS's AE is called by `stereo.launch`, If you want to **Disable** please comment code below:

```
<node pkg="zeabus_vision" name="auto_exposure_front" type="auto_exposure_front.py" output="screen">
	<param name="topic_right" type="str" value="/stereo/right/image_rect_color/compressed"/>
	<param name="client_right" type="str" value="ueye_cam_nodelet_$(arg nodelet_manager_name_right)/"/>
</node>
```
* If you want to use `auto exposure` but code be commented, can use command : ```rosrun zeabus_vision auto_exposre_front.py```


## Color Range


	**(A)** mission.launch ---------_-- run ---> task or mission file.py ----- mission name ----------
					|								  |------
	**(B)** color_range_*.launch --| -- run ---> color_range_main.py ---------- mission name ---------	|
					|									|
					V									|
	call color param from param/<interger 1,2,3,...>/\*.yaml **(C)**					|
														|
														|
		-------------------------------------------------------------------------------------------------
		|
		V
	after_preprocess ---> hsv ---> task or mission file.py -----> Can use get_color_range() in vision_lib.py
			       |
			       |
			       V
			color_range_main.py  ----> You click or slide hsv trackbar for get color  ------
													|
													|
													V
				set color param <--- save to param in sub directory 1, 2, or ... same **(C)**
	
	

* Color range have 2 part, **1. Get color value and 2. Use Color Value**. 
* The color value is saved `*.yaml` files in `params` directory (folder) in sub directory `1` or `2` or interger for seprerate color range value for each period (morning or noon) .

### 1. Get color value.

* In this part have three files are `color_range_main.py`, `color_range_front.launch` and `color_range_bottom.launch`.
* Have `pre_process(mission)` function in `vision_lib.py` for preprocess image before convert to hsv and use in your task or mission files when you use color range value. 

		explain 1. You preprocess image before convert to hsv in path.py 
			2. You get color from hsv in color_range_main.py also you copy code preprocess from path.py into color_range_main.py for hsv color from path.py is equal to hsv color from color_range_main.py
			3. I will description agian in part 2. Use Color Value
			
* *color_range_main.py* use for get color from image's topic (real or bag) then save to `*.yaml` 
* *color_range_\*.launch* that launch (run) *color_range_main.py* and config parameter for front and bottom cameras. Command is 

```
	roslaunch zeabus_vision color_range_<front or bottom>.launch mission:='<mission name>' number:='<directory name 1, 2, 3, ...>'
```


![alt text](https://raw.githubusercontent.com/zeabusTeam/zeabus2018/vision/zeabus_vision/images/color_range_example.png)


* Above image show color range main that launched by color_range_front.launch.

**Manual**

1. Run command for launch `color_range_main.py`
2. Press <color key> for set color that you want to get color into `mask` window. Please see the `m<->c` status is 0 or m, The m<->c status default is 1.
3. Now, you can get color by `left click` or slide trackbar. Please see below.
4. After, you satisfied in color press in *step 2* again for back to `color` window and save (press s), check your terminal show `<------------ save ------------>`  
5. Then, if you to get other color press <color key> until `m<->c` status is 2 or c, following step 2-4 again.
	
*P.S.* get color when `m<->c` status is 0 or m. and save when `m<->c` status is 2 or c.

* In the `image` window represent HSV image after pre_process, current color value, `m<->c` status that show now you choose mask(m or 0) window or color(c or 2) window. 
* `press <color key>`   <color key> is first letter of colors name. (e.g. red -> r, violet -> v)	
* `left-click`          click on image window for get color that you want.
* `press z`             undo
* `press x`             redo
* `press s`             save
* `press c`             clear color value set to lower : 179, 255, 255 and upper 0, 0, 0 
* `press q`		exit program. if not save cannot exit but you can `Ctrl+C` in termnal for exit.

### 2. Use color Value

* Have three files `mission.launch`(launch file for run task file), `color_front_example_color_range.yaml`(color range value from step 1.) and `example_color_range.py`(task file)
* Please read `example_color_range.py` file that edit from `roulette.py.py` and search `part of color range` for code about color range, in `line 191` show how to get color range 
* Run task file by 

```
	roslaunch zeabus_vision mission.launch mission:='example_color_range' number:='1' camera_position:='front'
```

### Refer

##### File
	
	python: color_range_main.py, task or mission.py and vision_lib.py (pre_process(), get_color_range(), 

	launch: color_range_front.launch, color_range_bottom.launch and mission.launch
	
	yaml: params/<1,2,3,...>/ *.yaml
	

##### Command
	
	get color : 

	roslaunch zeabus_vision color_range_<front or bottom>.launch mission:='<mission name>' number:='<directory name 1, 2, 3, ...>'
	
	
	use color (mission) : 

	roslaunch zeabus_vision mission.launch mission:='example_color_range' number:='1' camera_position:='front'

## Bag2JPG

* roslaunch zeabus_vision bag2img.launch *bag:=<bag path> topic:=<topic name> republish:=<status> out:=<output path>*

* `bag` is path of bag file.
* `topic` is topic that you want convert to `*.JPG` in bag file. Not include `/Compressed`.
* `republish` is status of republish that convert image compressed to image raw. Set `0` if bag record `Compressed`, 1 if bag record `Raw`.
* `out` is output path. Please specific `absolute path`.	
	
* **e.g.** Convert bag that record compressed images to images.

```
roslaunch zeabus_vision bag2img.launch *bag:=../bag/dice00.bag topic:=/top/center/image_raw republish:=0 out:=/home/skconan/img*
```


* **e.g.** Convert bag that record compressed images to images.

```
roslaunch zeabus_vision bag2img.launch *bag:=../bag/stereo00.bag topic:=/my_stereo/right/image_raw republish:=1 out:=/home/skconan/img*
```


## Bag2mp4

* Please `catkin_make` before use it.
* Command `rosrun zeabus_vision bag2mp4 filename.bag topicname fps output.mp4`


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


## Description

#### Focal length in millimeter

* Focal length of Kowa is 6 mm 
* Focal length have influence to *Field of view (FOV)*
* If number of Focal length is a low, number of FOV is high
* **Ps. In intrinic camera matrix use Focal length in pixel unit not mm unit.** 


#### Optical center or Principal point

* Optical center is the central of a lens. It's represented *O* in physics.

	
#### Skew coefficient

* Skew coefficient is non-zero if the image axes are not perpendiuclar *(From MATLAB)*.


#### Field of view (FOV)

* FOV *(HxV)* of Kowa is 96.8° x 79.4°
* FOV have vertical and horizontal FOV, that I use vFOV and hFOV instead of them.
	* vFOV = *2 * arctan( height of image [px] / (2 * working distance [mm])*
	* hFOV = *2 * arctan( width of image [px] / (2 * working distance [mm])*


### Intrinic camera

* Intrinic camera is a 3x3 matrix that used to map between coordinates in the image to physical world coordinates.
	
	
### CameraInfo Message

* sensor_msgs/CameraInfo.msg is the message of ROS ,contains information about *calibration parameter* that requires D, K, R, and P

* D is distortion parameters.

```
	D = [k1, k2, t1, t2, k3]
```
	* k1, k2 and k3 are radial distortion coeffients.
	* t1 and t2 are tangenial distortion coefficients.
	
	
* K is intrinsic camera metrix.

```
	    [Fx  0   Cx]
	K = [S   Fy  Cy]	
	    [0   0   1 ]
```

	* Fx, Fy are Focal length in px.
	* Cx, Cy are Optical center.
	* S is skew coefficient.

* R is rectification matrix or rotation matrix.

* P is projection matrix or camera matrix. 

## References

[**[1] ROS Camera calibration**](http://wiki.ros.org/camera_calibration)

[**[2] ROS CameraInfo**](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

[**[3] MATLAB Camera calibration**](https://www.mathworks.com/help/vision/ug/camera-calibration.html)

[**[4] MATLAB Stereo Camera calibrator**](https://www.mathworks.com/help/vision/ug/stereo-camera-calibrator-app.html)

[**[5] how-to-photograpy-6-focal-length**](http://www.tamemo.com/post/116/how-to-photograpy-6-focal-length/)
