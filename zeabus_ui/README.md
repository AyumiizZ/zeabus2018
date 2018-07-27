# zeabus vision 2018

## Table of Contents
**[Usage](#Usage)**<br>
**[Libraries](#libraries)**<br>

## Usage

### Before starting

1. Add command that your want to display on UI in `../zeabus_ui/src/command.csv`, in this file have 2 columns which the first column is `alias name` and the next column is `command (shell)` 
2. Run command `byobu-screen -S <yourname>` in terminal.
3. Run comamnd `roscore`

### Starting

1. Press `F2` for new tab then run command `rosrun zeabus_ui zeabus_ui.py`
2. On display have 2 sections.

    2.1 The `left` section show list of all node(s) and if you want to **kill node**, you can click node name in list.
    
    2.2 The `right` section contains a command button that are created from `command.csv`. 
    
    **P.S.  You can only switch 2 windows between `UI` and `terminal` that are runnig ui.**  

## Libraries 

* PyQt4
* pyautogui
* rosnode, rospy and rospkg

## Description

#### Coming soon ^^

## References

[**[1] PyQt4 tutorial**](https://www.tutorialspoint.com/pyqt/)

[**[2] Pyautogui**](https://automatetheboringstuff.com/chapter18/)

[**[2] ROS API**](http://docs.ros.org/api/)



