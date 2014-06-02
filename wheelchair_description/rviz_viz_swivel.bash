rosrun xacro xacro.py urdf/wheelchair_with_swivels.xacro > urdf/model.urdf

roslaunch urdf_tutorial display.launch model:=urdf/model.urdf
