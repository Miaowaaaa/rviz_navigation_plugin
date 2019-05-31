# rviz_navigation_plugin
![](https://img.shields.io/badge/ROS-Kinetic-brightgreen.svg) ![](https://img.shields.io/badge/Qt-5.5.1-orange.svg)

Descripiton
-----------
This is a complete package for `rviz_plugin_tutorial`. It has fixed the problem of panel `non-shown` in `rviz`. What's more,
it also provide a template `CMakeLists.txt` for compile correctly in `Qt4/Qt5`.

Attention
-----------
The main reason your plugin doesn't appear in rviz is follows:
 - forget source setup.bash
 - forget add `<exec_depend>rviz</exec_depend>` in `package.xml`
 
What can it do?
-----------
- Three buttons for rviz
- `Navigation Button` call a `patrol_service` for Robot Patrol
- `Record Voice` call a `voice_service` for Voice Command for Robot
- The usage of buttons can be easily changed for your work
