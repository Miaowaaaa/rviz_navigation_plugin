# rviz_navigation_plugin

Descripiton
-----------
This is a complete package for rviz_plugin_tutorial. It has fixed the problem of panel non-shown in rviz. What's more,
it also provide a template CMakeLists.txt for compile correctly in Qt4/Qt5.

Attention
-----------
The main reason your plugin doesn't appear in rviz is follows:
 - forget source setup.bash
 - forget add '<exec_depend>rviz</exec_depend>' in 'package.xml'
