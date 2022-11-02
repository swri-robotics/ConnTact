cmake_minimum_required(VERSION 3.0)
project(spiral_search_example)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_python_setup()
catkin_install_python(
    PROGRAMS
        nodes/spiral_search_node
        nodes/corner_search_node
        nodes/plotting_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
