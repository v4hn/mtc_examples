cmake_minimum_required(VERSION 3.0.2)
project(mtc_examples)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_task_constructor_core
  roscpp
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# this package consists of one-file examples only
function(example name)
    add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
    add_dependencies(${PROJECT_NAME}_${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${PROJECT_NAME}_${name} ${catkin_LIBRARIES})
    set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
endfunction()

example(merger)
example(cartesian_move_to)
example(ik_clearance)
example(path_length)
example(modifying_wrapper)
