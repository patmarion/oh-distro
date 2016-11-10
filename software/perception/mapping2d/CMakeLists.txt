cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mapping2d)
include(cmake/pods.cmake)

add_subdirectory(src/test)



# Unit Testing
include(CTest)
#set(test_data_dir ${CMAKE_SOURCE_DIR}/../../../../drc-testing-data)  # using ~/ probably won't work, instead define it relative to the project dir
add_test(NAME test_testSimpleIsamDemo
               COMMAND simple-isam-demo)
