
add_library(utils 
    display.cpp 
    tictoc.cpp
    opencv_utils.cpp
    math_utils.cpp
)
target_link_libraries(utils ${OpenCV_LIBS})