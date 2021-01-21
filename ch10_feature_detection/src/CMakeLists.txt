add_library(LoG LoG.cpp)
target_link_libraries(LoG ${OpenCV_LIBS} utils)


add_library(harris_corner harris_corner.cpp)
target_link_libraries(harris_corner ${OpenCV_LIBS} utils)

add_library(canny canny.cpp)
target_link_libraries(canny ${OpenCV_LIBS} utils)