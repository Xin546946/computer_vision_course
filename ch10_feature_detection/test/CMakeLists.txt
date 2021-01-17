add_executable(test_LoG test_LoG.cpp)
target_link_libraries(test_LoG LoG ${OpenCV_LIBS} utils)

add_executable(test_harris_corner test_harris_corner.cpp)
target_link_libraries(test_harris_corner harris_corner ${OpenCV_LIBS} utils)