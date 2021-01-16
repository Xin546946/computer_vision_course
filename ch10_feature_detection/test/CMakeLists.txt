add_executable(test_LoG test_LoG.cpp)
target_link_libraries(test_LoG LoG ${OpenCV_LIBS} utils)
