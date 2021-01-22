add_executable(test_intergral_img test_intergral_img.cpp)
target_link_libraries(test_intergral_img ${OpenCV_LIBS} utils)

add_executable(test_haar_feature test_haar_feature.cpp)
target_link_libraries(test_haar_feature haar_feature ${OpenCV_LIBS} utils)