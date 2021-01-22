add_library(haar_feature haar_feature.cpp)
target_link_libraries(haar_feature ${OpenCV_LIBS} utils)
