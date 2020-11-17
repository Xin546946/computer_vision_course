add_executable(test_cv test_cv.cpp)
target_link_libraries(test_cv 
    ${OpenCV_LIBS}
    lvlset
    sdfmap
    utils
)

add_executable(test_lbf test_lbf.cpp)
target_link_libraries(test_lbf 
    ${OpenCV_LIBS}
    lvlset
    sdfmap
    utils
)

add_executable(test_sdf test_sdf.cpp)
target_link_libraries(test_sdf 
    ${OpenCV_LIBS}
    lvlset
    sdfmap
    utils
)