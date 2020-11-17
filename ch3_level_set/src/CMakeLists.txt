
add_library(gdc_base gradient_descent_base.cpp)

add_library(sdfmap sdf_map.cpp)
target_link_libraries(sdfmap ${OpenCV_LIBS})

add_library(lvlset 
    level_set_utils.cpp
    level_set_cv.cpp
    level_set_lbf.cpp
)
target_link_libraries(lvlset 
    ${OpenCV_LIBS} 
    utils 
    sdfmap 
    gdc_base
)



