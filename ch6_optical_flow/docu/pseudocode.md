### 6. Tracking based on Optical Flow

----

~~~pseudocode
read video;
window = detect_object(first image,template);
feature_points = feature_points_extraction(first image,window,param);
last image = first image
for image : video{  // from the second image
	mask = get_mask(feature_points);
	iteration = 0;
 	while (not get enough points & not reach max iteration){
		iteration++;
		feature_points += feature_points_extraction(last image, weak_param, mask); 
	}
	if(still not get enough feature points){
        if(should exit){
        	exit;
        }else{
        	continue; // try next image
        }
    }
	pixel_motion = cal_optical_flow(last_image, image);
	delete_bad_feature_points();
	estimate_window_position();
}
~~~

### 1. Plan 

- [ ] 获取template的小图
- [ ] template matching 窗口初始化
- [ ] OpticalFlowTracker()
- [ ] FeaturePointsManager()
- [ ] test_optical_flow_tracker()
- [ ] window class

