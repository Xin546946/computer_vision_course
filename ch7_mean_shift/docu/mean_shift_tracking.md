### 7. Mean Shift Tracking

-------------

~~~pseudocode
Input: video, template
Output: position of the tracking object in each frame

output = mean_shift_tracking(video, template){
	while(there is no much difference bewteen two centers){
		center = template_matching(first image, template image);
		pdf of temp image = histogram and back_projection(template image);
		pdf of candidate image = histogram and back_projection(dandidate window of a bigger area around template image)
		matching score 1 = calculate the similarity of two images (pdf of temp image, pdf of candidate image);
    	weight = calculate the weight in each position(histogram of candidate image);
    	shift vector = calculate weighted average at the position xi(xi,weight,kernel function);
    	matching score 2 = calcualte the similarity of two images (pdf of temp image, pdf of shifted candidate image);
    	if(matching score has reduced){
    		break;
    	} else{
    		new mean point = average( center of mass and center of window)
    	}
     }
}
~~~

