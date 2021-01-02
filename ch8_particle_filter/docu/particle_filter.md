### 粒子滤波跟踪

~~~pseudocode
tracking(video,initial bbox,template){
	initialize particles with gauss distribution(init_bbox);
	for(each image in video)
		particles = update state(particles,motion model)
		particles = update weight(particles,observation model,image,template)
		particles = resampling(particles)
	}
}
~~~

