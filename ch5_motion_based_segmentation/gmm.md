## 5. GMM

~~~pseudocode
1. initialize GMM model(num_model, {mean},{var},{w})
2. for each frame of video:
3.	   for each pixel:
4. 		   if is_in_GMM():
5.				update_GMM();
6.			update_param();
7.		get_foreground();
~~~



Q: 是否再加一个for 用来for 1:num_gaussian