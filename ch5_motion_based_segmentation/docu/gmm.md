## 5. GMM

~~~pseudocode
1. initialize GMM parameter with first frame 
2. for each frame of video:{
3.	   for each pixel:{
4. 		   if !is_in_gmm():{
5.				replace_model();
			}
6.			else:{
7.				update_gmm();
			}
		}
8.		get_foreground();
	}
~~~



Q: 是否再加一个for 用来for 1:num_gaussian