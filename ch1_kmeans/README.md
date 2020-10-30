### 作业1 : k-means 代码实现部分

* **任务**: 实现k-means算法中的关键函数

* **说明**:

  *  test_k_means.cpp 是已经完成了的测试函数, 所有需要同学们完成的函数均在k_means.cpp中. 建议大家先看看test_k_means.cpp中调用的

    ~~~c++
    kmeans.run(iteration, convergence_radius);
    ~~~

    这个函数是理解k-means 的关键. 我们所要实现的就是run() 中的各个模块.

  * 所有的需要完成的模块都用TODO 进行了标记, 代码中有更详细的备注.

    1. ~~~cpp
       void Kmeans::initialize_centers();	
       ~~~

       初始化每个簇的中心点, 基本实现方法已经给出, 如果能实现更好的初始化方法可以获得加分.  请分析不同方法的收敛情况和聚类效果.

    2. ~~~cpp
       void Kmeans::update_labels();
       ~~~

       根据每次迭代后新的中心位置, 重新为每个样本分配新的中心.

    3. ~~~cpp
       void Kmeans::update_centers();
       ~~~

       根据每次迭代后聚类的结果, 重新计算新的中心位置.

    4. ~~~cpp
       bool Kmeans::is_terminate(int current_iter, int max_iteration, float smallest_convergence_rate);
       ~~~

       在每次迭代后判断是否收敛或达到最大迭代次数从而退出迭代.

  * 完成后请参考视频中的方式运行自己的函数.

  * 可以在代码的基础上探索ppt 24页的内容, 对于某一张特定的图像, 分析不同k值的结果. 

  * 最后请将作业写成报告的形式,最好是pdf 格式,将运行的结果图片展示出来, 并加以说明. 并请附上代码.



