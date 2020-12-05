## 5. GMM 作业代码部分

* 说明
  * 交作业的时候请简短说明作业完成的情况
  * 作业要求⽤报告的形式提交, 对运⾏结果展⽰并加以分析, 报告中可以写⼀下⾃⼰对各个部分
    的理解和⼀些问题. 并附上原始的代码包.
  * 作业中需要完成的部分都⽤ //todo 标记
* 完成GMM的相关函数，其中代码框架已给出如下

~~~c++
void GMM::add_sample(double sample) {
    model_param_.sort_with_priority();
    int id = get_gm_id(sample);
    if (id == -1) {
        replace_model(sample);
    } else {
        update_gmm(sample, id);
    }
}
~~~

* 作业部分

  * 完成add_sample里的函数

  ~~~c++
  int GMM::get_gm_id(double sample)
  ~~~

  ~~~c++
  void GMM::replace_model(double sample)
  ~~~

  ~~~c++
  void GMM::update_gmm(double sample, int id)
  ~~~

  * 完成判断sample是否属于前景的函数

  ~~~c++
  bool GMM::is_in_foreground(double sample)
  ~~~

