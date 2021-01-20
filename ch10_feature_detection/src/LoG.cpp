#include "LoG.h"
#include "opencv_utils.h"

void LoG::run() {
    // todo implement LoG
    // step 1: 用Gaussian函数对图像进行平滑，抑制噪声
    // step 2: 对经过平滑的图像使用Laplacian算子
    // step 3: 二值化图像（一般以0为阈值）
    // step 4: 使用形态学方法（膨胀腐蚀）得到图像边缘
}