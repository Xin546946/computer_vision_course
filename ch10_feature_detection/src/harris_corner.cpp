#include "harris_corner.h"
#include "opencv_utils.h"

void HarrisCornerDetector::run() {
    // todo implement harris corner detector
    // step 1: compute x and y derivatives of image
    // step 2: subtract the mean from each image gradient
    // step 3: compute M (covariance) matrix for each pixel
    // step 4: compute eigenvalues and eigenvectors or other criterions such as in slides
    // step 5: use threshold on eigenvalues to detect corners
}