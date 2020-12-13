#include <numeric>
#include <vector>
template <typename T>
T mean(const std::vector<T>& vec) {
    assert(!vec.empty());
    T sum = std::accumulate(vec.begin(), vec.end(), vec[0]) - vec[0];
    return sum / vec.size();
}