#include "tictoc.h"
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

long long run(long long from, long long to) {
    long long sum = 0;
    for (long long i = from; i < to; i++) {
        sum += i;
    }
    std::cout << "Thread ID: " << std::this_thread::get_id() << ", sum = " << sum << std::endl;
    return sum;
}

int main() {
    int num_thread = 0;

    std::cout << "Number of CPU: " << std::thread::hardware_concurrency() << std::endl;
    std::cout << "Please input the number of thread you want to start: ";
    std::cin >> num_thread;

    std::vector<std::future<long long>> result;
    tictoc::tic();
    for (long long i = 0; i < num_thread; i++) {
        result.push_back(std::async(run, i * 1e9 / num_thread, (i + 1) * 1e9 / num_thread));
    }
    long long sum = 0;
    for (long long i = 0; i < num_thread; i++) {
        sum += result[i].get();
    }
    std::cout << "Using " << num_thread << " threads costs " << tictoc::toc() / 1e6 << " microseconds" << '\n';
    std::cout << "The sum of number from 0 to 1e9 is: " << sum << '\n';
    return 0;
}
