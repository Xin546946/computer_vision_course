#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

void hello_thread() {
    std::cout << "Hello thread!" << std::endl;
}

int multi_sum(int a, int b) {
    int c = a + b;
    std::cout << a << " + " << b << " = " << c << '\n';
    return c;
}

int main(int argc, char** argv) {
    int c;

    do {
        c = getchar();
        putchar(c);
    } while (c != '.');

    std::thread t1(hello_thread);
    t1.join();

    std::cout << "Main here!" << std::endl;

    getchar();
    return 0;
}