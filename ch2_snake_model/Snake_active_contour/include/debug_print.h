#include <iostream>

#define show_str(str) std::cerr << '\n' << #str << " : " << str << '\n';
#define show(var) std::cerr << '\n' << #var << " : " << var << '\n';
#define show_vec(vec)                                                  \
    do {                                                               \
        cerr << '\n';                                                  \
        for (int i = 0; i < vec.size(); ++i) {                         \
            std::cerr << #vec << "[" << i << "] : " << vec[i] << '\t'; \
            if ((i + 1) % 5 == 0) std::cerr << '\n';                   \
        }                                                              \
        std::cerr << '\n';                                             \
    } while (0)
