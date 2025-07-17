#include "blast_rush.h"

void print(int a, bool endl = true) {
    std::cout << a;
    if (endl) {
        std::cout << std::endl;
    }
}

void print(float a, bool endl = true) {
    std::cout << a;
    if (endl) {
        std::cout << std::endl;
    }
}

template <typename T>
void print(std::vector<T> a, bool endl = true) {
    if (a.size() == 0) {
        std::cout << "error: impossible to print vector with size 0" << std::endl;
        return;
    }
    std::cout << "[ ";
    for (int i = 0; i < a.size()-1; i++) {
        print(a[i], false);
        std::cout << ", ";
    }
    print(a.back(), false);
    std::cout << " ]";
    if (endl) {
        std::cout << std::endl;
    }
}