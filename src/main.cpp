#include "../include/Types/Matrix.hpp"
#include <iostream>

Matrix<int> myMat = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
};

int main() {
    std::cout << myMat << std::endl;

    return 0;
}