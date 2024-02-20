#include <iostream>

int main() {
    // 宣告並初始化變數
    int a = 10;
    int b = 3;

    // 進行數學計算
    int sum = a + b;       // 加法
    int difference = a - b; // 減法
    int product = a * b;   // 乘法
    int quotient = a / b;  // 除法
    int remainder = a % b; // 取餘數

    // 輸出結果
    std::cout << "a = " << a << std::endl;
    std::cout << "b = " << b << std::endl;
    std::cout << "a + b = " << sum << std::endl;
    std::cout << "a - b = " << difference << std::endl;
    std::cout << "a * b = " << product << std::endl;
    std::cout << "a / b = " << quotient << std::endl;
    std::cout << "a % b = " << remainder << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
