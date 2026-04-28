#include <fstream>
#include <iostream>
#include <filesystem>

int main() {
    std::cout << "Current directory: " 
              << std::filesystem::current_path() << '\n';
    std::ifstream file("testfile.txt");
    if (!file) {
        std::cerr << "Cannot open file!\n";
        return 1;
    }

    int a, b, c;
    while (file >> a >> b >> c) {
        std::cout << a << ", " << b << ", " << c << '\n';
    }
}