// Tool to embed the model library into a C++ header file
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.so> <output.h>" << std::endl;
        return 1;
    }
    
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {
        std::cerr << "Cannot open input file: " << argv[1] << std::endl;
        return 1;
    }
    
    // Read the entire file
    input.seekg(0, std::ios::end);
    size_t size = input.tellg();
    input.seekg(0, std::ios::beg);
    
    std::vector<uint8_t> buffer(size);
    input.read(reinterpret_cast<char*>(buffer.data()), size);
    input.close();
    
    // Write as C++ header
    std::ofstream output(argv[2]);
    if (!output) {
        std::cerr << "Cannot open output file: " << argv[2] << std::endl;
        return 1;
    }
    
    output << "// Auto-generated from " << argv[1] << std::endl;
    output << "#pragma once" << std::endl;
    output << "#include <cstdint>" << std::endl;
    output << "#include <cstddef>" << std::endl << std::endl;
    output << "namespace franka_model {" << std::endl;
    output << "constexpr size_t MODEL_LIBRARY_SIZE = " << size << ";" << std::endl;
    output << "constexpr uint8_t MODEL_LIBRARY_DATA[] = {" << std::endl;
    
    for (size_t i = 0; i < size; ++i) {
        if (i % 16 == 0) output << "    ";
        output << "0x" << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(buffer[i]);
        if (i < size - 1) output << ",";
        if (i % 16 == 15 || i == size - 1) output << std::endl;
        else output << " ";
    }
    
    output << "};" << std::endl;
    output << "} // namespace franka_model" << std::endl;
    
    output.close();
    
    std::cout << "Generated " << argv[2] << " with " << size << " bytes" << std::endl;
    return 0;
}