//
// Created by Richard Mule on 3/28/25.
//

#include "ShaderLoader.hpp"

#include <fstream>


namespace utils
{
    std::vector<char> readFile(const std::string& fileName)
    {
        std::ifstream file(fileName, std::ios::ate | std::ios::binary);
        if ( !file.is_open() )
            throw std::runtime_error("Failed to open " + fileName);
        const long fileSize = file.tellg();
        std::vector<char> shaderCode(fileSize);
        file.seekg(0);
        file.read(shaderCode.data(), fileSize);
        file.close();
        return shaderCode;
    }

}