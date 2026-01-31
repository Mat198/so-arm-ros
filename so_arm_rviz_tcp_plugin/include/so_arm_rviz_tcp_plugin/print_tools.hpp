#ifndef PRINT_TOOLS_HPP
#define PRINT_TOOLS_HPP

#include <vector>
#include <array>
#include <sstream>

namespace print {

const static std::string RED = "\033[91m";
const static std::string GREEN = "\033[92m";
const static std::string YELLOW = "\033[93m";
const static std::string BLUE = "\033[94m";
const static std::string PURPLE = "\033[95m";
const static std::string CYAN = "\033[96m";
const static std::string ENDC = "\033[0m";

inline std::string red(const std::string & text){
    return RED + text + ENDC;
}

inline std::string green(const std::string & text){
    return GREEN + text + ENDC;
}

inline std::string yellow(const std::string & text){
    return YELLOW + text + ENDC;
}

inline std::string blue(const std::string & text){ 
    return BLUE + text + ENDC;
}

inline std::string purple(const std::string & text) {
    return PURPLE + text + ENDC;
}

inline std::string cyan(const std::string & text){
    return CYAN + text + ENDC;
}

template<typename T>
std::string vector2Str(const std::vector<T>& vector, const int precision = 3) {

    std::stringstream outStream;
    outStream << std::fixed << std::setprecision(precision);
    outStream << "[";
    for (size_t i = 0; i < vector.size(); i++) {
        outStream << vector[i];
        if (i != vector.size() - 1) {
            outStream << ", ";
        }
    }
    outStream << "]";
    return outStream.str();
}

template<typename T, size_t N>
std::string array2Str(const std::array<T,N> & array, const int precision = 3){
  std::stringstream outStream;
  outStream << std::fixed << std::setprecision(precision);
  outStream << "{";
  for (size_t i = 0; i < array.size(); i++){
    outStream << array[i];
    if (i != array.size() - 1){
      outStream << ", ";
    }
  }
  outStream << "}";
  return outStream.str();
}

inline std::string bool2Str(const bool value) {
    
    return value ? "true" : "false";   
}

inline std::string paintBool(const bool flag) {
    std::stringstream stream;
    stream << std::boolalpha;
    if (flag) {
        stream << GREEN;
    } else {
        stream << RED;
    }
    stream << flag << ENDC;
    return stream.str();
}

} // namespace print

#endif // PRINT_TOOLS_HPP
