
#ifndef BOOST_FIX_H
#define BOOST_FIX_H

#include <locale>
#include <sstream>
#include <string>
#include <ios>
#include <vector>
#include <map>

// 强制包含这些头文件以解决boost lexical_cast问题
namespace boost_fix {
    template<typename T>
    std::string to_string(const T& value) {
        std::ostringstream ss;
        ss << value;
        return ss.str();
    }

    template<typename T>
    T from_string(const std::string& str) {
        T value;
        std::istringstream(str) >> value;
        return value;
    }
}

#endif // BOOST_FIX_H
