#ifndef UTILS_H
#define UTILS_H

#include <memory>

// 仅在 C++11 中提供 make_unique (C++14 已经有了)
#if __cplusplus < 201402L
namespace std {
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}
#endif

#endif // UTILS_H
