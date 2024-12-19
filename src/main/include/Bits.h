#include <functional>

// optimization hints
#define if_hot(x) if (__builtin_expect(!!(x), 1))
#define if_cold(x) if (__builtin_expect(!!(x), 0))

// typedefs
template<typename T>
using Fn = std::function<T()>;