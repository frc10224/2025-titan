#pragma once

#include <rev/SparkMax.h>
#include <functional>

// optimization hints
#define if_hot(x) if (__builtin_expect(!!(x), 1))
#define if_cold(x) if (__builtin_expect(!!(x), 0))

// typedefs
template<typename T>
using Fn = std::function<T()>;

enum TUNE_SETTINGS {
    TUNE_P = 1,
    TUNE_I = 2,
    TUNE_D = 4,
    TUNE_FF = 8,
};

#ifdef PID_TUNE
void pid_tune_init(std::string subsystem, int settings, 
        double p, double i, double d, double ff, std::string motor);
void pid_tune(rev::spark::SparkMax &spark, int settings, std::string subsystem, std::string motor = "");
#else
#define pid_tune(...)
#define pid_tune_init(...)
#endif /* PID_TUNE */