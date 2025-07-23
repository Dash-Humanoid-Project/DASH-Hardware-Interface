#pragma once

template <typename T>
void logPrintSingle(const T& value) {
#ifdef UPXTREME_i14
    std::cout << value;
#elif defined(TEENSY_4_1)
    Serial.print(value);
#else
    #error "Unknown platform: define either UPXTREME_i14 or TEENSY_4_1"
#endif
}

template <typename T, typename... Args>
void logPrint(const T& first, const Args&... rest) {
    logPrintSingle(first);
    if constexpr (sizeof...(rest) > 0) {
        logPrint(rest...);  // recursively print remaining arguments
    }
}

template <typename... Args>
void logPrintln(const Args&... args) {
    logPrint(args...);
#ifdef UPXTREME_i14
    std::cout << std::endl;
#elif defined(TEENSY_4_1)
    Serial.println();
#else
    #error "Unknown platform: define either UPXTREME_i14 or TEENSY_4_1"
#endif
}

#define PRINT(...) logPrint(__VA_ARGS__)
#define PRINTLN(...) logPrintln(__VA_ARGS__)