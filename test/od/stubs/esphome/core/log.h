#pragma once
#include <cstdio>
#define ESP_LOGE(tag, ...) (std::printf("E %s: ", tag), std::printf(__VA_ARGS__), std::printf("\n"))
