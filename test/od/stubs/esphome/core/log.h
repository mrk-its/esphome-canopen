#pragma once
#include <cstdio>
#define ESP_LOGE(tag, ...) (std::printf("E %s: ", tag), std::printf(__VA_ARGS__), std::printf("\n"))
#define ESPHOME_LOG_TAG(name, tag) static constexpr const char *const name = tag
