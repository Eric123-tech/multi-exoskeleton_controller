#pragma once

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cctype>

namespace actuator {

constexpr float EXTENSION_MARGIN_MM = 3.0f;

inline float clamp_position(float position, float stroke)
{
    if (position < 0.0f) return 0.0f;
    const float limit = stroke - EXTENSION_MARGIN_MM;
    return position > limit ? limit : position;
}

inline float smoothstep01(float progress)
{
    if (progress <= 0.0f) return 0.0f;
    if (progress >= 1.0f) return 1.0f;
    return progress * progress * (3.0f - 2.0f * progress);
}

inline float pulse_ms(float position, float stroke)
{
    // Physical stroke determines the mapping, not the reduced command limit.
    return 1.0f + clamp_position(position, stroke) / stroke;
}

inline uint16_t pwm_count(float pulse)
{
    return static_cast<uint16_t>(pulse / 20.0f * 4096.0f + 0.5f);
}

struct Command {
    float position_mm;
    size_t motor_id;
};

inline void skip_space(const char *&cursor)
{
    while (std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
}

// Match one case-insensitive command word with optional surrounding whitespace.
inline bool matches_command_word(const char *line, const char *word)
{
    const char *cursor = line;
    skip_space(cursor);

    while (*word != '\0') {
        if (std::tolower(static_cast<unsigned char>(*cursor)) !=
            std::tolower(static_cast<unsigned char>(*word))) {
            return false;
        }
        ++cursor;
        ++word;
    }

    skip_space(cursor);
    return *cursor == '\0';
}

// Only P,<finite position>,<zero-based integer ID>, with optional whitespace.
// Output is untouched on failure; no valid prefix of a malformed line is run.
inline bool parse_command(const char *line, size_t motor_count, Command &out)
{
    const char *cursor = line;
    if (*cursor++ != 'P') return false;
    skip_space(cursor);
    if (*cursor++ != ',') return false;
    skip_space(cursor);
    char *end = nullptr;
    errno = 0;
    const float position = std::strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !std::isfinite(position)) return false;
    cursor = end;
    skip_space(cursor);
    if (*cursor++ != ',') return false;
    skip_space(cursor);
    // Reject signs and decimal IDs, including negative zero.
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    const unsigned long id = std::strtoul(cursor, &end, 10);
    if (errno == ERANGE || id >= motor_count) return false;
    cursor = end;
    skip_space(cursor);
    if (*cursor != '\0') return false;
    out = {position, static_cast<size_t>(id)};
    return true;
}

} // namespace actuator
