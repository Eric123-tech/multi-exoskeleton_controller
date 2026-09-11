#include "../main/actuator_logic.h"
#include <cassert>

int main()
{
    using namespace actuator;
    Command c = {};
    assert(parse_command("P,20,0", 2, c) && c.motor_id == 0 && c.position_mm == 20);
    assert(parse_command("P, 35, 1 \t", 2, c) && c.motor_id == 1 && c.position_mm == 35);
    assert(parse_command("P,-10,0", 2, c) && c.position_mm == -10);
    const char *invalid[] = {"", "P", "P,", "P,25", "p,25,0", "P,20,-1",
        "P,20,2", "P,20,0.5", "P,20,1junk", "P,20,1,0", "P,nan,0",
        "P,inf,0", "P,1e999,0", "P,20,9999999999999999999999999", "P,20,-0"};
    for (const char *line : invalid) {
        c = {123, 123};
        assert(!parse_command(line, 2, c));
        assert(c.position_mm == 123 && c.motor_id == 123);
    }
    assert(parse_command("P,20,8", 9, c) && c.motor_id == 8);
    assert(!parse_command("P,20,9", 9, c));
    assert(parse_command("P,20,9", 10, c) && c.motor_id == 9);
    assert(!parse_command("P,20,10", 10, c));
    assert(matches_command_word("close", "close"));
    assert(matches_command_word("  CLOSE \t", "close"));
    assert(matches_command_word("Open", "open"));
    assert(!matches_command_word("close,1", "close"));
    assert(!matches_command_word("closed", "close"));
    assert(!matches_command_word("", "close"));
    assert(clamp_position(35, 30) == 27);
    assert(clamp_position(35, 50) == 35);
    assert(clamp_position(60, 50) == 47);
    assert(clamp_position(-10, 30) == 0);
    assert(clamp_position(27, 30) == 27);
    assert(clamp_position(47, 50) == 47);
    assert(pulse_ms(15, 30) == 1.5f);
    assert(pulse_ms(25, 50) == 1.5f);
    assert(std::fabs(pulse_ms(60, 30) - 1.9f) < 1e-6f);
    assert(std::fabs(pulse_ms(60, 50) - 1.94f) < 1e-6f);
    assert(pwm_count(pulse_ms(15, 30)) == 307);
    assert(pwm_count(pulse_ms(60, 30)) == 389);
    assert(pwm_count(pulse_ms(60, 50)) == 397);
    assert(smoothstep01(-1.0f) == 0.0f);
    assert(smoothstep01(0.0f) == 0.0f);
    assert(smoothstep01(0.5f) == 0.5f);
    assert(smoothstep01(1.0f) == 1.0f);
    assert(smoothstep01(2.0f) == 1.0f);
}
