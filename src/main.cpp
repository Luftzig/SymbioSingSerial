#include <Arduino.h>
#include <FlowIO_2022.1.24/FlowIO.h>
#include <etl/optional.h>
#include <cstdlib>


#ifndef DEVICE_NAME
#define DEVICE_NAME "FlowIO_Serial"
#endif // DEVICE_NAME

#ifndef DEBUG
#define DEBUG true
#endif // DEBUG

using etl::optional;

FlowIO flow_io;

constexpr size_t COMMAND_BUFFER_LENGTH = 64;
char command_buffer[COMMAND_BUFFER_LENGTH];

struct Instruction {
    char command;
    uint8_t pwm;
    uint8_t ports;
};

Instruction STOP_ALL{'!', 0, 0b00011111};

void setup() {
    Serial.begin(115200);
    flow_io = FlowIO(INFLATION_SERIES);
}

/*
 * Commands:
 * "name?" -> Print the device name
 * "state?" -> Print device status (current command)
 * "!all" -> Stop all
 * "!"|"+"|"a"|"-"|"^",<n>,<p> -> Stop, inflate, actuate, vacuum or release with n=pwm (0..255)
 *      and p=bbbbb where b=0|1 is port state, from left to right ports 1,2,3,4,5
 */

optional<Instruction> nextInstruction{etl::nullopt};

bool is_actuate_inflation() {
    auto config = flow_io.getConfig();
    return (config == INFLATION_SERIES || config == INFLATION_PARALLEL || config == GENERAL);
}

#define DEBUG_println(s) if (DEBUG) { Serial.print("DEBUG: "); Serial.println(s); }

bool parse_binary(const String &s, unsigned long *out) {
    *out = strtoul(s.c_str(), nullptr, 2);
    return true;
}

bool parse_pwm_port(const String &s, uint8_t *pwm_out, uint8_t *ports_out) {
    size_t first_comma = s.indexOf(',');
    if (first_comma < 0) {
        return false;
    }
    size_t second_comma = s.indexOf(',', first_comma + 1);
    if (second_comma < 0) {
        return false;
    }
    const String &pwm_str = s.substring(first_comma + 1, second_comma);
    DEBUG_println("pwm_str = " + pwm_str);
    *pwm_out = atoi(pwm_str.c_str());
    String ports_str = s.substring(second_comma + 1);
    DEBUG_println("port_str = " + ports_str);
    unsigned  long ports;
    parse_binary(ports_str, &ports);
    *ports_out = (uint8_t)ports;
    return true;
}

bool parse_stop_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    DEBUG_println("stop");
    if (parse_pwm_port(s, &pwm, &ports)) {
        nextInstruction = {'!', pwm, ports};
        DEBUG_println(ports);
        return true;
    } else {
        return false;
    }
}

bool parse_inflate_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    DEBUG_println("inflate");
    if (parse_pwm_port(s, &pwm, &ports)) {
        nextInstruction = {'+', pwm, ports};
        DEBUG_println(ports);
        return true;
    } else {
        return false;
    }
}

bool parse_vacuum_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    if (parse_pwm_port(s, &pwm, &ports)) {
        nextInstruction = {'-', pwm, ports};
        return true;
    } else {
        return false;
    }
}

bool parse_release_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    if (parse_pwm_port(s, &pwm, &ports)) {
        nextInstruction = {'^', pwm, ports};
        return true;
    } else {
        return false;
    }
}

bool parse_command(const char *data) {
    String command{data};
    command.trim();
    if (command.startsWith("!all")) {
        DEBUG_println("STOP ALL");
        nextInstruction = STOP_ALL;
        return true;
    } else if (command.startsWith("name?")) {
        Serial.println(DEVICE_NAME);
        return true;
    } else if (command.startsWith("state?")) {
        Serial.println(flow_io.getHardwareState(), BIN);
        return true;
    }  else if (command[0] == '!') {
        return parse_stop_command(command.substring(1));
    } else if (command[0] == '+') {
        return parse_inflate_command(command.substring(1));
    } else if (command[0] == '-') {
        return parse_vacuum_command(command.substring(1));
    } else if (command[0] == '^') {
        return parse_release_command(command.substring(1));
    } else if (command[0] == 'a') {
        if (is_actuate_inflation()) {
            return parse_inflate_command(command.substring(1));
        } else {
            return parse_vacuum_command(command.substring(1));
        }
    } else {
        return false;
    }
}

struct run_every {
    uint32_t every_ms;

    void (*callback)();

    void update(uint32_t current_time) {
        if (current_time - last_run > every_ms) {
            callback();
            last_run = current_time;
        }
    }

    run_every(uint32_t every_ms, void (*cb)()) : every_ms(every_ms), callback(cb), last_run(0) {}

private:
    uint32_t last_run = 0;
};

bool green_led_state = false;

run_every blink_led{500, []() {
    flow_io.pixel(0, 5 * green_led_state, 0);
    green_led_state = !green_led_state;
}};

void loop() {
    blink_led.update(millis());
    if (Serial.available()) {
        size_t num_read = Serial.readBytesUntil('\n', command_buffer, COMMAND_BUFFER_LENGTH);

        if (num_read > 0) {
            bool succeeded = parse_command(command_buffer);

            if (succeeded) {
                Serial.println("ok");
                if (nextInstruction.has_value()) {
                    flow_io.command(nextInstruction->command, nextInstruction->ports, nextInstruction->pwm);
                }
            } else {
                Serial.println("error: Failed to parse");
            }

            nextInstruction = etl::nullopt;
            std::fill_n(command_buffer, COMMAND_BUFFER_LENGTH, '\0');
        }
    }
    flow_io.optimizePower(150, 200);
}

