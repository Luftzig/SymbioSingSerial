#include "constants.h"
#include "etl/algorithm.h"
#include "Adafruit_ADS1X15.h"
#include <Arduino.h>
#include <FlowIO_2022.1.24/FlowIO.h>
#include <etl/optional.h>
#include <etl/variant.h>
#include <etl/container.h>
#include <cstdlib>
#include <SPI.h>


#ifndef DEVICE_NAME
#define DEVICE_NAME "FlowIO_Serial"
#endif // DEVICE_NAME
#pragma message DEVICE_NAME

#ifndef DEBUG
#define DEBUG false
#endif // DEBUG

#ifndef SHOULD_ACK
#define SHOULD_ACK false
#endif // SHOULD_ACK

const uint8_t SENSOR_S0_PIN = 19;
const uint8_t SENSOR_S1_PIN = 26;
const uint8_t SENSOR_S2_PIN = 25;
const uint8_t SENSOR_S3_PIN = 24;
const uint8_t SENSOR_ENABLE_PIN = 17;
const uint8_t SENSOR_OUT_PIN = A4;

using etl::optional;
using etl::variant;

FlowIO flow_io;

constexpr size_t COMMAND_BUFFER_LENGTH = 128;
char command_buffer[COMMAND_BUFFER_LENGTH];

struct Instruction {
    char command;
    uint8_t pwm;
    uint8_t ports;
};

struct SensorReady {
} sensor_ready;

struct SensorReadingRequest {
} sensor_reading_request;

struct SensorWaiting {
    uint8_t sensor_n;
    int16_t previous_readings[4];
};

struct SensorReadDone {
    int16_t values[4];
};

typedef variant<SensorReady, SensorReadingRequest, SensorWaiting, SensorReadDone> SensorState;
SensorState sensor_state{SensorReady{}};

enum SensorSampling {
    Active,
    Inactive
} sensor_sampling_mode;

Instruction STOP_ALL{'!', 0, 0b00011111};

Adafruit_ADS1015 ads;

void setup() {
    Serial.begin(230400);
    flow_io = FlowIO(INFLATION_SERIES);
    if (!ads.begin()) {
        Serial.println("Failed to initialise ADS.");
        flow_io.pixel(1, 0, 0);
    }
}

/*
 * Commands:
 * "name?" -> Print the device name
 * "state?" -> Print device status (current command)
 * "!all" -> Stop all
 * "!"|"+"|"a"|"-"|"^",<n>,<p> -> Stop, inflate, actuate, vacuum or release with n=pwm (0..255)
 *      and p=bbbbb where b=0|1 is port state, from left to right ports 1,2,3,4,5
 */

optional<Instruction> next_instruction{etl::nullopt};

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
    unsigned long ports;
    parse_binary(ports_str, &ports);
    *ports_out = (uint8_t) ports;
    return true;
}

bool parse_stop_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    DEBUG_println("stop");
    if (parse_pwm_port(s, &pwm, &ports)) {
        next_instruction = {'!', pwm, ports};
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
        next_instruction = {'+', pwm, ports};
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
        next_instruction = {'-', pwm, ports};
        return true;
    } else {
        return false;
    }
}

bool parse_release_command(const String &s) {
    uint8_t pwm;
    uint8_t ports;
    if (parse_pwm_port(s, &pwm, &ports)) {
        next_instruction = {'^', pwm, ports};
        return true;
    } else {
        return false;
    }
}

bool parse_sensor_command(const String &s) {
    sensor_state = sensor_ready;
    sensor_sampling_mode = SensorSampling::Active;
    return true;
}

bool parse_command(const String &data) {
    String command{data};
    command.trim();
    if (command.startsWith("!all")) {
        DEBUG_println("STOP ALL");
        next_instruction = STOP_ALL;
        Serial.println("ok");
        return true;
    } else if (command.startsWith("name?")) {
        Serial.println(DEVICE_NAME);
        Serial.println("ok");
        return true;
    } else if (command.startsWith("state?")) {
        Serial.println(flow_io.getHardwareState(), BIN);
        Serial.println("ok");
        return true;
    } else if (command[0] == '!') {
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
    } else if (command[0] == 'r') {
        sensor_sampling_mode = Active;
        return true;
    } else if (command[0] == 's') {
        sensor_sampling_mode = Inactive;
        return true;
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

run_every blink_led{250, []() {
    flow_io.pixel(0, 5 * green_led_state, 0);
    green_led_state = !green_led_state;
}};

auto next_buffer_space = etl::begin(command_buffer);

void push_input_to_buffer() {
    if (size_t to_read = Serial.available()) {
        auto available = etl::distance(next_buffer_space, etl::end(command_buffer));
        auto amount_read = Serial.readBytes(next_buffer_space, min(available, to_read));
        next_buffer_space = next_buffer_space + amount_read;
    }
}

optional<String> extract_command() {
    char *newline_index = etl::find(etl::begin(command_buffer), etl::end(command_buffer), '\n');
    if (newline_index != etl::end(command_buffer)) {
//        DEBUG_println("command buffer: " + String((size_t)command_buffer));
//        DEBUG_println("next buffer space: " + String((size_t)next_buffer_space));
//        DEBUG_println("newline index: " + String((size_t)newline_index));
        char bf[COMMAND_BUFFER_LENGTH];
        auto last = etl::copy(etl::begin(command_buffer), newline_index, etl::begin(bf));
        *last = '\0';
        String command(bf);
        DEBUG_println("read command: " + command);

        // Move the next command to the beginning of the buffer and override the end with null;
        etl::rotate(command_buffer, newline_index + 1, etl::end(command_buffer));
        etl::fill(next_buffer_space, etl::end(command_buffer), '\0');
        next_buffer_space -= (command.length() + 1);
        DEBUG_println("buffer after rotate" + String(command_buffer));
        return command;
    } else {
        return etl::nullopt;
    }
}

void handle_command(optional<String> &command) {
    bool succeeded = parse_command(command.value());

    if (succeeded) {
        if (SHOULD_ACK) {
            Serial.print("ok");
        }
        if (next_instruction.has_value()) {
            flow_io.pixel(0, 5 * green_led_state, 5);
            flow_io.command(next_instruction->command, next_instruction->ports, next_instruction->pwm);
            flow_io.pixel(0, 5 * green_led_state, 0);
            if (SHOULD_ACK) {
                Serial.print(", ");
                Serial.print(flow_io.getHardwareState(), BIN);
            }
        }
        if (SHOULD_ACK) {
            Serial.println();
        }
    } else {
        Serial.println("error: Failed to parse");
    }

    next_instruction = etl::nullopt;
    command = etl::nullopt;
}

void start_sample_sensor(uint16_t channel) {
    switch (channel) {
        case 0:
            ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
            return;
        case 1:
            ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, false);
            return;
        case 2:
            ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, false);
            return;
        case 3:
            ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_3, false);
            return;
        default:
            return;
    }
}

struct SensorReadingStates : public SensorState::reader {
    void read(typename etl::parameter_type<SensorReady>::type value) override {}

    void read(typename etl::parameter_type<SensorReadingRequest>::type value) override {
        if (sensor_sampling_mode == Active) {
            start_sample_sensor(0);
            sensor_state = SensorWaiting{0, {-1, -1, -1, -1}};
        }
    }

    void read(typename etl::parameter_type<SensorWaiting>::type waiting) override {
        if (ads.conversionComplete()) {
            int16_t results = ads.getLastConversionResults();
            int16_t new_readings[4] = {waiting.previous_readings[0], waiting.previous_readings[1],
                                       waiting.previous_readings[2], waiting.previous_readings[3]};
            new_readings[waiting.sensor_n] = results;
            if (waiting.sensor_n == 3) {
                sensor_state = SensorReadDone{{new_readings[0], new_readings[1], new_readings[2], new_readings[3]}};
            } else {
                start_sample_sensor(waiting.sensor_n + 1);
                sensor_state = SensorWaiting{static_cast<uint8_t>(waiting.sensor_n + 1),
                                             {new_readings[0], new_readings[1], new_readings[2], new_readings[3]}};
            }
        }
    }

    void read(typename etl::parameter_type<SensorReadDone>::type results) override {
        Serial.print("Sensors: ");
        Serial.print(results.values[0]);
        Serial.print(", ");
        Serial.print(results.values[1]);
        Serial.print(", ");
        Serial.print(results.values[2]);
        Serial.print(", ");
        Serial.print(results.values[3]);
        Serial.println();
    }
} sensor_reading_states;

void continuous_sensor_sampling() {
    sensor_state.call(sensor_reading_states);
}

run_every sensor_sample{100l, []() {
    if (sensor_sampling_mode == Active
        && (sensor_state.is_type<SensorReady>() || sensor_state.is_type<SensorReadDone>())) {
        sensor_state = SensorReadingRequest {};
    }
}};

void loop() {
    uint32_t time = millis();
    blink_led.update(time);
    push_input_to_buffer();
    optional<String> command = extract_command();
    if (command.has_value()) {
        handle_command(command);
    }
    sensor_sample.update(time);
    continuous_sensor_sampling();
    flow_io.optimizePower(150, 200);
}

