/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Trisonica.hpp"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <drivers/device/device.h>
#include <drivers/drv_sensor.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/getopt.h>

namespace
{

constexpr uint32_t BAUDRATE = 115200;

// Commands sent before reading the field table when applying the streaming profile.
constexpr const char *PREPARE_COMMANDS[] = {
	"units si",
	"expert enable",
	"paramdelim space",
	"tagdelim space",
	"orientuv std",
	"display",
};
constexpr uint8_t NUM_PREPARE_COMMANDS = sizeof(PREPARE_COMMANDS) / sizeof(PREPARE_COMMANDS[0]);

// Fields kept in the streaming profile and their tags. Everything else is hidden.
struct WantedField {
	const char *name;
	const char *tag;
};

constexpr WantedField WANTED_FIELDS[] = {
	{"S2D", "S2"},
	{"D", "D"},
	{"U", "U"},
	{"V", "V"},
	{"W", "W"},
	{"T", "T"},
	{"Status", "St"},
};

bool contains_no_case(const char *haystack, const char *needle)
{
	const size_t len = strlen(needle);

	for (; *haystack != '\0'; haystack++) {
		if (strncasecmp(haystack, needle, len) == 0) {
			return true;
		}
	}

	return false;
}

char *trim(char *text)
{
	while (isspace((unsigned char)*text)) { text++; }

	size_t len = strlen(text);

	while (len > 0 && isspace((unsigned char)text[len - 1])) { text[--len] = '\0'; }

	return text;
}

void copy(char *dst, size_t size, const char *src)
{
	strncpy(dst, src, size - 1);
	dst[size - 1] = '\0';
}

// Velocity conversion to m/s, 0 if the unit is not recognized.
float velocity_scale(const char *units)
{
	if (!strcmp(units, "m/s") || !strcmp(units, "mps")) { return 1.f; }

	if (!strcmp(units, "kph") || !strcmp(units, "km/h")) { return 1.f / 3.6f; }

	if (!strcmp(units, "mph")) { return 0.44704f; }

	if (!strcmp(units, "fps")) { return 0.3048f; }

	if (!strcmp(units, "kts")) { return 1852.f / 3600.f; }

	return 0.f;
}

} // namespace

Trisonica::Trisonica(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	copy(_port, sizeof(_port), port);

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.devtype = DRV_WIND_DEVTYPE_TRISONICA;

	// Bus number from the serial port, e.g. /dev/ttyS1 -> 1
	const size_t len = strlen(_port);

	if (len > 0 && isdigit((unsigned char)_port[len - 1])) {
		device_id.devid_s.bus = _port[len - 1] - '0';
	}

	_last_report.device_id = device_id.devid;
}

Trisonica::~Trisonica()
{
	ScheduleClear();
	_serial.close();

	perf_free(_sample_perf);
	perf_free(_parse_errors);
	perf_free(_invalid_samples);
	perf_free(_config_count);
	perf_free(_config_failures);
}

int Trisonica::init()
{
	if (!_serial.setPort(_port) || !_serial.setBaudrate(BAUDRATE)) {
		PX4_ERR("invalid port %s", _port);
		return PX4_ERROR;
	}

	_sensor_airflow_pub.advertise();

	// The port is opened in Run(): on NuttX file descriptors belong to the opening task,
	// so it must be opened on the work queue that uses it.
	// 115200 baud is ~58 bytes per 5 ms, well within the UART RX buffer.
	ScheduleOnInterval(5_ms);

	return PX4_OK;
}

void Trisonica::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_serial.close();
		exit_and_cleanup();
		return;
	}

	if (!_serial.isOpen()) {
		if (_state == State::Backoff && hrt_elapsed_time(&_state_time) < BACKOFF_TIME) {
			return;
		}

		if (!_serial.open()) {
			configurationFailed("failed to open port");
			return;
		}

		startConfiguration();
	}

	readSerial();

	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case State::EnterCli:
		if (now - _state_time > PROMPT_TIMEOUT) {
			// In an already open CLI, Ctrl+C is literal input; CR submits it and restores a prompt.
			write("\r");
			_state = State::EnterCliRetry;
			_state_time = now;
		}

		break;

	case State::EnterCliRetry:
		if (now - _state_time > PROMPT_TIMEOUT) {
			if (_rx_bytes != _rx_bytes_at_start) {
				// The instrument streams but does not answer (e.g. FC TX not connected): parse with standard tags.
				startListenOnly();

			} else {
				configurationFailed("no data received (check sensor TX -> FC RX)");
			}
		}

		break;

	case State::Command:
		if (now - _state_time > PROMPT_TIMEOUT) {
			configurationFailed("command timeout");
		}

		break;

	case State::Streaming:
		if (now - _last_valid_sample > STALE_TIMEOUT) {
			PX4_WARN("no measurements for %.1f s, reconfiguring", (double)(STALE_TIMEOUT * 1e-6));
			startConfiguration();
		}

		break;

	case State::Backoff:
		if (now - _state_time > BACKOFF_TIME) {
			startConfiguration();
		}

		break;
	}
}

void Trisonica::startConfiguration()
{
	perf_count(_config_count);
	_rx_bytes_at_start = _rx_bytes;
	_listen_only = false;

	_phase = Phase::Version;
	_prepare_index = 0;
	_field_index = 0;
	_field_step = 0;
	_identity_ok = false;
	_line_len = 0;
	_line_overflow = false;

	// Ctrl+C interrupts the measurement stream and opens the CLI.
	write("\x03");
	_state = State::EnterCli;
	_state_time = hrt_absolute_time();
}

void Trisonica::configurationFailed(const char *reason)
{
	perf_count(_config_failures);

	if (strcmp(_last_error, reason) != 0) {
		PX4_ERR("configuration failed: %s", reason);
		copy(_last_error, sizeof(_last_error), reason);
	}

	// Always try to resume acquisition, the next attempt re-enters the CLI.
	write("exit\r");
	_state = State::Backoff;
	_state_time = hrt_absolute_time();
}

void Trisonica::startListenOnly()
{
	for (auto &channel : _channels) {
		copy(channel.active_tag, sizeof(channel.active_tag), channel.tag);
		channel.scale = (&channel == &_channels[CH_D]) ? math::radians(1.f) : 1.f;
		channel.available = true;
	}

	_listen_only = true;
	_state = State::Streaming;
	_last_valid_sample = hrt_absolute_time();

	PX4_WARN("no CLI response, listen-only with standard tags (check FC TX -> sensor RX)");
	copy(_last_error, sizeof(_last_error), "no CLI response, listen-only");
}

bool Trisonica::isWantedField(const char *name, const char **tag)
{
	for (const auto &field : WANTED_FIELDS) {
		if (!strcmp(name, field.name)) {
			*tag = field.tag;
			return true;
		}
	}

	return false;
}

void Trisonica::sendNextCommand()
{
	char *command = _command;
	const size_t command_size = sizeof(_command);
	_awaiting_version = false;

	while (true) {
		switch (_phase) {
		case Phase::Version:
			_phase = _param_setup.get() ? Phase::Prepare : Phase::Display;
			_awaiting_version = true;
			write("version\r");
			_state = State::Command;
			_state_time = hrt_absolute_time();
			return;

		case Phase::Prepare:
			if (_prepare_index < NUM_PREPARE_COMMANDS) {
				const char *cmd = PREPARE_COMMANDS[_prepare_index++];

				if (!strcmp(cmd, "display")) {
					_num_fields = 0;
				}

				snprintf(command, command_size, "%s\r", cmd);
				break;
			}

			if (_num_fields == 0) {
				configurationFailed("unrecognized display table");
				return;
			}

			_phase = Phase::Fields;
			continue;

		case Phase::Fields:
			if (_field_index < _num_fields) {
				const Field &field = _fields[_field_index];
				const char *tag = nullptr;

				if (!isWantedField(field.name, &tag)) {
					snprintf(command, command_size, "hide %s\r", field.name);
					_field_index++;
					break;
				}

				switch (_field_step) {
				case 0: snprintf(command, command_size, "tagid %s %s\r", field.name, tag); break;

				case 1: snprintf(command, command_size, "tag %s\r", field.name); break;

				default: snprintf(command, command_size, "show %s\r", field.name); break;
				}

				if (++_field_step == 3) {
					_field_step = 0;
					_field_index++;
				}

				break;
			}

			_phase = Phase::Rate;
			continue;

		case Phase::Rate:
			snprintf(command, command_size, "outputrate %d\r", (int)_param_rate.get());
			_phase = Phase::Display;
			break;

		case Phase::Display:
			_num_fields = 0;
			snprintf(command, command_size, "display\r");
			_phase = Phase::Done;
			break;

		case Phase::Done:
			if (!resolveChannels()) {
				return;
			}

			write("exit\r");
			_state = State::Streaming;
			_last_valid_sample = hrt_absolute_time();
			copy(_last_error, sizeof(_last_error), "none");

			PX4_INFO("configured on %s @ %" PRIu32 " baud%s, streaming", _port, BAUDRATE,
				 _param_setup.get() ? " (profile applied)" : "");
			return;
		}

		write(command);
		_state = State::Command;
		_state_time = hrt_absolute_time();
		return;
	}
}

bool Trisonica::resolveChannels()
{
	for (auto &channel : _channels) {
		channel.available = false;
		channel.active_tag[0] = '\0';

		for (uint8_t i = 0; i < _num_fields; i++) {
			const Field &field = _fields[i];

			if (strcmp(field.name, channel.name) != 0 || !field.enabled || !field.tagged || field.tag[0] == '\0') {
				continue;
			}

			if (&channel == &_channels[CH_D]) {
				if (strcasecmp(field.units, "degrees") && strcmp(field.units, "deg")) {
					PX4_WARN("unsupported direction units: %s", field.units);
					break;
				}

				channel.scale = math::radians(1.f);

			} else {
				channel.scale = velocity_scale(field.units);

				if (channel.scale <= 0.f) {
					PX4_WARN("unsupported %s units: %s", field.name, field.units);
					break;
				}
			}

			copy(channel.active_tag, sizeof(channel.active_tag), field.tag);
			channel.available = true;
			break;
		}
	}

	const bool has_speed = _channels[CH_S2D].available || (_channels[CH_U].available && _channels[CH_V].available);

	if (!has_speed || !_channels[CH_D].available) {
		configurationFailed("S2D (or U and V) and D must be enabled and tagged");
		return false;
	}

	return true;
}

void Trisonica::write(const char *text)
{
	// LI-550 CLI consumes CR only, a following LF becomes literal input.
	if (_serial.isOpen()) {
		_serial.write(text, strlen(text));
	}
}

void Trisonica::readSerial()
{
	uint8_t *buffer = _rx_buffer;
	ssize_t count;

	while ((count = _serial.read(_rx_buffer, sizeof(_rx_buffer))) > 0) {
		_rx_bytes += count;

		for (ssize_t i = 0; i < count; i++) {
			const char c = buffer[i];
			_rx_tail[_rx_tail_pos++ % sizeof(_rx_tail)] = c;

			if (c == '\n') {
				_line[_line_len] = '\0';

				if (!_line_overflow) {
					handleLine(_line);
				}

				_line_len = 0;
				_line_overflow = false;

			} else if (!_line_overflow) {
				if (_line_len < LINE_LENGTH - 1) {
					_line[_line_len++] = c;

				} else {
					perf_count(_parse_errors);
					_line_overflow = true;
				}
			}
		}

		if (_state == State::EnterCli || _state == State::EnterCliRetry || _state == State::Command) {
			// The CLI prompt '>' is not followed by a newline. Don't modify the partial line,
			// the rest of it may arrive with the next read.
			size_t len = _line_len;

			while (len > 0 && isspace((unsigned char)_line[len - 1])) { len--; }

			if (len > 0 && _line[len - 1] == '>') {
				_line_len = 0;

				if (_state == State::Command && _awaiting_version && !_identity_ok) {
					// Reply to "version" did not identify the instrument.
					configurationFailed("instrument is not an LI-550 TriSonica");
					return;
				}

				sendNextCommand();

				if (_state != State::Command) {
					return;
				}
			}
		}
	}
}

void Trisonica::handleLine(char *line)
{
	line = trim(line);

	if (line[0] == '\0') {
		return;
	}

	switch (_state) {
	case State::Command:
		handleReplyLine(line);
		break;

	case State::Streaming:
		// The CLI echoes exit before the first record.
		if (strcmp(line, "exit") != 0 && strcmp(line, ">") != 0) {
			parseMeasurement(line);
		}

		break;

	default:
		break;
	}
}

void Trisonica::handleReplyLine(const char *line)
{
	if (contains_no_case(line, "unknown command") || contains_no_case(line, "invalid")
	    || contains_no_case(line, "error") || contains_no_case(line, "unrecognized")) {
		snprintf(_command, sizeof(_command), "rejected: %s", line);
		configurationFailed(_command);
		return;
	}

	if (contains_no_case(line, "LI-550") && contains_no_case(line, "TriSonica")) {
		_identity_ok = true;
	}

	if (strchr(line, '|') != nullptr) {
		parseDisplayRow(line);
	}
}

void Trisonica::parseDisplayRow(const char *line)
{
	// | Name | Description | Tagged | Tag | Decimals | Enabled | Units |
	char *row = _row;
	copy(row, sizeof(_row), line);

	char *cells[8] {};
	int num_cells = 0;
	char *cell = row;

	while (num_cells < 8) {
		char *next = strchr(cell, '|');

		if (next) { *next = '\0'; }

		cells[num_cells++] = trim(cell);

		if (!next) { break; }

		cell = next + 1;
	}

	if (num_cells < 8 || cells[1][0] == '\0' || !strcmp(cells[1], "Name") || !isalpha((unsigned char)cells[1][0])) {
		return;
	}

	if (_num_fields >= MAX_FIELDS) {
		return;
	}

	Field &field = _fields[_num_fields++];
	copy(field.name, sizeof(field.name), cells[1]);
	copy(field.tag, sizeof(field.tag), cells[4]);
	copy(field.units, sizeof(field.units), cells[7]);
	field.tagged = !strcmp(cells[3], "Yes");
	field.enabled = !strcmp(cells[6], "Yes");
}

void Trisonica::parseMeasurement(char *line)
{
	const hrt_abstime timestamp = hrt_absolute_time();

	float values[CH_COUNT];
	bool present[CH_COUNT] {};
	bool valid[CH_COUNT] {};

	char *save = nullptr;
	char *tag = strtok_r(line, " \t", &save);

	while (tag != nullptr) {
		char *token = strtok_r(nullptr, " \t", &save);

		if (token == nullptr) {
			perf_count(_parse_errors);
			return;
		}

		for (int i = 0; i < CH_COUNT; i++) {
			if (_channels[i].available && !strcmp(tag, _channels[i].active_tag)) {
				char *end = nullptr;
				const float value = strtof(token, &end);

				if (end == token || *end != '\0') {
					perf_count(_parse_errors);
					return;
				}

				present[i] = true;
				// The instrument reports -99.x for failed measurements.
				valid[i] = PX4_ISFINITE(value) && !(value <= -99.f && value > -100.f);
				values[i] = value * _channels[i].scale;
				break;
			}
		}

		tag = strtok_r(nullptr, " \t", &save);
	}

	if (!present[CH_S2D] && !present[CH_D] && !present[CH_U] && !present[CH_V]) {
		perf_count(_parse_errors);
		return;
	}

	_last_valid_sample = timestamp;

	sensor_airflow_s report{};
	report.timestamp = timestamp;
	report.device_id = _last_report.device_id;
	report.speed = NAN;
	report.direction = NAN;

	// The instrument sets D to zero when a horizontal component fails.
	const bool horizontal_failed = (present[CH_U] && !valid[CH_U]) || (present[CH_V] && !valid[CH_V]);

	if (!horizontal_failed) {
		if (valid[CH_S2D]) {
			report.speed = values[CH_S2D];

		} else if (valid[CH_U] && valid[CH_V]) {
			report.speed = sqrtf(values[CH_U] * values[CH_U] + values[CH_V] * values[CH_V]);
		}

		if (valid[CH_D]) {
			report.direction = matrix::wrap_pi(values[CH_D]);
		}
	}

	report.status = (PX4_ISFINITE(report.speed) && PX4_ISFINITE(report.direction)) ? 0 : 1;

	if (report.status != 0) {
		perf_count(_invalid_samples);
	}

	perf_count(_sample_perf);
	_sensor_airflow_pub.publish(report);
	_last_report = report;
}

int Trisonica::print_status()
{
	static constexpr const char *STATE_NAMES[] = {"entering CLI", "entering CLI (retry)", "configuring", "streaming", "backoff"};

	PX4_INFO("port: %s @ %" PRIu32 " baud", _port, BAUDRATE);
	PX4_INFO("state: %s%s, profile on startup: %s, rate: %" PRId32 " Hz", STATE_NAMES[(int)_state],
		 _listen_only ? " (listen-only)" : "",
		 _param_setup.get() ? "yes" : "no", _param_rate.get());
	PX4_INFO("last error: %s", _last_error);

	char tail[sizeof(_rx_tail) * 4 + 1] {};
	size_t pos = 0;

	for (size_t i = 0; i < sizeof(_rx_tail); i++) {
		const uint8_t c = _rx_tail[(_rx_tail_pos + i) % sizeof(_rx_tail)];
		pos += snprintf(tail + pos, sizeof(tail) - pos, isprint(c) ? "%c" : "\\x%02x", c);
	}

	PX4_INFO("rx bytes: %" PRIu32 ", last: %s", _rx_bytes, tail);

	for (const auto &channel : _channels) {
		PX4_INFO("field %-3s: %s%s", channel.name, channel.available ? "tag " : "unavailable",
			 channel.available ? channel.active_tag : "");
	}

	if (_last_report.timestamp != 0) {
		PX4_INFO("last sample: speed %.2f m/s, direction %.1f deg, status %d, age %.2f s",
			 (double)_last_report.speed, (double)math::degrees(_last_report.direction), _last_report.status,
			 (double)(hrt_elapsed_time(&_last_report.timestamp) * 1e-6));
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_invalid_samples);
	perf_print_counter(_parse_errors);
	perf_print_counter(_config_count);
	perf_print_counter(_config_failures);
	return 0;
}

int Trisonica::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *port = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		default:
			print_usage("unrecognized flag");
			return PX4_ERROR;
		}
	}

	if (port == nullptr || strlen(port) >= sizeof(Trisonica::_port)) {
		print_usage("valid serial device required");
		return PX4_ERROR;
	}

	Trisonica *instance = new Trisonica(port);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int Trisonica::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Trisonica::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for the LI-COR LI-550 TriSonica ultrasonic anemometer (115200 baud, 8N1).

On every (re)connection the driver opens the instrument CLI, verifies its identity and, if SENS_TRSNC_SETUP
is set, applies a streaming profile (SI units, space delimiters, orientuv std, tagged U/V/W/S2D/D/T, rate
SENS_TRSNC_RATE). Nothing is saved to the instrument flash.

Publishes sensor_airflow: speed is the horizontal wind speed (S2D) in m/s and direction is the instrument
wind direction (D) in radians, wrapped to [-pi, pi], relative to the sensor north mark.
status is 0 for a valid sample and 1 if the horizontal wind measurement failed (speed and direction NAN).

Most boards start the driver on the UART selected by SENS_TRSNC_CFG.

### Examples

$ trisonica start -d /dev/ttyS1
$ trisonica status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trisonica", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int trisonica_main(int argc, char *argv[])
{
	return Trisonica::main(argc, argv);
}
