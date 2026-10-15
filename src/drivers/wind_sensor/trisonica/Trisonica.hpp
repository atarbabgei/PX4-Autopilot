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

/**
 * @file Trisonica.hpp
 *
 * Serial driver for the LI-COR LI-550 TriSonica ultrasonic anemometer.
 * Publishes horizontal wind speed and direction on sensor_airflow.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_airflow.h>

using namespace time_literals;

class Trisonica : public ModuleBase<Trisonica>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Trisonica(const char *port);
	~Trisonica() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;

private:
	enum class State : uint8_t {
		EnterCli,       // Ctrl+C sent, waiting for the CLI prompt
		EnterCliRetry,  // CR sent (CLI was already open), waiting for the prompt
		Command,        // command sent, waiting for the prompt
		Streaming,      // CLI exited, parsing measurement records
		Backoff,        // configuration failed, waiting before retrying
	};

	enum class Phase : uint8_t {
		Version,
		Prepare,
		Fields,
		Rate,
		Display,
		Done,
	};

	// Instrument field from the "display" table.
	struct Field {
		char name[12];
		char tag[8];
		char units[12];
		bool enabled;
		bool tagged;
	};

	// Measurement channel used for sensor_airflow.
	struct Channel {
		const char *name;   // instrument field name
		const char *tag;    // tag applied by the streaming profile
		char active_tag[8]; // tag reported by the instrument
		float scale;        // conversion to SI (m/s or rad)
		bool available;
	};

	enum ChannelIndex { CH_S2D, CH_D, CH_U, CH_V, CH_COUNT };

	static constexpr hrt_abstime PROMPT_TIMEOUT{2_s};
	static constexpr hrt_abstime STALE_TIMEOUT{3_s};
	static constexpr hrt_abstime BACKOFF_TIME{2_s};
	static constexpr size_t MAX_FIELDS{32};
	static constexpr size_t LINE_LENGTH{256};

	void Run() override;

	void startConfiguration();
	void sendNextCommand();
	void configurationFailed(const char *reason);
	void startListenOnly();
	bool resolveChannels();

	void readSerial();
	void handleLine(char *line);
	void handleReplyLine(const char *line);
	void parseDisplayRow(const char *line);
	void parseMeasurement(char *line);

	void write(const char *text);
	static bool isWantedField(const char *name, const char **tag);

	device::Serial _serial;
	char _port[20] {};

	State _state{State::EnterCli};
	Phase _phase{Phase::Version};
	uint8_t _prepare_index{0};
	uint8_t _field_index{0};
	uint8_t _field_step{0};
	bool _identity_ok{false};
	bool _awaiting_version{false};
	hrt_abstime _state_time{0};
	hrt_abstime _last_valid_sample{0};

	// Scratch buffers kept off the small serial work queue stack.
	char _line[LINE_LENGTH] {};
	char _row[LINE_LENGTH] {};
	char _command[48] {};
	uint8_t _rx_buffer[64] {};
	size_t _line_len{0};
	bool _line_overflow{false};

	Field _fields[MAX_FIELDS] {};
	uint8_t _num_fields{0};

	Channel _channels[CH_COUNT] {
		{"S2D", "S2", {}, 1.f, false},
		{"D",   "D",  {}, 1.f, false},
		{"U",   "U",  {}, 1.f, false},
		{"V",   "V",  {}, 1.f, false},
	};

	char _last_error[64] {"none"};
	uint8_t _rx_tail[32] {};  // last received bytes, for diagnostics
	uint8_t _rx_tail_pos{0};
	uint32_t _rx_bytes{0};
	uint32_t _rx_bytes_at_start{0};
	bool _listen_only{false};
	sensor_airflow_s _last_report{};

	uORB::Publication<sensor_airflow_s> _sensor_airflow_pub{ORB_ID(sensor_airflow)};

	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": samples")};
	perf_counter_t _parse_errors{perf_alloc(PC_COUNT, MODULE_NAME": parse errors")};
	perf_counter_t _invalid_samples{perf_alloc(PC_COUNT, MODULE_NAME": invalid samples")};
	perf_counter_t _config_count{perf_alloc(PC_COUNT, MODULE_NAME": configurations")};
	perf_counter_t _config_failures{perf_alloc(PC_COUNT, MODULE_NAME": configuration failures")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_TRSNC_RATE>) _param_rate,
		(ParamBool<px4::params::SENS_TRSNC_SETUP>) _param_setup
	)
};
