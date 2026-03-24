/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "formation_rates_sender.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

FormationRatesSender::FormationRatesSender(uavcan::INode &node) :
	_publisher(node),
	_timer(node)
{
	_publisher.setPriority(uavcan::TransferPriority::OneLowerThanHighest);
}

int FormationRatesSender::init()
{
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &FormationRatesSender::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(1000000 / MAX_RATE_HZ));
	}

	return 0;
}

void FormationRatesSender::periodic_update(const uavcan::TimerEvent &)
{
	// 获取最新的遥控输入, 范围均为[-1, 1]
	manual_control_setpoint_s manual{};
	_manual_sub.copy(&manual);

	if (hrt_elapsed_time(&manual.timestamp) > 500000 || !manual.valid) {
		return;
	}

	if (!PX4_ISFINITE(manual.throttle) || !PX4_ISFINITE(manual.yaw) || !PX4_ISFINITE(manual.roll) || !PX4_ISFINITE(manual.pitch)) {
		return;
	}

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	if (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !status.in_transition_mode) {
		return;
	}
	// 将manual输入封装到自定义的DroneCAN消息: dronecan::formation::ControlInput 中
	dronecan::formation::ControlInput msg;
	msg.throttle = math::constrain(manual.throttle, -1.0f, 1.0f);
	msg.yaw = math::constrain(manual.yaw, -1.0f, 1.0f);
	msg.roll = math::constrain(manual.roll, -1.0f, 1.0f);
	msg.pitch = math::constrain(manual.pitch, -1.0f, 1.0f);
	msg.flags = dronecan::formation::ControlInput::FLAG_VALID;

	if (manual.sticks_moving) {
		msg.flags |= dronecan::formation::ControlInput::FLAG_STICKS_MOVING;
	}

	(void)_publisher.broadcast(msg);
}
