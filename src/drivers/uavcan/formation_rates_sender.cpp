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
#include <matrix/math.hpp>

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
	// 读取主机期望控制姿态，超过 500 ms 未更新则停止发送
	vehicle_attitude_setpoint_s att_sp{};
	_vehicle_attitude_setpoint_sub.copy(&att_sp);

	if (hrt_elapsed_time(&att_sp.timestamp) > 500000) {
		return;
	}

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	if (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !status.in_transition_mode) {
		return;
	}

	vehicle_rates_setpoint_s rates_sp{};
	_vehicle_rates_setpoint_sub.copy(&rates_sp);

	// 反解主机姿态设定，得到滚转/俯仰设定角
	matrix::Quatf q_sp(att_sp.q_d);
	matrix::Eulerf euler_sp(q_sp);

	// 将主机期望控制姿态（姿态设定、滚转速率前馈、推力设定与偏航指令）封装到消息中
	dronecan::formation::ControlInput msg;
	msg.thrust = math::constrain(att_sp.thrust_body[0], 0.0f, 1.0f);
	msg.pitch = euler_sp.theta();
	msg.roll_target = euler_sp.phi();
	msg.roll_rate_target = rates_sp.roll;
	msg.flags = dronecan::formation::ControlInput::FLAG_VALID;

	// 偏航指令源：手动姿态操纵模式使用速率设定（操纵杆偏航已折算在内），
	// 自主/半自主模式使用导航解算的偏航速率指令
	const bool host_in_manual_attitude =
		(status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) ||
		(status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB) ||
		(status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) ||
		(status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL) ||
		(status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL);

	if (host_in_manual_attitude) {
		// yaw: 协调转弯 + 遥控器 的角速率
		// yaw_sp_move_rate: 固定翼自身链路无生产者，恒 0
		// 纯手动/自稳：直接使用 rates_sp.yaw（含操纵杆偏航输入）
		msg.yaw = PX4_ISFINITE(rates_sp.yaw) ? rates_sp.yaw : 0.0f;

	} else if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
		// 自主/半自主模式：使用导航系统设定的偏航速率
		msg.yaw = att_sp.yaw_sp_move_rate;

	} else {
		// 最终回退：优先速率设定
		msg.yaw = PX4_ISFINITE(rates_sp.yaw) ? rates_sp.yaw : 0.0f;
	}

	(void)_publisher.broadcast(msg);
}
