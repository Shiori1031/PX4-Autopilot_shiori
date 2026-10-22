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

	// 偏航指令源：固定翼链路上唯一的偏航速率指令是 vehicle_rates_setpoint.yaw
	//   STAB / ALTCTL / POSCTL : 协调转弯 + 偏航杆量      (fw_att_control 生成)
	//   MISSION 等自主模式      : 协调转弯 tan(roll)*cos(pitch)*g/V (fw_att_control 生成)
	//   ACRO                    : 杆量偏航速率            (fw_rate_control 生成)
	// 注意: att_sp.yaw_sp_move_rate 在固定翼链路上没有任何生产者
	//       (FwLateralLongitudinalControl 只写 q_d 与 thrust_body[0])，恒为 0，
	//       不能作为自主模式的偏航指令源，否则从机在 MISSION 下收到的偏航指令恒为 0。
	const bool rates_sp_valid = PX4_ISFINITE(rates_sp.yaw)
				    && (hrt_elapsed_time(&rates_sp.timestamp) < 500000);
	msg.yaw = rates_sp_valid ? rates_sp.yaw : 0.0f;

	(void)_publisher.broadcast(msg);
}
