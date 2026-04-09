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

#include "formation_motor_pwm_sender.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

namespace
{
// 将电机 1~4 的有效性分别编码到 bit0~bit3。
constexpr uint8_t MOTOR_RPM_VALID_BIT(unsigned index)
{
	return static_cast<uint8_t>(1u << index);
}
}

FormationMotorPwmSender::FormationMotorPwmSender(uavcan::INode &node) :
	_publisher(node),
	_timer(node)
{
	_publisher.setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	_param_pwm_channel_h = param_find("UAVCAN_MPWM_CH");
}

int FormationMotorPwmSender::init()
{
	update_parameters();

	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &FormationMotorPwmSender::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromUSec(1000000 / MAX_RATE_HZ));
	}

	return PX4_OK;
}

void FormationMotorPwmSender::update_parameters()
{
	if (_param_pwm_channel_h != PARAM_INVALID) {
		param_get(_param_pwm_channel_h, &_pwm_channel);
	}
}

void FormationMotorPwmSender::periodic_update(const uavcan::TimerEvent &)
{
	// 参数更新后重新读取要转发的 PWM 通道号。
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		update_parameters();
	}

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	if (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || status.in_transition_mode) {
		return;
	}

	dronecan::formation::MotorPwmStatus msg;
	// 只要本次消息被正常构造，就先打上总有效标志；
	// 电机 RPM / PWM 是否分别有效再由其他 flag 和 mask 表示。
	msg.flags = dronecan::formation::MotorPwmStatus::FLAG_VALID;

	esc_status_s esc_status{};
// 从本地 `esc_status` 主题获取电机 RPM 遥测数据
	if (_esc_status_sub.copy(&esc_status) && hrt_elapsed_time(&esc_status.timestamp) <= TOPIC_TIMEOUT_US) {
		msg.flags |= dronecan::formation::MotorPwmStatus::FLAG_ESC_TELEMETRY;

		for (unsigned esc_idx = 0; esc_idx < math::min<unsigned>(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX); ++esc_idx) {
			const esc_report_s &report = esc_status.esc[esc_idx];
			// 只取出前四个电机的 RPM
			if ((report.actuator_function >= esc_report_s::ACTUATOR_FUNCTION_MOTOR1) &&
			    (report.actuator_function <= esc_report_s::ACTUATOR_FUNCTION_MOTOR4)) {
				const unsigned motor_index = report.actuator_function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
				// Motor1 -> msg.motor_rpm[0], Motor2 -> msg.motor_rpm[1] ...
				msg.motor_rpm[motor_index] = report.esc_rpm;
				msg.motor_valid_mask |= MOTOR_RPM_VALID_BIT(motor_index);
			}
		}

		// 某些驱动不会填写 actuator_function ，此时退回到 esc[0..3] 的顺序拷贝。
		if (msg.motor_valid_mask == 0) {
			for (unsigned i = 0; i < math::min<unsigned>(esc_status.esc_count, 4); ++i) {
				// 同样地，这里也是把 esc_status 中的电机 RPM 直接写入发送消息 msg.motor_rpm[]。
				msg.motor_rpm[i] = esc_status.esc[i].esc_rpm;
				msg.motor_valid_mask |= MOTOR_RPM_VALID_BIT(i);
			}
		}
	}
// 从本地 `actuator_outputs` 主题获取 PWM 输出值数据(舵机锁定/分离信号)
	actuator_outputs_s actuator_outputs{};
	const int32_t pwm_channel = math::constrain<int32_t>(_pwm_channel, 1, actuator_outputs_s::NUM_ACTUATOR_OUTPUTS);
	msg.pwm_channel = static_cast<uint8_t>(pwm_channel);

	if (_actuator_outputs_sub.copy(&actuator_outputs) &&
	    hrt_elapsed_time(&actuator_outputs.timestamp) <= TOPIC_TIMEOUT_US) {
		const unsigned pwm_index = static_cast<unsigned>(pwm_channel - 1);

		if (pwm_index < actuator_outputs.noutputs) {
			// 获取对应通道的pwm值，通道号由_pwm_channel参数设置，在地面站中修改
			msg.pwm_us = math::max(0, static_cast<int>(actuator_outputs.output[pwm_index]));
			msg.flags |= dronecan::formation::MotorPwmStatus::FLAG_PWM_VALID;
		}
	}

	// 如果 RPM 和 PWM 都没拿到有效数据，就不发空包。
	if ((msg.motor_valid_mask == 0) &&
	    ((msg.flags & dronecan::formation::MotorPwmStatus::FLAG_PWM_VALID) == 0)) {
		return;
	}

	// 广播
	(void)_publisher.broadcast(msg);
}
