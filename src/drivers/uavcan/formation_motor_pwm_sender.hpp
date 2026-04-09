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

#pragma once

#include <uavcan/uavcan.hpp>
#include <dronecan/formation/MotorPwmStatus.hpp>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

/**
 * @brief 四旋翼电机转速 + PWM 附加通道发送器
 *
 * 周期读取本机 `esc_status` 中前四个电机的真实 RPM，以及
 * `actuator_outputs` 中一个指定通道的实际 PWM 输出值，
 * 再封装为自定义 DroneCAN 消息 `MotorPwmStatus` 广播出去。
 */
class FormationMotorPwmSender
{
public:
	FormationMotorPwmSender(uavcan::INode &node);

	int init();

private:
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr hrt_abstime TOPIC_TIMEOUT_US = 200000;

	/// 定时执行一次采样 + 打包 + 广播
	void periodic_update(const uavcan::TimerEvent &);
	/// 读取运行时参数，例如要附带发送的 PWM 通道号
	void update_parameters();

	typedef uavcan::MethodBinder<FormationMotorPwmSender *, void (FormationMotorPwmSender::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	uavcan::Publisher<dronecan::formation::MotorPwmStatus> _publisher;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	/// 发送端数据来源:
	/// - esc_status: 电机真实转速遥测
	/// - actuator_outputs: PWM 输出实际值
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	param_t _param_pwm_channel_h{PARAM_INVALID};
	/// 1-based PWM 通道编号，默认附带发送第 5 路输出
	int32_t _pwm_channel{5};
};
