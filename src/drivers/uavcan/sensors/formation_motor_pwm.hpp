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

#include "sensor_bridge.hpp"

#include <dronecan/formation/MotorPwmStatus.hpp>
#include <uORB/topics/formation_motor_pwm.h>

/**
 * @brief 四旋翼电机转速 + PWM 接收桥
 *
 * 订阅自定义 DroneCAN 消息 `MotorPwmStatus`，
 * 将收到的四个电机 RPM 和一个 PWM 输出值解析后，
 * 发布成本地 uORB 主题 `formation_motor_pwm`，供 PX4 其他模块读取。
 */
class FormationMotorPwmBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	FormationMotorPwmBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	typedef uavcan::MethodBinder<FormationMotorPwmBridge *,
		void (FormationMotorPwmBridge::*)(const uavcan::ReceivedDataStructure<dronecan::formation::MotorPwmStatus> &)>
		MotorPwmCbBinder;

	/// 将 CAN 消息字段逐项拷贝到本地 uORB `formation_motor_pwm`
	void motor_pwm_sub_cb(const uavcan::ReceivedDataStructure<dronecan::formation::MotorPwmStatus> &msg);
	int init_driver(uavcan_bridge::Channel *channel) override;

	uavcan::Subscriber<dronecan::formation::MotorPwmStatus, MotorPwmCbBinder> _sub_motor_pwm;
};
