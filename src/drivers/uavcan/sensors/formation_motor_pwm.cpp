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

#include "formation_motor_pwm.hpp"

#include <drivers/drv_hrt.h>

const char *const FormationMotorPwmBridge::NAME = "formation_motor_pwm";

FormationMotorPwmBridge::FormationMotorPwmBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_formation_motor_pwm", ORB_ID(formation_motor_pwm), nullptr, 8),
	_sub_motor_pwm(node)
{ }

int FormationMotorPwmBridge::init()
{
	int res = _sub_motor_pwm.start(MotorPwmCbBinder(this, &FormationMotorPwmBridge::motor_pwm_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start motor pwm sub: %d", res);
		return res;
	}

	return PX4_OK;
}
// 接收主机的 msg: <dronecan::formation::MotorPwmStatus> &msg
void FormationMotorPwmBridge::motor_pwm_sub_cb(const uavcan::ReceivedDataStructure<dronecan::formation::MotorPwmStatus> &msg)
{
	formation_motor_pwm_s report{};
	// timestamp 使用本机接收时间，source_node_id 记录消息来自哪个 CAN 节点。
	report.timestamp = hrt_absolute_time();
	report.source_node_id = static_cast<uint8_t>(msg.getSrcNodeID().get());

	// 这些字段与自定义 DroneCAN 消息一一对应，便于本地模块直接消费。
	report.pwm_channel = msg.pwm_channel;
	report.motor_valid_mask = msg.motor_valid_mask;
	report.flags = msg.flags;
	report.pwm_us = msg.pwm_us;

	// 固定拷贝四个电机 RPM，具体哪些值有效由 motor_valid_mask 指示。
	for (unsigned i = 0; i < 4; ++i) {
		// 这里就是把 CAN 消息里的四个电机 RPM，落到本地 uORB 结构体 report.motor_rpm[] 里。
		report.motor_rpm[i] = msg.motor_rpm[i];
	}

	// 发布为本地 uORB 多实例主题，实例号由源节点 ID 自动区分。
	publish(msg.getSrcNodeID().get(), &report);
}

int FormationMotorPwmBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}

/*
 * 其他 PX4 模块如果想使用这里接收到的电机 RPM + PWM 数据，
 * 不需要再 include DroneCAN 的消息头，也不需要直接碰
 * uavcan::ReceivedDataStructure<dronecan::formation::MotorPwmStatus>。
 *
 * 这个桥接模块已经把 CAN 消息转成了本地 uORB 话题 `formation_motor_pwm`，
 * 所以其他模块只需要像普通 uORB 一样订阅即可。
 * listener formation_motor_pwm -n 10
 *
 * 最少需要 include:
 *   #include <uORB/Subscription.hpp>
 *   #include <uORB/topics/formation_motor_pwm.h>
 *
 * 典型用法示例:
 *
 *   uORB::Subscription _formation_motor_pwm_sub{ORB_ID(formation_motor_pwm)};
 *
 *   void MyModule::Run()
 *   {
 *       formation_motor_pwm_s motor_pwm{};
 *
 *       if (_formation_motor_pwm_sub.update(&motor_pwm)) {
 *           // 四个电机转速
 *           const int32_t rpm1 = motor_pwm.motor_rpm[0];
 *           const int32_t rpm2 = motor_pwm.motor_rpm[1];
 *           const int32_t rpm3 = motor_pwm.motor_rpm[2];
 *           const int32_t rpm4 = motor_pwm.motor_rpm[3];
 *
 *           // 附带发送过来的 PWM 通道号和该通道的输出值(us)
 *           const uint8_t pwm_channel = motor_pwm.pwm_channel;
 *           const uint16_t pwm_us = motor_pwm.pwm_us;
 *
 *           // bit0~bit3 分别表示电机1~4是否有效
 *           const uint8_t valid_mask = motor_pwm.motor_valid_mask;
 *
 *           // 如果需要区分是哪一个 UAVCAN 节点发来的，可以看 source_node_id
 *           const uint8_t src_node_id = motor_pwm.source_node_id;
 *       }
 *   }
 *
 * 注意:
 * 1. 这里只是订阅本地 uORB，不是直接接收 CAN 回调。
 * 2. 如果总线上只有一个发送节点，一般直接订阅 `formation_motor_pwm` 即可。
 * 3. `motor_valid_mask` 用来判断某个电机 RPM 是否真的有效，不建议只看数值本身。
 */
