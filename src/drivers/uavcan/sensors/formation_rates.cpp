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

#include "formation_rates.hpp"

const char *const FormationRatesBridge::NAME = "formation_rates";

FormationRatesBridge::FormationRatesBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_formation_rates", ORB_ID(vehicle_rates_setpoint), node_info_publisher, 8),
	_sub_formation_rates(node)
{
	// 查找参数句柄
	_param_follower_enable_h = param_find("FORM_FOLLOWER_EN");
	_param_formation_position_h = param_find("FORM_POSITION");
	_param_timeout_h = param_find("FORM_TIMEOUT");
	_param_roll_comp_gain_h = param_find("FORM_ROLL_COMP");
}

int FormationRatesBridge::init()
{
	// 加载参数
	if (_param_follower_enable_h != PARAM_INVALID) {
		param_get(_param_follower_enable_h, &_follower_enable);
	}

	if (_param_formation_position_h != PARAM_INVALID) {
		param_get(_param_formation_position_h, &_formation_position);
	}

	if (_param_timeout_h != PARAM_INVALID) {
		param_get(_param_timeout_h, &_timeout);
	}

	if (_param_roll_comp_gain_h != PARAM_INVALID) {
		param_get(_param_roll_comp_gain_h, &_roll_comp_gain);
	}

	int res = _sub_formation_rates.start(FormationRatesCbBinder(this, &FormationRatesBridge::formation_rates_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start formation rates sub: %d", res);
		return res;
	}

	return PX4_OK;
}
// UAVCAN 速率指令订阅回调
void FormationRatesBridge::formation_rates_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand> &msg)
{
	// 重新加载参数（支持运行时更新）
	if (_param_follower_enable_h != PARAM_INVALID) {
		param_get(_param_follower_enable_h, &_follower_enable);
	}

	if (_param_roll_comp_gain_h != PARAM_INVALID) {
		param_get(_param_roll_comp_gain_h, &_roll_comp_gain);
	}

	// 检查是否启用从机模式
	if (_follower_enable == 0) {
		return;
	}

	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_vehicle_attitude);
	}

	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	float self_roll = euler.phi();

	// 解码 ArrayCommand 消息
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	float thrust_x = 0.5f; // 默认 50% 油门
	float thrust_y = 0.0f;
	float thrust_z = 0.0f;
	uint8_t formation_position = 0;

	for (const auto &cmd : msg.commands) {
		switch (cmd.actuator_id) {
		case 100: // 滚转速率
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED) {
				roll = cmd.command_value;
			}
			break;

		case 101: // 俯仰速率（主控制通道）
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED) {
				pitch = cmd.command_value;
			}
			break;

		case 102: // 偏航速率
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED) {
				yaw = cmd.command_value;
			}
			break;

		case 103: // 前向推力
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS) {
				thrust_x = cmd.command_value;
			}
			break;

		case 104: // 横向推力
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS) {
				thrust_y = cmd.command_value;
			}
			break;

		case 105: // 垂向推力
			if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS) {
				thrust_z = cmd.command_value;
			}
			break;

		case 110: // 编队位置标识
			formation_position = static_cast<uint8_t>(cmd.command_value);
			break;

		default:
			// 忽略未知 actuator_id
			break;
		}
	}

	// 重新加载位置参数
	if (_param_formation_position_h != PARAM_INVALID) {
		param_get(_param_formation_position_h, &_formation_position);
	}

	// 验证指令是否属于本机
	if (_formation_position == 0 || formation_position != _formation_position) {
		// 不是发给本机的指令，忽略
		return;
	}

	_last_command_time = hrt_absolute_time();

	// 发布 offboard_control_mode 以维持 Offboard 模式
	offboard_control_mode_s offboard_mode{};
	offboard_mode.timestamp = _last_command_time;
	offboard_mode.position = false;
	offboard_mode.velocity = false;
	offboard_mode.acceleration = false;
	offboard_mode.attitude = false;
	offboard_mode.body_rate = true;
	_offboard_control_mode_pub.publish(offboard_mode);

	// 发布 vehicle_rates_setpoint 给速率控制器
	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp = _last_command_time;
	rates_sp.roll = _roll_comp_gain * (roll - self_roll);
	rates_sp.pitch = pitch;
	rates_sp.yaw = yaw;
	rates_sp.thrust_body[0] = thrust_x;
	rates_sp.thrust_body[1] = thrust_y;
	rates_sp.thrust_body[2] = thrust_z;
	_vehicle_rates_setpoint_pub.publish(rates_sp);
}

int FormationRatesBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}
