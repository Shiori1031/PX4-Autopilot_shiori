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

#include <lib/mathlib/mathlib.h>

const char *const FormationRatesBridge::NAME = "formation_rates";

namespace
{
constexpr int32_t FORMATION_POSITION_LEFT = 1;
constexpr int32_t FORMATION_POSITION_RIGHT = 2;
}

FormationRatesBridge::FormationRatesBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_formation_rates", ORB_ID(vehicle_rates_setpoint), node_info_publisher, 8),
	_sub_formation_rates(node)
{
	// 查找参数句柄
	_param_follower_enable_h = param_find("FORM_FOLLOWER_EN");
	_param_formation_position_h = param_find("FORM_POSITION");
	_param_roll_to_pitch_gain_h = param_find("FORM_R2P_GAIN");
	_param_yaw_throttle_gain_h = param_find("FORM_YAW_K");
	_param_pitch_sync_h = param_find("FORM_PITCH_SYNC");
	_param_pitch_comp_gain_h = param_find("FORM_PITCH_COMP");
	_param_yaw_sync_h = param_find("FORM_YAW_SYNC");
	_param_yaw_comp_gain_h = param_find("FORM_YAW_COMP");
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

	if (_param_roll_to_pitch_gain_h != PARAM_INVALID) {
		param_get(_param_roll_to_pitch_gain_h, &_roll_to_pitch_gain);
	}

	if (_param_yaw_throttle_gain_h != PARAM_INVALID) {
		param_get(_param_yaw_throttle_gain_h, &_yaw_throttle_gain);
	}

	if (_param_pitch_sync_h != PARAM_INVALID) {
		param_get(_param_pitch_sync_h, &_pitch_sync);
	}

	if (_param_pitch_comp_gain_h != PARAM_INVALID) {
		param_get(_param_pitch_comp_gain_h, &_pitch_comp_gain);
	}

	if (_param_yaw_sync_h != PARAM_INVALID) {
		param_get(_param_yaw_sync_h, &_yaw_sync);
	}

	if (_param_yaw_comp_gain_h != PARAM_INVALID) {
		param_get(_param_yaw_comp_gain_h, &_yaw_comp_gain);
	}

	if (_param_roll_comp_gain_h != PARAM_INVALID) {
		param_get(_param_roll_comp_gain_h, &_roll_comp_gain);
	}
	// 启动 UAVCAN 订阅
	// 启动 UAVCAN 订阅
	int res = _sub_formation_rates.start(FormationRatesCbBinder(this, &FormationRatesBridge::formation_rates_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start formation rates sub: %d", res);
		return res;
	}

	return PX4_OK;
}

void FormationRatesBridge::formation_rates_sub_cb(const uavcan::ReceivedDataStructure<dronecan::formation::ControlInput> &msg)
{
	// 重新加载参数（支持运行时更新）
	if (_param_follower_enable_h != PARAM_INVALID) {
		param_get(_param_follower_enable_h, &_follower_enable);
	}

	if (_param_formation_position_h != PARAM_INVALID) {
		param_get(_param_formation_position_h, &_formation_position);
	}

	if (_param_roll_to_pitch_gain_h != PARAM_INVALID) {
		param_get(_param_roll_to_pitch_gain_h, &_roll_to_pitch_gain);
	}

	if (_param_yaw_throttle_gain_h != PARAM_INVALID) {
		param_get(_param_yaw_throttle_gain_h, &_yaw_throttle_gain);
	}

	if (_param_pitch_sync_h != PARAM_INVALID) {
		param_get(_param_pitch_sync_h, &_pitch_sync);
	}

	if (_param_pitch_comp_gain_h != PARAM_INVALID) {
		param_get(_param_pitch_comp_gain_h, &_pitch_comp_gain);
	}

	if (_param_yaw_sync_h != PARAM_INVALID) {
		param_get(_param_yaw_sync_h, &_yaw_sync);
	}

	if (_param_yaw_comp_gain_h != PARAM_INVALID) {
		param_get(_param_yaw_comp_gain_h, &_yaw_comp_gain);
	}

	if (_param_roll_comp_gain_h != PARAM_INVALID) {
		param_get(_param_roll_comp_gain_h, &_roll_comp_gain);
	}

	// 检查是否启用从机模式
	if (_follower_enable == 0) {
		return;
	}

	if ((msg.flags & dronecan::formation::ControlInput::FLAG_VALID) == 0) {
		return;
	}

	if (_formation_position != FORMATION_POSITION_LEFT && _formation_position != FORMATION_POSITION_RIGHT) {
		return;
	}

	// 获取从机自身姿态与角速度，用于自稳补偿
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_vehicle_attitude);
	}

	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);

	// 取出从机自身 roll / pitch / yaw_rate 进行补偿
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	const float self_roll = euler.phi();
	const float self_pitch = euler.theta();
	const float self_yaw_rate = angular_velocity.xyz[2];
	const float side_sign = (_formation_position == FORMATION_POSITION_LEFT) ? 1.0f : -1.0f;
	const float base_thrust = math::constrain((static_cast<float>(msg.throttle) + 1.0f) * 0.5f, 0.0f, 1.0f);
	const float yaw_for_boost = (_formation_position == FORMATION_POSITION_LEFT) ?
		math::max(static_cast<float>(msg.yaw), 0.0f) : math::max(-static_cast<float>(msg.yaw), 0.0f);

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

// 主要控制逻辑
	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp = _last_command_time;

	// 滚转：系数 x (遥控输入 - 自身滚转)
	rates_sp.roll = _roll_comp_gain * (static_cast<float>(msg.roll) - self_roll);

	// 俯仰：左右机系数(±) x 滚转映射 + 俯仰同步 - 自身 pitch 补偿
	rates_sp.pitch = side_sign * static_cast<float>(msg.roll) * _roll_to_pitch_gain + static_cast<float>(msg.pitch) * _pitch_sync - self_pitch * _pitch_comp_gain;

	 // 偏航：主机偏航同步系数 x 遥控输入 - 自身 yaw_rate 阻尼补偿
	rates_sp.yaw = static_cast<float>(msg.yaw) * _yaw_sync - self_yaw_rate * _yaw_comp_gain;

	// 推力：基础油门 + 外侧机增量
	rates_sp.thrust_body[0] = math::constrain(base_thrust + yaw_for_boost * _yaw_throttle_gain, 0.0f, 1.0f);
	rates_sp.thrust_body[1] = 0.0f;
	rates_sp.thrust_body[2] = 0.0f;
	rates_sp.reset_integral = (msg.flags & dronecan::formation::ControlInput::FLAG_RESET_INTEGRAL) != 0;
	_vehicle_rates_setpoint_pub.publish(rates_sp);
}

int FormationRatesBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}
