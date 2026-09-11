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
#include <px4_platform_common/log.h>

const char *const FormationRatesBridge::NAME = "formation_rates";

namespace
{
constexpr int32_t FORMATION_POSITION_LEFT = 1;
constexpr int32_t FORMATION_POSITION_RIGHT = 2;
}

FormationRatesBridge::FormationRatesBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_formation_rates", ORB_ID(vehicle_attitude_setpoint), node_info_publisher, 8),
	_sub_formation_rates(node)
{
	// 查找参数句柄
	_param_follower_enable_h = param_find("FORM_FOLLOWER_EN");
	_param_formation_position_h = param_find("FORM_POSITION");
	_param_hinge_gain_h = param_find("FORM_HINGE_K");
	_param_roll_angle_max_h = param_find("FORM_ROLL_AMAX");
	_param_pitch_angle_max_h = param_find("FORM_PTCH_AMAX");
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

	if (_param_hinge_gain_h != PARAM_INVALID) {
		param_get(_param_hinge_gain_h, &_hinge_gain);
	}

	if (_param_roll_angle_max_h != PARAM_INVALID) {
		param_get(_param_roll_angle_max_h, &_roll_angle_max);
	}

	if (_param_pitch_angle_max_h != PARAM_INVALID) {
		param_get(_param_pitch_angle_max_h, &_pitch_angle_max);
	}

	// 启动 UAVCAN 订阅
	int res = _sub_formation_rates.start(FormationRatesCbBinder(this, &FormationRatesBridge::formation_rates_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start formation rates sub: %d", res);
		return res;
	}

	return PX4_OK;
}
// 接收主机的 msg: <dronecan::formation::ControlInput> &msg
void FormationRatesBridge::formation_rates_sub_cb(const uavcan::ReceivedDataStructure<dronecan::formation::ControlInput> &msg)
{
	/// 仅在参数发生更新时重新加载（避免每次回调都调用 param_get）
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		if (_param_follower_enable_h != PARAM_INVALID) {
			param_get(_param_follower_enable_h, &_follower_enable);
		}

		if (_param_formation_position_h != PARAM_INVALID) {
			param_get(_param_formation_position_h, &_formation_position);
		}

		if (_param_hinge_gain_h != PARAM_INVALID) {
			param_get(_param_hinge_gain_h, &_hinge_gain);
		}

		if (_param_roll_angle_max_h != PARAM_INVALID) {
			param_get(_param_roll_angle_max_h, &_roll_angle_max);
		}

		if (_param_pitch_angle_max_h != PARAM_INVALID) {
			param_get(_param_pitch_angle_max_h, &_pitch_angle_max);
		}
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

	// 读取从机自身姿态（用于保持自身航向）与飞行模式（Offboard 接管门控）
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_vehicle_attitude);
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	// 取出从机自身航向，姿态设定中航向分量保持当前航向（不做航向位置指令）
	matrix::Quatf q_self(_vehicle_attitude.q);
	matrix::Eulerf euler_self(q_self);
	const float self_yaw = euler_self.psi();

	_last_command_time = hrt_absolute_time();

	// 发布 offboard_control_mode 以维持 Offboard 模式
	offboard_control_mode_s offboard_mode{};
	offboard_mode.timestamp = _last_command_time;
	offboard_mode.position = false;
	offboard_mode.velocity = false;
	offboard_mode.acceleration = false;
	offboard_mode.attitude = true;
	offboard_mode.body_rate = false;
	_offboard_control_mode_pub.publish(offboard_mode);

	// 如果当前不是 Offboard 直接退出这次回调
	if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		return;
	}

// 从机姿态设定：跟随主机期望控制姿态 + 铰链运动学前馈补偿
	const float side_sign = (_formation_position == FORMATION_POSITION_LEFT) ? 1.0f : -1.0f;

	const float roll_sp = math::constrain(static_cast<float>(msg.roll_target), -_roll_angle_max, _roll_angle_max);
	const float pitch_sp = math::constrain(static_cast<float>(msg.pitch)
					     + side_sign * _hinge_gain * static_cast<float>(msg.roll_rate_target),
					     -_pitch_angle_max, _pitch_angle_max);
	const float yaw_sp = matrix::wrap_pi(self_yaw);

	vehicle_attitude_setpoint_s att_sp{};
	att_sp.timestamp = _last_command_time;

	// 欧拉角转四元数
	matrix::Quatf q_d(matrix::Eulerf(roll_sp, pitch_sp, yaw_sp));
	q_d.copyTo(att_sp.q_d);
	att_sp.yaw_sp_move_rate = static_cast<float>(msg.yaw);

	// 推力直接透传主机推力设定（偏航增推已移至 ControlAllocator 处理）
	att_sp.thrust_body[0] = math::constrain(static_cast<float>(msg.thrust), 0.0f, 1.0f);
	att_sp.thrust_body[1] = 0.0f;
	att_sp.thrust_body[2] = 0.0f;
	_vehicle_attitude_setpoint_pub.publish(att_sp);
}

int FormationRatesBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}
