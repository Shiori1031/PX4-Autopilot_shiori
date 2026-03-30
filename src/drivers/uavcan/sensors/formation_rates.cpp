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
	UavcanSensorBridgeBase("uavcan_formation_rates", ORB_ID(vehicle_rates_setpoint), node_info_publisher, 8),
	_sub_formation_rates(node)
{
	// 查找参数句柄
	_param_follower_enable_h = param_find("FORM_FOLLOWER_EN");
	_param_formation_position_h = param_find("FORM_POSITION");
	_param_roll_to_pitch_gain_h = param_find("FORM_R2P_GAIN");
	_param_yaw_throttle_gain_h = param_find("FORM_YAW_K");
	_param_roll_rel_offset_h = param_find("FORM_ROLL_OFS");
	_param_pitch_rel_offset_h = param_find("FORM_PITCH_OFS");
	_param_yaw_rel_offset_h = param_find("FORM_YAW_OFS");
	_param_roll_kp_h = param_find("FORM_ROLL_KP");
	_param_roll_kd_h = param_find("FORM_ROLL_KD");
	_param_pitch_kp_h = param_find("FORM_PITCH_KP");
	_param_pitch_kd_h = param_find("FORM_PITCH_KD");
	_param_yaw_kp_h = param_find("FORM_YAW_KP");
	_param_yaw_kd_h = param_find("FORM_YAW_KD");
	_param_roll_rate_max_h = param_find("FORM_ROLL_RMAX");
	_param_pitch_rate_max_h = param_find("FORM_PTCH_RMAX");
	_param_yaw_rate_max_h = param_find("FORM_YAW_RMAX");
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

	if (_param_roll_rel_offset_h != PARAM_INVALID) {
		param_get(_param_roll_rel_offset_h, &_roll_rel_offset);
	}

	if (_param_pitch_rel_offset_h != PARAM_INVALID) {
		param_get(_param_pitch_rel_offset_h, &_pitch_rel_offset);
	}

	if (_param_yaw_rel_offset_h != PARAM_INVALID) {
		param_get(_param_yaw_rel_offset_h, &_yaw_rel_offset);
	}

	if (_param_roll_kp_h != PARAM_INVALID) {
		param_get(_param_roll_kp_h, &_roll_kp);
	}

	if (_param_roll_kd_h != PARAM_INVALID) {
		param_get(_param_roll_kd_h, &_roll_kd);
	}

	if (_param_pitch_kp_h != PARAM_INVALID) {
		param_get(_param_pitch_kp_h, &_pitch_kp);
	}

	if (_param_pitch_kd_h != PARAM_INVALID) {
		param_get(_param_pitch_kd_h, &_pitch_kd);
	}

	if (_param_yaw_kp_h != PARAM_INVALID) {
		param_get(_param_yaw_kp_h, &_yaw_kp);
	}

	if (_param_yaw_kd_h != PARAM_INVALID) {
		param_get(_param_yaw_kd_h, &_yaw_kd);
	}

	if (_param_roll_rate_max_h != PARAM_INVALID) {
		param_get(_param_roll_rate_max_h, &_roll_rate_max);
	}

	if (_param_pitch_rate_max_h != PARAM_INVALID) {
		param_get(_param_pitch_rate_max_h, &_pitch_rate_max);
	}

	if (_param_yaw_rate_max_h != PARAM_INVALID) {
		param_get(_param_yaw_rate_max_h, &_yaw_rate_max);
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

		if (_param_roll_to_pitch_gain_h != PARAM_INVALID) {
			param_get(_param_roll_to_pitch_gain_h, &_roll_to_pitch_gain);
		}

		if (_param_yaw_throttle_gain_h != PARAM_INVALID) {
			param_get(_param_yaw_throttle_gain_h, &_yaw_throttle_gain);
		}

		if (_param_roll_rel_offset_h != PARAM_INVALID) {
			param_get(_param_roll_rel_offset_h, &_roll_rel_offset);
		}

		if (_param_pitch_rel_offset_h != PARAM_INVALID) {
			param_get(_param_pitch_rel_offset_h, &_pitch_rel_offset);
		}

		if (_param_yaw_rel_offset_h != PARAM_INVALID) {
			param_get(_param_yaw_rel_offset_h, &_yaw_rel_offset);
		}

		if (_param_roll_kp_h != PARAM_INVALID) {
			param_get(_param_roll_kp_h, &_roll_kp);
		}

		if (_param_roll_kd_h != PARAM_INVALID) {
			param_get(_param_roll_kd_h, &_roll_kd);
		}

		if (_param_pitch_kp_h != PARAM_INVALID) {
			param_get(_param_pitch_kp_h, &_pitch_kp);
		}

		if (_param_pitch_kd_h != PARAM_INVALID) {
			param_get(_param_pitch_kd_h, &_pitch_kd);
		}

		if (_param_yaw_kp_h != PARAM_INVALID) {
			param_get(_param_yaw_kp_h, &_yaw_kp);
		}

		if (_param_yaw_kd_h != PARAM_INVALID) {
			param_get(_param_yaw_kd_h, &_yaw_kd);
		}

		if (_param_roll_rate_max_h != PARAM_INVALID) {
			param_get(_param_roll_rate_max_h, &_roll_rate_max);
		}

		if (_param_pitch_rate_max_h != PARAM_INVALID) {
			param_get(_param_pitch_rate_max_h, &_pitch_rate_max);
		}

		if (_param_yaw_rate_max_h != PARAM_INVALID) {
			param_get(_param_yaw_rate_max_h, &_yaw_rate_max);
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

	// 获取从机自身姿态、角速度与飞行模式，用于相对姿态控制与接管门控
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_vehicle_attitude);
	}

	vehicle_angular_velocity_s angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&angular_velocity);

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	// 取出从机自身 roll / pitch / yaw 以及 p / q / r
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	const float self_roll = euler.phi();
	const float self_pitch = euler.theta();
	const float self_yaw = euler.psi();
	const float self_roll_rate = angular_velocity.xyz[0];
	const float self_pitch_rate = angular_velocity.xyz[1];
	const float self_yaw_rate = angular_velocity.xyz[2];
	// 判断左右机，计算油门及偏航增量; 左 +1, 右 -1
	const float side_sign = (_formation_position == FORMATION_POSITION_LEFT) ? 1.0f : -1.0f;
	const float base_thrust = math::constrain((static_cast<float>(msg.throttle) + 1.0f) * 0.5f, 0.0f, 1.0f);
	const float yaw_for_boost = (_formation_position == FORMATION_POSITION_LEFT) ? math::max(static_cast<float>(msg.yaw), 0.0f) : math::max(-static_cast<float>(msg.yaw), 0.0f);

	_last_command_time = hrt_absolute_time();

	// // 调试输出主机姿态角与 p/q/r，限频到 1 Hz，避免刷屏
	// if ((_last_command_time - _last_debug_print_time) > 1000000) {
	// 	_last_debug_print_time = _last_command_time;
	// 	PX4_INFO("leader att[rpy]=[%.2f %.2f %.2f] pqr=[%.2f %.2f %.2f]",
	// 		 (double)msg.att_roll, (double)msg.att_pitch, (double)msg.att_yaw,
	// 		 (double)msg.p, (double)msg.q, (double)msg.r);
	// }

	// 发布 offboard_control_mode 以维持 Offboard 模式
	offboard_control_mode_s offboard_mode{};
	offboard_mode.timestamp = _last_command_time;
	offboard_mode.position = false;
	offboard_mode.velocity = false;
	offboard_mode.acceleration = false;
	offboard_mode.attitude = false;
	offboard_mode.body_rate = true;
	_offboard_control_mode_pub.publish(offboard_mode);

	// 如果当前不是 Offboard 直接退出这次回调
	if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		return;
	}

// 从机主要控制逻辑: 相对姿态PID控制 + 遥控器控制 加权融合
	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp = _last_command_time;

	// 主机状态（使用发送端广播的实际姿态角和 p/q/r）
	const float leader_roll = static_cast<float>(msg.att_roll);
	const float leader_pitch = static_cast<float>(msg.att_pitch);
	const float leader_yaw = static_cast<float>(msg.att_yaw);
	const float leader_p = static_cast<float>(msg.p);
	const float leader_q = static_cast<float>(msg.q);
	const float leader_r = static_cast<float>(msg.r);

	// 期望相对姿态（安装偏角 / 编队偏角）
	// 初始安装角度，直接设置为0
	const float roll_rel_des = _roll_rel_offset;
	const float pitch_rel_des = _pitch_rel_offset * side_sign;
	const float yaw_rel_des = _yaw_rel_offset;

	// 相对误差
	const float e_roll = (leader_roll + roll_rel_des) - self_roll;
	const float e_pitch = (leader_pitch + pitch_rel_des) - self_pitch;
	const float e_yaw = matrix::wrap_pi((leader_yaw + yaw_rel_des) - self_yaw);

// 1. 相对姿态 + 主机 p/q/r 前馈 + 相对速率阻尼
	// 滚转：主机 p 前馈 + 相对姿态反馈(P x roll误差) + 相对速率阻尼(D x rate误差)
	const float relative_roll_sp = leader_p + _roll_kp * e_roll + _roll_kd * (leader_p - self_roll_rate);

	// 俯仰：主机 q 前馈 + 相对姿态反馈(P x pitch误差) + 相对速率阻尼(D x rate误差) + roll-pitch 耦合
	// 从机的俯仰影响整体滚转，side_sign 区分左右机进行反向俯仰
	const float relative_pitch_sp = leader_q + _pitch_kp * e_pitch + _pitch_kd * (leader_q - self_pitch_rate)
				 + side_sign * _roll_to_pitch_gain * e_roll;

	// 偏航：主机 r 前馈 + 相对姿态反馈(P x yaw误差) + 相对速率阻尼(D x rate误差)
	// 偏航方向舵都同向偏转，(rates_sp.yaw: 机体系 z 轴偏航角速度设定值)
	const float relative_yaw_sp = leader_r + _yaw_kp * e_yaw + _yaw_kd * (leader_r - self_yaw_rate);

// 2. PID 改造前的上一版遥控器直接控制逻辑
	// 滚转：系数 x (遥控输入 - 自身滚转); default MAX: ±1.22 rad/s
	const float stick_roll_sp = _roll_comp_gain * (static_cast<float>(msg.roll) - self_roll);

	// 俯仰：左右机系数(±) x 滚转映射 + 俯仰同步 - 自身 pitch 补偿; default MAX: ±1.05 rad/s
	const float stick_pitch_sp = side_sign * static_cast<float>(msg.roll) * _roll_to_pitch_gain
				 + static_cast<float>(msg.pitch) * _pitch_sync - self_pitch * _pitch_comp_gain;

	// 偏航：主机偏航同步系数 x 遥控输入 - 自身 yaw_rate 阻尼补偿; default MAX: ±0.87 rad/s
	const float stick_yaw_sp = static_cast<float>(msg.yaw) * _yaw_sync - self_yaw_rate * _yaw_comp_gain;

// 加权融合
	constexpr float relative_ctrl_weight = 0.5f;
	constexpr float stick_ctrl_weight = 0.5f;
	rates_sp.roll = relative_ctrl_weight * relative_roll_sp + stick_ctrl_weight * stick_roll_sp;
	rates_sp.pitch = relative_ctrl_weight * relative_pitch_sp + stick_ctrl_weight * stick_pitch_sp;
	rates_sp.yaw = relative_ctrl_weight * relative_yaw_sp + stick_ctrl_weight * stick_yaw_sp;

	// 限幅：参考 PX4 fixed-wing 原生默认 rate setpoint 上限
	rates_sp.roll = math::constrain(rates_sp.roll, -_roll_rate_max, _roll_rate_max);
	rates_sp.pitch = math::constrain(rates_sp.pitch, -_pitch_rate_max, _pitch_rate_max);
	rates_sp.yaw = math::constrain(rates_sp.yaw, -_yaw_rate_max, _yaw_rate_max);

	// 推力：基础油门 + 外侧机增量; default MAX: [0, 1]
	// yaw_for_boost 已经判断左右机并取正值或 0; _yaw_throttle_gain 固定增益
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
