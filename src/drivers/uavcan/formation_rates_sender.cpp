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
#include <px4_platform_common/log.h>

FormationRatesSender::FormationRatesSender(uavcan::INode &node) :
	_publisher(node),
	_timer(node)
{
	// 编队指令使用高优先级，降低传输延迟
	_publisher.setPriority(uavcan::TransferPriority::OneLowerThanHighest);

	// 查找参数句柄
	_param_roll_to_pitch_gain_h = param_find("FORM_R2P_GAIN");
	_param_yaw_coupling_h = param_find("FORM_YAW_K");
	_param_throttle_diff_h = param_find("FORM_THR_DIFF");
	_param_pitch_sync_h = param_find("FORM_PITCH_SYNC");
	_param_left_node_id_h = param_find("FORM_LEFT_ID");
	_param_right_node_id_h = param_find("FORM_RIGHT_ID");
}

int FormationRatesSender::init()
{
	// 加载参数
	update_params();

	// 启动周期定时器（100Hz）
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &FormationRatesSender::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void FormationRatesSender::update_params()
{
	if (_param_roll_to_pitch_gain_h != PARAM_INVALID) {
		param_get(_param_roll_to_pitch_gain_h, &_roll_to_pitch_gain);
	}

	if (_param_yaw_coupling_h != PARAM_INVALID) {
		param_get(_param_yaw_coupling_h, &_yaw_coupling);
	}

	if (_param_throttle_diff_h != PARAM_INVALID) {
		param_get(_param_throttle_diff_h, &_throttle_diff);
	}

	if (_param_pitch_sync_h != PARAM_INVALID) {
		param_get(_param_pitch_sync_h, &_pitch_sync);
	}

	if (_param_left_node_id_h != PARAM_INVALID) {
		param_get(_param_left_node_id_h, &_left_node_id);
	}

	if (_param_right_node_id_h != PARAM_INVALID) {
		param_get(_param_right_node_id_h, &_right_node_id);
	}
}

void FormationRatesSender::periodic_update(const uavcan::TimerEvent &)
{
	// 获取遥控器输入（使用 copy 而不是 update，保证始终有数据）
	manual_control_setpoint_s manual{};
	_manual_sub.copy(&manual);

	// 检查数据有效性（时间戳不能太旧，500ms = 500000us）
	if (hrt_elapsed_time(&manual.timestamp) > 500000) {
		return;  // 遥控器数据超过 500ms 未更新，停止发送
	}

	// 检查是否在固定翼模式
	vehicle_status_s status{};
	_vehicle_status_sub.update(&status);

	if (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING &&
	    !status.in_transition_mode) {
		return;  // 仅在固定翼模式下工作
	}

	// ========== 计算左机指令 ==========
	uavcan::equipment::actuator::ArrayCommand left_msg;

	// 滚转速率：直接传递主机滚转指令，提高从机抗扰和一致性
	uavcan::equipment::actuator::Command left_roll;
	left_roll.actuator_id = 100;
	left_roll.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	left_roll.command_value = manual.roll;
	left_msg.commands.push_back(left_roll);

	// 俯仰速率 = 编队滚转映射 + 俯仰同步
	// 编队滚转：主机横滚 → 左机抬头（产生升力差 → 编队滚转力矩）
	// 俯仰同步：主机俯仰 → 三机一起抬头/低头（可相对俯仰所以从机需跟随）
	uavcan::equipment::actuator::Command left_pitch;
	left_pitch.actuator_id = 101;
	left_pitch.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	left_pitch.command_value = manual.roll * _roll_to_pitch_gain
				  + manual.pitch * _pitch_sync;
	left_msg.commands.push_back(left_pitch);

	// 偏航速率（协调转弯）
	uavcan::equipment::actuator::Command left_yaw;
	left_yaw.actuator_id = 102;
	left_yaw.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	left_yaw.command_value = manual.yaw + (-manual.roll * _yaw_coupling);
	left_msg.commands.push_back(left_yaw);

	// 推力 = 基础油门 + 偏航差速（仅右偏航时增加，左偏航时保持）
	uavcan::equipment::actuator::Command left_thrust;
	left_thrust.actuator_id = 103;
	left_thrust.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
	left_thrust.command_value = math::constrain(
		(manual.throttle + 1.0f) * 0.5f + math::max(manual.yaw, 0.0f) * _throttle_diff, 0.0f, 1.0f);
	left_msg.commands.push_back(left_thrust);

	// 编队位置标识
	uavcan::equipment::actuator::Command left_position;
	left_position.actuator_id = 110;
	left_position.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
	left_position.command_value = 1.0f;  // 1 = 左机
	left_msg.commands.push_back(left_position);

	// 广播左机指令
	(void)_publisher.broadcast(left_msg);

	// ========== 计算右机指令 ==========
	uavcan::equipment::actuator::ArrayCommand right_msg;

	// 滚转速率：直接传递主机滚转指令，提高从机抗扰和一致性
	uavcan::equipment::actuator::Command right_roll;
	right_roll.actuator_id = 100;
	right_roll.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	right_roll.command_value = manual.roll;
	right_msg.commands.push_back(right_roll);

	// 俯仰速率 = 编队滚转映射（反向）+ 俯仰同步
	// 编队滚转：方向与左机相反（低头减少升力 → 产生滚转力矩）
	// 俯仰同步：与左机相同（三机一起抬头/低头）
	uavcan::equipment::actuator::Command right_pitch;
	right_pitch.actuator_id = 101;
	right_pitch.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	right_pitch.command_value = -manual.roll * _roll_to_pitch_gain
				   + manual.pitch * _pitch_sync;
	right_msg.commands.push_back(right_pitch);

	// 偏航速率（协调转弯，耦合方向与左机相反）
	uavcan::equipment::actuator::Command right_yaw;
	right_yaw.actuator_id = 102;
	right_yaw.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_SPEED;
	right_yaw.command_value = manual.yaw + (manual.roll * _yaw_coupling);
	right_msg.commands.push_back(right_yaw);

	// 推力 = 基础油门 - 偏航差速（仅左偏航时增加，右偏航时保持）
	uavcan::equipment::actuator::Command right_thrust;
	right_thrust.actuator_id = 103;
	right_thrust.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
	right_thrust.command_value = math::constrain(
		(manual.throttle + 1.0f) * 0.5f - math::min(manual.yaw, 0.0f) * _throttle_diff, 0.0f, 1.0f);
	right_msg.commands.push_back(right_thrust);

	// 编队位置标识
	uavcan::equipment::actuator::Command right_position;
	right_position.actuator_id = 110;
	right_position.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
	right_position.command_value = 2.0f;  // 2 = 右机
	right_msg.commands.push_back(right_position);

	// 广播右机指令
	(void)_publisher.broadcast(right_msg);
}
