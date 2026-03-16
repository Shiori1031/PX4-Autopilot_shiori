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
#include <uavcan/equipment/actuator/ArrayCommand.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <parameters/param.h>

/**
 * @brief 编队速率发送器（集成控制解算逻辑）
 *
 * 直接订阅 manual_control_setpoint，内部计算编队速率指令，
 * 通过 UAVCAN ArrayCommand 广播到左右从机。
 *
 * 无需独立的 formation_master 模块。
 *
 * 编码格式：
 *   actuator_id 100 = 滚转速率 (rad/s)
 *   actuator_id 101 = 俯仰速率 (rad/s) - 主控制通道
 *   actuator_id 102 = 偏航速率 (rad/s)
 *   actuator_id 103 = 前向推力
 *   actuator_id 110 = 编队位置 (1=左机, 2=右机)
 */
class FormationRatesSender
{
public:
	FormationRatesSender(uavcan::INode &node);

	/**
	 * 初始化周期发布器并加载参数
	 */
	int init();

private:
	static constexpr unsigned MAX_RATE_HZ = 200;  // 200Hz 更新频率

	void periodic_update(const uavcan::TimerEvent &);
	void update_params();

	typedef uavcan::MethodBinder<FormationRatesSender *, void (FormationRatesSender::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _publisher;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	// 输入数据订阅
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// 参数句柄
	param_t _param_roll_to_pitch_gain_h;
	param_t _param_yaw_coupling_h;
	param_t _param_throttle_diff_h;
	param_t _param_pitch_sync_h;
	param_t _param_left_node_id_h;
	param_t _param_right_node_id_h;

	// 参数缓存值
	float _roll_to_pitch_gain{2.0f};		// 横滚到俯仰映射增益
	float _yaw_coupling{0.3f};		// 偏航耦合系数
	float _throttle_diff{0.05f};		// 油门差异
	float _pitch_sync{0.1f};		// 俯仰同步系数
	int32_t _left_node_id{2};		// 左机节点ID
	int32_t _right_node_id{3};		// 右机节点ID
};
