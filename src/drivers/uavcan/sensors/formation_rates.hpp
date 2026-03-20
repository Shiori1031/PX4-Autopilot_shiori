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

#include <drivers/drv_hrt.h>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <parameters/param.h>
#include <matrix/math.hpp>

/**
 * @brief 编队速率指令 UAVCAN 接收器（直接控制模式）
 *
 * 通过 UAVCAN ArrayCommand 接收主机发送的编队速率指令，
 * 解码角速率和推力值，并直接发布到
 * vehicle_rates_setpoint 和 offboard_control_mode 主题。
 *
 * 无需手动启动模块 —— 当 FORM_FOLLOWER_EN=1 时自动激活。
 *
 * 解码格式：
 *   actuator_id 100 = 滚转速率 (rad/s)
 *   actuator_id 101 = 俯仰速率 (rad/s)
 *   actuator_id 102 = 偏航速率 (rad/s)
 *   actuator_id 103 = 前向推力
 *   actuator_id 104 = 横向推力
 *   actuator_id 105 = 垂向推力
 *   actuator_id 110 = 编队位置（必须与 FORM_POSITION 匹配）
 */
class FormationRatesBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	FormationRatesBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	typedef uavcan::MethodBinder<FormationRatesBridge *,
		void (FormationRatesBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand> &)>
		FormationRatesCbBinder;

	void formation_rates_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand> &msg);

	int init_driver(uavcan_bridge::Channel *channel) override;

	uavcan::Subscriber<uavcan::equipment::actuator::ArrayCommand, FormationRatesCbBinder> _sub_formation_rates;

	// 直接控制发布器
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	vehicle_attitude_s _vehicle_attitude{};

	hrt_abstime _last_command_time{0};

	// 参数句柄
	param_t _param_follower_enable_h;
	param_t _param_formation_position_h;
	param_t _param_roll_comp_gain_h;

	// 参数缓存值
	int32_t _follower_enable{0};
	int32_t _formation_position{0};
	float _roll_comp_gain{1.0f};
};
