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
#include <dronecan/formation/ControlInput.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <parameters/param.h>
#include <matrix/math.hpp>

/**
 * @brief 编队控制输入 UAVCAN 接收器
 *
 * 接收主机广播的期望控制姿态（姿态设定值、偏航速率、铰链前馈量与推力设定），
 * 结合从机自身当前航向在本地合成从机姿态设定，并发布到
 * vehicle_attitude_setpoint 和 offboard_control_mode。
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
		void (FormationRatesBridge::*)(const uavcan::ReceivedDataStructure<dronecan::formation::ControlInput> &)>
		FormationRatesCbBinder;

	void formation_rates_sub_cb(const uavcan::ReceivedDataStructure<dronecan::formation::ControlInput> &msg);

	int init_driver(uavcan_bridge::Channel *channel) override;

	uavcan::Subscriber<dronecan::formation::ControlInput, FormationRatesCbBinder> _sub_formation_rates;

	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_status_s _vehicle_status{};

	hrt_abstime _last_command_time{0};

	param_t _param_follower_enable_h;
	param_t _param_formation_position_h;
	param_t _param_hinge_gain_h;
	param_t _param_roll_angle_max_h;
	param_t _param_pitch_angle_max_h;

	int32_t _follower_enable{0};
	int32_t _formation_position{0};
	float _hinge_gain{1.0f};
	float _roll_angle_max{0.52f};
	float _pitch_angle_max{0.35f};
};
