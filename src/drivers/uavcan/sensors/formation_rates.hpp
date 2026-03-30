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
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <parameters/param.h>
#include <matrix/math.hpp>

/**
 * @brief 编队控制输入 UAVCAN 接收器
 *
 * 接收主机发送的 throttle/yaw/roll/pitch/flags 自定义 DroneCAN 消息，
 * 按本机 FORM_POSITION 在从机本地完成编队控制解算，并直接发布到
 * vehicle_rates_setpoint 和 offboard_control_mode。
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

	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_status_s _vehicle_status{};

	hrt_abstime _last_command_time{0};
	hrt_abstime _last_debug_print_time{0};

	param_t _param_follower_enable_h;
	param_t _param_formation_position_h;
	param_t _param_roll_to_pitch_gain_h;
	param_t _param_yaw_throttle_gain_h;
	param_t _param_roll_rel_offset_h;
	param_t _param_pitch_rel_offset_h;
	param_t _param_yaw_rel_offset_h;
	param_t _param_roll_kp_h;
	param_t _param_roll_kd_h;
	param_t _param_pitch_kp_h;
	param_t _param_pitch_kd_h;
	param_t _param_yaw_kp_h;
	param_t _param_yaw_kd_h;
	param_t _param_roll_rate_max_h;
	param_t _param_pitch_rate_max_h;
	param_t _param_yaw_rate_max_h;
	param_t _param_pitch_sync_h;
	param_t _param_pitch_comp_gain_h;
	param_t _param_yaw_sync_h;
	param_t _param_yaw_comp_gain_h;
	param_t _param_roll_comp_gain_h;

	int32_t _follower_enable{0};
	int32_t _formation_position{0};
	float _roll_to_pitch_gain{2.0f};
	float _yaw_throttle_gain{0.05f};
	float _roll_rel_offset{0.0f};
	float _pitch_rel_offset{0.0f};
	float _yaw_rel_offset{0.0f};
	float _roll_kp{2.0f};
	float _roll_kd{0.3f};
	float _pitch_kp{2.0f};
	float _pitch_kd{0.3f};
	float _yaw_kp{2.0f};
	float _yaw_kd{0.3f};
	float _roll_rate_max{1.22f};
	float _pitch_rate_max{1.05f};
	float _yaw_rate_max{0.87f};
	float _pitch_sync{1.0f};
	float _pitch_comp_gain{1.0f};
	float _yaw_sync{1.0f};
	float _yaw_comp_gain{0.3f};
	float _roll_comp_gain{1.0f};
};
