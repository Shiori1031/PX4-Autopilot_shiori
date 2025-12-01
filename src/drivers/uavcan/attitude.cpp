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

#include "attitude.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_global_position.h>

UavcanAttitudePublisher::UavcanAttitudePublisher(uavcan::INode &node) :
	_publisher(node),
	_timer(node)
{
	_publisher.setPriority(uavcan::TransferPriority::Default);
}

int UavcanAttitudePublisher::init()
{
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanAttitudePublisher::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void UavcanAttitudePublisher::periodic_update(const uavcan::TimerEvent &)
{
	vehicle_attitude_s attitude{};

	if (!_attitude_sub.update(&attitude)) {
		/* 没有新的姿态数据可用 */
		if (!_attitude_sub.copy(&attitude)) {
			return;
		}
	}

	vehicle_angular_velocity_s angular_velocity{};
	(void)_angular_velocity_sub.copy(&angular_velocity);

	vehicle_acceleration_s acceleration{};
	(void)_acceleration_sub.copy(&acceleration);

	/*
	Read global position data (GPS)
	vehicle_global_position uORB 话题：融合后的全局位置估计（经纬度）
	*/ 
	vehicle_global_position_s global_pos{};
	bool has_position = _global_position_sub.copy(&global_pos);

	uavcan::equipment::ahrs::Solution msg{};

	const uint64_t timestamp_sample = (attitude.timestamp_sample != 0) ? attitude.timestamp_sample : hrt_absolute_time();
	msg.timestamp.usec = timestamp_sample;

	/* PX4 四元数顺序是 w, x, y, z -> 转换为 UAVCAN 的 x, y, z, w */
	msg.orientation_xyzw[0] = attitude.q[1];  // x
	msg.orientation_xyzw[1] = attitude.q[2];  // y
	msg.orientation_xyzw[2] = attitude.q[3];  // z
	msg.orientation_xyzw[3] = attitude.q[0];  // w

	msg.angular_velocity[0] = angular_velocity.xyz[0];
	msg.angular_velocity[1] = angular_velocity.xyz[1];
	msg.angular_velocity[2] = angular_velocity.xyz[2];

	msg.linear_acceleration[0] = acceleration.xyz[0];
	msg.linear_acceleration[1] = acceleration.xyz[1];
	msg.linear_acceleration[2] = acceleration.xyz[2];

	// 将 GPS 位置数据嵌入到 orientation_covariance 字段（9个元素）
	// 这是一个权宜之计，因为 UAVCAN Solution 消息本身没有位置字段
	// 格式：[纬度, 经度, 海拔, 预留, 预留, 预留, 水平精度, 垂直精度, 预留]
	if (has_position) {
		msg.orientation_covariance.resize(9);
		
		// 获取 GPS 经纬度
		double lat_deg = global_pos.lat;  // 纬度（度）
		double lon_deg = global_pos.lon;  // 经度（度）
		
		// 将纬度和经度直接存储为单精度浮点（对大多数应用来说精度损失可接受）
		// 如需更高精度，可以拆分为高位/低位部分
		msg.orientation_covariance[0] = static_cast<float>(lat_deg);  // 纬度
		msg.orientation_covariance[1] = static_cast<float>(lon_deg);  // 经度
		msg.orientation_covariance[2] = global_pos.alt;  // 海拔高度 AMSL（米）
		
		// 速度数据（预留给未来使用）
		msg.orientation_covariance[3] = 0.0f;  // 预留：可用于 vx
		msg.orientation_covariance[4] = 0.0f;  // 预留：可用于 vy
		msg.orientation_covariance[5] = 0.0f;  // 预留：可用于 vz
		
		// 精度信息
		msg.orientation_covariance[6] = global_pos.eph;  // 水平位置精度（米）
		msg.orientation_covariance[7] = global_pos.epv;  // 垂直位置精度（米）
		msg.orientation_covariance[8] = 0.0f;  // 预留
	}
	//UAVCAN 广播
	(void)_publisher.broadcast(msg);
}

