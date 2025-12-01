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

#include "ahrs_solution.hpp"

#include <cmath>

const char *const UavcanAhrsSolutionBridge::NAME = "ahrs_solution";

UavcanAhrsSolutionBridge::UavcanAhrsSolutionBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_ahrs_solution", ORB_ID(uavcan_attitude), 8),
	_sub_solution(node)
{ }

int UavcanAhrsSolutionBridge::init()
{
	int res = _sub_solution.start(SolutionCbBinder(this, &UavcanAhrsSolutionBridge::solution_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start solution sub: %d", res);
		return res;
	}

	return PX4_OK;
}

void UavcanAhrsSolutionBridge::solution_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::Solution> &msg)
{
	uavcan_attitude_s report{};
	report.timestamp = hrt_absolute_time();
	report.timestamp_sample = msg.timestamp.usec;
	report.source_node_id = static_cast<uint8_t>(msg.getSrcNodeID().get());

	// 解析姿态数据
	if (msg.orientation_xyzw.size() == 4) {
		// PX4 四元数顺序是 w, x, y, z，而 UAVCAN Solution 使用 x, y, z, w
		report.q[0] = msg.orientation_xyzw[3];  // w
		report.q[1] = msg.orientation_xyzw[0];  // x
		report.q[2] = msg.orientation_xyzw[1];  // y
		report.q[3] = msg.orientation_xyzw[2];  // z

	} else {
		for (unsigned i = 0; i < 4; i++) {
			report.q[i] = NAN;
		}
	}

	for (unsigned i = 0; i < 3; i++) {
		report.angular_velocity[i] = (msg.angular_velocity.size() == 3) ? msg.angular_velocity[i] : NAN;
		report.linear_acceleration[i] = (msg.linear_acceleration.size() == 3) ? msg.linear_acceleration[i] : NAN;
	}

	// 从 orientation_covariance 字段解析 GPS 位置数据
	// 格式：[纬度, 经度, 海拔, 预留, 预留, 预留, 水平精度, 垂直精度, 预留]
	if (msg.orientation_covariance.size() >= 9) {
		// 解码 GPS 坐标
		report.lat = static_cast<double>(msg.orientation_covariance[0]);  // 纬度（度）
		report.lon = static_cast<double>(msg.orientation_covariance[1]);  // 经度（度）
		report.alt = msg.orientation_covariance[2];  // 海拔（米）
		
		// 检查坐标是否有效（非 NAN 且在合理范围内）
		if (!std::isnan(report.lat) && !std::isnan(report.lon) && 
		    report.lat >= -90.0 && report.lat <= 90.0 &&
		    report.lon >= -180.0 && report.lon <= 180.0) {
			report.xy_valid = true;
		} else {
			report.xy_valid = false;
			report.lat = NAN;
			report.lon = NAN;
		}
		
		if (!std::isnan(report.alt)) {
			report.z_valid = true;
		} else {
			report.z_valid = false;
			report.alt = NAN;
		}
		
		// 速度数据（当前版本预留，未使用）
		report.v_xy_valid = false;
		report.v_z_valid = false;
		report.vx = NAN;
		report.vy = NAN;
		report.vz = NAN;
		report.heading = NAN;
		
		// 精度信息
		report.eph = msg.orientation_covariance[6];  // 水平精度（米）
		report.epv = msg.orientation_covariance[7];  // 垂直精度（米）
	} else {
		// 没有位置数据可用
		report.xy_valid = false;
		report.z_valid = false;
		report.v_xy_valid = false;
		report.v_z_valid = false;

		report.lat = NAN;
		report.lon = NAN;
		report.alt = NAN;
		report.vx = NAN;
		report.vy = NAN;
		report.vz = NAN;
		report.heading = NAN;
		report.eph = NAN;
		report.epv = NAN;
	}

	publish(msg.getSrcNodeID().get(), &report);
}

int UavcanAhrsSolutionBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}
