/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file LADRC1.cpp
 *
 * 一阶线性自抗扰控制器实现 (参见 LADRC1.hpp)
 */

#include "LADRC1.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

void LADRC1::setGains(float b0, float wo, float wc)
{
	_b0 = math::max(b0, 0.01f);
	_wo = math::max(wo, 0.1f);
	_wc = math::max(wc, 0.01f);

	updateGains();
}

void LADRC1::updateGains()
{
	// Gao 2003 带宽参数化 (一阶对象)
	_beta1 = 2.f * _wo;
	_beta2 = _wo * _wo;
	_kp    = _wc;
}

float LADRC1::update(float y, float r, float dt, bool landed)
{
	// 热启动: 首次调用 (或复位后) 观测器从当前测量出发, 避免初始瞬态
	if (!_initialized) {
		_z1 = y;
		_x1 = r;
		_snap = false;
		_initialized = true;
	}

	// ① TD: 指令平滑 + 导数前馈 (一阶, 常开)
	if (_snap) {
		_x1 = r;
		_snap = false;
	}

	const float r_dot = kTdLambda * (r - _x1);
	_x1 += dt * r_dot;
	const float r_smooth = _x1;

	// ② ESO (2 阶): z1→y, z2→f
	if (landed || !PX4_ISFINITE(y)) {
		// 地面/信号异常: 保持复位, 防止估计漂移
		_z1 = y;
		_z2 = 0.f;

	} else {
		const float e = _z1 - y;
		_z1 += dt * (_z2 - _beta1 * e + _b0 * _u_prev);
		_z2 += dt * (-_beta2 * e);
	}

	// ③ SEF + ④ 扰动补偿: u = (ωc·(r* − z1) + ṙ* − z2) / b0
	float u = (_kp * (r_smooth - _z1) + r_dot - _z2) / _b0;

	if (!PX4_ISFINITE(u)) {
		u = 0.f;
		_z1 = y;
		_z2 = 0.f;
	}

	_u_prev = u;   // 默认回写; 下游限幅后由 setAppliedControl() 覆写为实际施加值

	return u;
}

void LADRC1::setAppliedControl(float u_applied)
{
	if (PX4_ISFINITE(u_applied)) {
		_u_prev = math::constrain(u_applied, -5.f, 5.f);
	}
}

void LADRC1::reset()
{
	// 下一拍 update 以当前测量/指令热启动; TD 吸附到新指令
	_initialized = false;
	_snap = true;
	_z2 = 0.f;
	_u_prev = 0.f;
}
