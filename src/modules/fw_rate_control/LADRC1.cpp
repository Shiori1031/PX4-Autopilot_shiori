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

void LADRC1::setGains(float b0, float wc)
{
	_b0 = math::max(b0, kMinB0);
	_wc = math::max(wc, 0.01f);

	updateGains();
}

void LADRC1::updateGains()
{
	// Gao 2003 带宽参数化 (一阶对象); 观测器带宽固定为 4×控制器带宽
	_wo    = kWoRatio * _wc;
	_beta1 = 2.f * _wo;
	_beta2 = _wo * _wo;
	_kp    = _wc;
}

float LADRC1::update(float y, float r, float dt, bool landed)
{
	// 热启动: 首次调用 (或复位后) 观测器从当前测量出发, TD 吸附到当前指令
	if (!_initialized) {
		_z1 = y;
		_x1 = r;
		_initialized = true;
	}

	// TD/ESO 均为显式欧拉: dt 过大 (陀螺掉帧、20 ms backup 调度) 时
	// β1·dt 与 λ·dt 会越过稳定边界, 按观测器带宽限制内部积分步长
	const float dt_int = math::min(dt, kMaxDtRatio / _wo);

	// ① TD: 指令平滑 + 导数前馈 (一阶, 常开)
	const float r_dot = kTdLambda * (r - _x1);
	_x1 += dt_int * r_dot;
	const float r_smooth = _x1;

	// ② ESO (2 阶): z1→y, z2→f
	if (landed || !PX4_ISFINITE(y)) {
		// 地面/信号异常: 保持复位, 防止估计漂移
		// (y 非有限时保留旧 z1, 避免把 NaN 注入观测器)
		if (PX4_ISFINITE(y)) {
			_z1 = y;
		}

		_z2 = 0.f;

	} else {
		const float e = _z1 - y;
		_z1 += dt_int * (_z2 - _beta1 * e + _b0 * _u_prev);
		_z2 += dt_int * (-_beta2 * e);
		// 扰动估计限幅: 模型失配/测量噪声下 z2 会持续累积
		_z2 = math::constrain(_z2, -kZ2MaxRatio * _b0, kZ2MaxRatio * _b0);
	}

	// ③ SEF + ④ 扰动补偿: u = (ωc·(r* − z1) + ṙ* − z2) / b0
	// 微分前馈单独限幅: 先按控制量折算到 r_dot 的等效范围 (±kFfMax·b0)，超出的部分截断成±kFfMax·b0，
	// 再与其余项一起除以 b0；正常小幅前馈不受影响，大指令翻极尖峰被削掉
	const float r_dot_ff = math::constrain(r_dot, -kFfMax * _b0, kFfMax * _b0);
	float u = (_kp * (r_smooth - _z1) + r_dot_ff - _z2) / _b0;

	if (!PX4_ISFINITE(u)) {
		// 异常兜底: 输出清零, 下一拍以当前测量/指令重新热启动
		// (指令恢复有限后自动回正, 防止 NaN 永久锁死该轴)
		u = 0.f;
		_z2 = 0.f;
		_initialized = false;
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
	// 下一拍 update 以当前测量/指令热启动 (TD 同时吸附到新指令)
	_initialized = false;
	_z2 = 0.f;
	_u_prev = 0.f;
}
