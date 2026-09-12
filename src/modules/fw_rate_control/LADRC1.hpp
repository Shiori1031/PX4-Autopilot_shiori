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
 * @file LADRC1.hpp
 *
 * 一阶线性自抗扰控制器 (LADRC) — 单通道
 *
 * 被控对象: ẏ = f + b0·u  (相对阶 1, 如机体角速率回路)
 *
 *   TD (一阶, 常开):    指令平滑 + 导数前馈 (带宽固定 kTdLambda = 40 rad/s)
 *                        ṙ* = λ·(r − r*), 输出 r* 与 ṙ*
 *   ESO (2 阶):         z1→y, z2→f
 *                        ż1 = z2 − β1·e + b0·u_prev
 *                        ż2 = −β2·e,        e = z1 − y
 *   SEF + 扰动补偿:      u = (ωc·(r* − z1) + ṙ* − z2) / b0
 *
 * 带宽参数化 (Gao, ACC 2003):  β1 = 2ωo, β2 = ωo², kp = ωc
 *
 * 工程要点:
 *  - ESO 使用"实际施加"的控制量 (下游限幅/叠加 trim 后由 setAppliedControl() 写回) → 抗积分饱和
 *  - 首次调用/复位后以当前测量热启动 (z1=y) → 避免初始瞬态
 *
 * 仿真参考实现: docs/reference/LADRC1.m (链翼编队仿真)
 */

#pragma once

class LADRC1
{
public:
	LADRC1() = default;
	~LADRC1() = default;

	/**
	 * 设置增益 (带宽参数化)
	 * @param b0 控制增益 (单位控制量 → 被控量加速度)
	 * @param wo 观测器带宽 [rad/s], 建议 3~5×wc
	 * @param wc 控制器带宽 [rad/s] (一阶闭环极点 -wc)
	 */
	void setGains(float b0, float wo, float wc);

	/**
	 * 一步更新 (TD + ESO + 控制律)
	 * @param y      测量输出 (当前角速率)
	 * @param r      参考信号 (速率设定值)
	 * @param dt     步长 [s]
	 * @param landed 地面标志 (真时观测器保持复位, 防止地面误学习)
	 * @return 控制量 (与 RateControl::update 输出同域)
	 */
	float update(float y, float r, float dt, bool landed);

	/**
	 * 反饱和: 写回"实际施加"的等效控制量 (下游限幅/叠加 trim 之后调用)
	 * ESO 使用该值做模型预测, 饱和时不会把未真正施加的控制计入估计
	 */
	void setAppliedControl(float u_applied);

	/** 复位: 下一拍 update 时以当前测量/指令热启动, 扰动估计清零 */
	void reset();

	/** 总扰动估计 z2 (调试/日志用) */
	float getDisturbanceEstimate() const { return _z2; }

private:
	static constexpr float kTdLambda{40.f};  ///< TD 带宽 [rad/s] (固定, ≈4~5×wc, 接近直通; 如需调整改此常量)

	void updateGains();

	// 增益
	float _b0{1.f};      ///< 控制增益
	float _wo{40.f};     ///< 观测器带宽 [rad/s]
	float _wc{10.f};     ///< 控制器带宽 [rad/s]

	// 带宽参数化增益
	float _beta1{0.f};
	float _beta2{0.f};
	float _kp{0.f};

	// ESO 状态
	float _z1{0.f};       ///< 跟踪 y
	float _z2{0.f};       ///< 跟踪总扰动 f
	float _u_prev{0.f};   ///< 上一拍实际控制 (反饱和)
	bool  _initialized{false};

	// TD 状态 (常开, 带宽固定)
	float _x1{0.f};       ///< 平滑后指令 r*
	bool  _snap{true};    ///< 复位后吸附到新指令
};
