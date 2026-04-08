/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/**
 * UAVCAN mode
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables support for UAVCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for UAVCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for UAVCAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to UAVCAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

/**
 * UAVCAN Node ID.
 *
 * Read the specs at https://dronecan.github.io/ to learn more about Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_NODE_ID, 1);

/**
 * UAVCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_BITRATE, 1000000);

/**
 * UAVCAN rangefinder minimum range
 *
 * This parameter defines the minimum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MIN, 0.0f);

/**
 * UAVCAN rangefinder maximum range
 *
 * This parameter defines the maximum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MAX, 999.0f);

/**
 * UAVCAN fuel tank maximum capacity
 *
 * This parameter defines the maximum fuel capacity of the vehicle's fuel tank.
 *
 * @min 0.0
 * @max 100000.0
 * @unit liters
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_ECU_MAXF, 15.0f);

/**
 * UAVCAN fuel tank fuel type
 *
 * This parameter defines the type of fuel used in the vehicle's fuel tank.
 *
 * 0: Unknown
 * 1: Liquid (e.g., gasoline, diesel)
 * 2: Gas (e.g., hydrogen, methane, propane)
 *
 * @min 0
 * @max 2
 * @value 0 Unknown
 * @value 1 Liquid
 * @value 2 Gas
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ECU_FUELT, 1);

/**
 * UAVCAN ANTI_COLLISION light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the ANTI_COLLISION lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_ANTCL, 2);

/**
 * UAVCAN STROBE light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the STROBE lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_STROB, 1);

/**
 * UAVCAN RIGHT_OF_WAY light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the RIGHT_OF_WAY lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_NAV, 3);

/**
 * UAVCAN LIGHT_ID_LANDING light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the LIGHT_ID_LANDING lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_LAND, 0);

/**
 * publish Arming Status stream
 *
 * Enable UAVCAN Arming Status stream publication
 *  uavcan::equipment::safety::ArmingStatus
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_ARM, 0);

/**
 * publish attitude and position
 *
 * Enable UAVCAN attitude and position publication
 *  uavcan::equipment::ahrs::Solution
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_ATT, 0);

/**
 * publish control command
 *
 * Enable UAVCAN control command publication
 *  uavcan::equipment::actuator::ArrayCommand
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_CTRL, 0);

/**
 * publish formation rates
 *
 * Enable UAVCAN formation control input publication
 *  dronecan::formation::ControlInput
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_FORM, 0);

/**
 * publish RTCM stream
 *
 * Enable UAVCAN RTCM stream publication
 *  uavcan::equipment::gnss::RTCMStream
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_RTCM, 0);

/**
 * publish moving baseline data RTCM stream
 *
 * Enable UAVCAN RTCM stream publication
 *  ardupilot::gnss::MovingBaselineData
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_MBD, 0);

/**
 * subscription airspeed
 *
 * Enable UAVCAN airspeed subscriptions.
 *  uavcan::equipment::air_data::IndicatedAirspeed
 *  uavcan::equipment::air_data::TrueAirspeed
 *  uavcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_ASPD, 0);

/**
 * subscription barometer
 *
 * Enable UAVCAN barometer subscription.
 *  uavcan::equipment::air_data::StaticPressure
 *  uavcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BARO, 0);

/**
 * subscription battery
 *
 * Enable UAVCAN battery subscription.
 *  uavcan::equipment::power::BatteryInfo
 *  ardupilot::equipment::power::BatteryInfoAux
 *  cuav::equipment::power::CBAT
 *
 *  0 - Disable
 *  1 - Use raw data. Recommended for Smart battery
 *  2 - Filter the data with internal battery library (unsupported with CBAT)
 *
 * @min 0
 * @max 2
 * @value 0 Disable
 * @value 1 Raw data
 * @value 2 Filter data
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BAT, 0);

/**
 * subscription differential pressure
 *
 * Enable UAVCAN differential pressure subscription.
 *  uavcan::equipment::air_data::RawAirData
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_DPRES, 0);

/**
 * subscription flow
 *
 * Enable UAVCAN optical flow subscription.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_FLOW, 0);

/**
 * subscription fuel tank
 *
 * Enable UAVCAN fuel tank status subscription.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_FUEL, 0);

/**
 * subscription GPS
 *
 * Enable UAVCAN GPS subscriptions.
 *  uavcan::equipment::gnss::Fix
 *  uavcan::equipment::gnss::Fix2
 *  uavcan::equipment::gnss::Auxiliary
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_GPS, 1);

/**
 * subscription GPS Relative
 *
 * Enable UAVCAN GPS Relative subscription.
 * ardupilot::gnss::RelPosHeading
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_GPS_R, 1);

/**
 * subscription hygrometer
 *
 * Enable UAVCAN hygrometer subscriptions.
 *  dronecan::sensors::hygrometer::Hygrometer
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_HYGRO, 0);

/**
 * subscription ICE
 *
 * Enable UAVCAN internal combustion engine (ICE) subscription.
 *  uavcan::equipment::ice::reciprocating::Status
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_ICE, 0);

/**
 * subscription IMU
 *
 * Enable UAVCAN IMU subscription.
 *  uavcan::equipment::ahrs::RawIMU
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_IMU, 0);

/**
 * subscription magnetometer
 *
 * Enable UAVCAN mag subscription.
 *  uavcan::equipment::ahrs::MagneticFieldStrength
 *  uavcan::equipment::ahrs::MagneticFieldStrength2
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_MAG, 1);

/**
 * subscription attitude
 *
 * Enable UAVCAN attitude subscription.
 *  uavcan::equipment::ahrs::Solution
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_ATT, 0);

/**
 * subscription control command
 *
 * Enable UAVCAN control command subscription.
 *  uavcan::equipment::actuator::ArrayCommand
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_CTRL, 0);

/**
 * subscription formation rates
 *
 * Enable UAVCAN formation control input subscription.
 *  dronecan::formation::ControlInput
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_FORM, 0);

/**
 * subscription range finder
 *
 * Enable UAVCAN range finder subscription.
 *  uavcan::equipment::range_sensor::Measurement
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_RNG, 0);

/**
 * subscription button
 *
 * Enable UAVCAN button subscription.
 *  ardupilot::indication::Button
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BTN, 0);

/**
 * subscription MovingBaselineData
 *
 * Enable UAVCAN MovingBaselineData subscription.
 *  ardupilot::gnss::MovingBaselineData
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_MBD, 0);

/**
 * Formation follower enable
 *
 * Enable the formation follower controller.
 * Set to 1 on left/right follower aircraft only.
 *
 * @boolean
 * @group Formation Control
 * @reboot_required true
 */
PARAM_DEFINE_INT32(FORM_FOLLOWER_EN, 0);

/**
 * Formation position
 *
 * Aircraft position in formation:
 * 0 = Center (master)
 * 1 = Left follower
 * 2 = Right follower
 *
 * This must match the physical connection position.
 *
 * @value 0 Center
 * @value 1 Left
 * @value 2 Right
 * @group Formation Control
 */
PARAM_DEFINE_INT32(FORM_POSITION, 0);

/**
 * Roll to pitch mapping gain
 *
 * Converts master aircraft roll input to follower pitch rate command.
 * Physical meaning: master roll input (normalized) → follower pitch rate (rad/s).
 *
 * Larger values = stronger "whole-plane-as-aileron" effect.
 *
 * @unit rad/s
 * @min 0.5
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_R2P_GAIN, 2.0f);

/**
 * Follower roll compensation gain
 *
 * Follower-side self-stabilization gain for body-rate setpoint:
 * roll_rate_sp = FORM_ROLL_COMP * (roll_cmd_tx - self_roll).
 *
 * @unit norm
 * @min 0.5
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_COMP, 2.0f);

/**
 * Relative roll offset for follower tracking
 *
 * Desired roll offset relative to the leader.
 *
 * @unit rad
 * @min -1.57
 * @max 1.57
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_OFS, 0.0f);

/**
 * Relative pitch offset for follower tracking
 *
 * Desired pitch offset magnitude relative to the leader.
 * Left/right followers apply opposite signs internally.
 *
 * @unit rad
 * @min -1.57
 * @max 1.57
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_OFS, 0.0f);

/**
 * Relative yaw offset for follower tracking
 *
 * Desired yaw offset relative to the leader.
 *
 * @unit rad
 * @min -3.14
 * @max 3.14
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_OFS, 0.0f);

/**
 * Maximum follower roll attitude setpoint magnitude
 *
 * Roll stick command is scaled to this maximum absolute attitude.
 *
 * @unit rad
 * @min 0.1
 * @max 1.57
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_AMAX, 0.52f);

/**
 * Maximum follower pitch attitude setpoint magnitude
 *
 * Pitch stick command is scaled to this maximum absolute attitude.
 *
 * @unit rad
 * @min 0.1
 * @max 1.57
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PTCH_AMAX, 0.35f);

/**
 * Relative roll attitude feedforward gain
 *
 * Feedforward gain applied to the leader roll attitude in the
 * follower's auxiliary attitude-correction branch.
 *
 * @unit norm
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_FF, 0.5f);

/**
 * Relative roll attitude proportional gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_KP, 2.0f);

/**
 * Relative roll rate-difference damping gain
 *
 * Scales the roll-rate difference term before it is converted into an
 * auxiliary attitude correction.
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_KD, 0.0f);

/**
 * Relative pitch attitude feedforward gain
 *
 * Feedforward gain applied to the leader pitch attitude in the
 * follower's auxiliary attitude-correction branch.
 *
 * @unit norm
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_FF, 0.5f);

/**
 * Relative pitch attitude proportional gain
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_KP, 2.0f);

/**
 * Relative pitch rate-difference damping gain
 *
 * Scales the pitch-rate difference term before it is converted into an
 * auxiliary attitude correction.
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_KD, 0.0f);

/**
 * Reserved yaw auxiliary feedforward gain
 *
 * Reserved for future dedicated yaw auxiliary control tuning.
 * The current attitude-injection implementation uses simple yaw following.
 *
 * @unit norm
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_FF, 0.5f);

/**
 * Reserved yaw auxiliary proportional gain
 *
 * Reserved for future dedicated yaw auxiliary control tuning.
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_KP, 2.0f);

/**
 * Reserved yaw auxiliary damping gain
 *
 * Reserved for future dedicated yaw auxiliary control tuning.
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_KD, 0.0f);

/**
 * Maximum follower roll rate setpoint
 *
 * @unit rad/s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_ROLL_RMAX, 1.22f);

/**
 * Maximum follower pitch rate setpoint
 *
 * @unit rad/s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PTCH_RMAX, 1.05f);

/**
 * Maximum follower yaw rate setpoint
 *
 * @unit rad/s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_RMAX, 0.87f);

/**
 * Yaw throttle boost coefficient
 *
 * Additional throttle applied to the outer follower during yaw maneuvers.
 * Positive yaw boosts the left follower, negative yaw boosts the right follower.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_K, 0.3f);

/**
 * Pitch synchronization coefficient
 *
 * How much follower planes should track master pitch input.
 * 0.0 = no sync, 1.0 = full sync.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_SYNC, 0.1f);

/**
 * Pitch self-stabilization coefficient
 *
 * Additional pitch compensation based on the follower's own pitch angle.
 * Larger values increase pitch disturbance rejection.
 *
 * @unit norm
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_PITCH_COMP, 1.0f);

/**
 * Yaw synchronization coefficient
 *
 * How much follower planes should track master yaw input.
 * 0.0 = no sync, 1.0 = full sync.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_SYNC, 1.0f);

/**
 * Yaw self-stabilization coefficient
 *
 * Additional damping based on the follower's own yaw rate.
 * Larger values increase yaw disturbance rejection.
 *
 * @unit norm
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.1
 * @group Formation Control
 */
PARAM_DEFINE_FLOAT(FORM_YAW_COMP, 0.3f);
