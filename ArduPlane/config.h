#pragma once

#include "defines.h"

// Just so that it's completely clear...
// 以便完全清楚...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
// 这样可以避免非常常见的配置错误
#define ENABLE ENABLED
#define DISABLE DISABLED

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
// 硬件配置和连接
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is depreated! use CONFIG_HAL_BOARD instead.
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// Advanced Failsafe support
// 高级故障保护支持
//

#ifndef ADVANCED_FAILSAFE
 # define ADVANCED_FAILSAFE ENABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// Optical flow sensor support
// 光流传感器支持
//

#ifndef OPTFLOW
#if AP_AHRS_NAVEKF_AVAILABLE
 # define OPTFLOW ENABLED
#else
 # define OPTFLOW DISABLED
#endif
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
// 无线电配置
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#ifndef FLAP_1_PERCENT
 # define FLAP_1_PERCENT 0
#endif
#ifndef FLAP_1_SPEED
 # define FLAP_1_SPEED 0
#endif
#ifndef FLAP_2_PERCENT
 # define FLAP_2_PERCENT 0
#endif
#ifndef FLAP_2_SPEED
 # define FLAP_2_SPEED 0
#endif
//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
// 飞行模式
// FLIGHT_MODE_CHANNEL
// 飞行模式通道
//
#ifndef FLIGHT_MODE_CHANNEL
 # define FLIGHT_MODE_CHANNEL    8
#endif
#if (FLIGHT_MODE_CHANNEL != 5) && (FLIGHT_MODE_CHANNEL != 6) && (FLIGHT_MODE_CHANNEL != 7) && (FLIGHT_MODE_CHANNEL != 8)
 # error XXX
 # error XXX You must set FLIGHT_MODE_CHANNEL to 5, 6, 7 or 8
 # error XXX
#endif

#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif


//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM
// 自动配平
//
#ifndef AUTO_TRIM
 # define AUTO_TRIM                              DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
// 启动行为
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
// 地面启动延迟
//
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             0
#endif

#ifndef DSPOILR_RUD_RATE_DEFAULT
 #define DSPOILR_RUD_RATE_DEFAULT 100
#endif

//////////////////////////////////////////////////////////////////////////////
// CAMERA TRIGGER AND CONTROL
// 相机触发和控制
//
#ifndef CAMERA
 # define CAMERA         ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// MOUNT (ANTENNA OR CAMERA)
// 安装（天线或相机）
//
#ifndef MOUNT
#define MOUNT          ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
// 飞行和导航控制
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
#ifndef AIRSPEED_CRUISE
 # define AIRSPEED_CRUISE                12 // 12 m/s
#endif
#define AIRSPEED_CRUISE_CM AIRSPEED_CRUISE*100


//////////////////////////////////////////////////////////////////////////////
// MIN_GNDSPEED
// 最小地速
//
#ifndef MIN_GNDSPEED
 # define MIN_GNDSPEED                   0 // m/s (0 disables)
#endif
#define MIN_GNDSPEED_CM MIN_GNDSPEED*100


//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
// 电传B空速控制
//
#ifndef AIRSPEED_FBW_MIN
 # define AIRSPEED_FBW_MIN               9
#endif
#ifndef AIRSPEED_FBW_MAX
 # define AIRSPEED_FBW_MAX               22
#endif

#ifndef ALT_HOLD_FBW
 # define ALT_HOLD_FBW 0
#endif
#define ALT_HOLD_FBW_CM ALT_HOLD_FBW*100


//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
// 伺服映射
//
#ifndef THROTTLE_MIN
 # define THROTTLE_MIN                   0 // percent
#endif
#ifndef THROTTLE_CRUISE
 # define THROTTLE_CRUISE                45
#endif
#ifndef THROTTLE_MAX
 # define THROTTLE_MAX                   100
#endif

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
// 自动驾驶仪控制极限
//
#ifndef HEAD_MAX
 # define HEAD_MAX                               45
#endif
#ifndef PITCH_MAX
 # define PITCH_MAX                              20
#endif
#ifndef PITCH_MIN
 # define PITCH_MIN                              -25
#endif
#define HEAD_MAX_CENTIDEGREE HEAD_MAX * 100
#define PITCH_MAX_CENTIDEGREE PITCH_MAX * 100
#define PITCH_MIN_CENTIDEGREE PITCH_MIN * 100

#ifndef RUDDER_MIX
 # define RUDDER_MIX           0.5f
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
// 调试
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Logging control
// 记录控制
//

#ifndef LOGGING_ENABLED
 # define LOGGING_ENABLED                ENABLED
#endif

#define DEFAULT_LOG_BITMASK   0xffff


//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
// 导航默认
//
#ifndef WP_RADIUS_DEFAULT
 # define WP_RADIUS_DEFAULT              90
#endif

#ifndef LOITER_RADIUS_DEFAULT
 # define LOITER_RADIUS_DEFAULT 60
#endif

#ifndef ALT_HOLD_HOME
 # define ALT_HOLD_HOME 100
#endif
#define ALT_HOLD_HOME_CM ALT_HOLD_HOME*100

//////////////////////////////////////////////////////////////////////////////
// Developer Items
// 开发人员项目
//

#ifndef SCALING_SPEED
 # define SCALING_SPEED          15.0
#endif

// use this to disable geo-fencing
// 使用此项禁用地理围栏
#ifndef GEOFENCE_ENABLED
 # define GEOFENCE_ENABLED ENABLED
#endif

// pwm value on FENCE_CHANNEL to use to enable fenced mode
// 占空比值用于启用地理围栏模式
#ifndef FENCE_ENABLE_PWM
 # define FENCE_ENABLE_PWM 1750
#endif

// a digital pin to set high when the geo-fence triggers. Defaults
// to -1, which means don't activate a pin
// 触发地理围栏时将数字引脚设置为高电平。默认值为-1，表示不激活引脚
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

// if RESET_SWITCH_CH is not zero, then this is the PWM value on
// that channel where we reset the control mode to the current switch
// position (to for example return to switched mode after failsafe or
// fence breach)
// 如果RESET_SWITCH_CH不为零，则这是该通道上的PWM值，我们在该通道上将控制模式重置为当前开关位置（例如，在发生故障保护或栅栏破坏后返回到开关模式）
#ifndef RESET_SWITCH_CHAN_PWM
 # define RESET_SWITCH_CHAN_PWM 1750
#endif

#ifndef HIL_SUPPORT
# define HIL_SUPPORT !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
// Parachute release
// 降落伞释放
#ifndef PARACHUTE
#define PARACHUTE ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Payload Gripper
// 负载爪夹
#ifndef GRIPPER_ENABLED
  #define GRIPPER_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef STATS_ENABLED
 # define STATS_ENABLED ENABLED
#endif

#ifndef OSD_ENABLED
 #define OSD_ENABLED DISABLED
#endif

#ifndef SOARING_ENABLED
 #define SOARING_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef LANDING_GEAR_ENABLED
 #define LANDING_GEAR_ENABLED !HAL_MINIMIZE_FEATURES
#endif
