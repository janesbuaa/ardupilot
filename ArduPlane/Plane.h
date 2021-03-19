/*
   Lead developer: Andrew Tridgell & Tom Pittenger

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library数学库
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library惯性传感器库
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration 加速度计校准的界面和数学
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library范围查找器库
#include <Filter/Filter.h>                     // Filter library过滤器库
#include <AP_Camera/AP_Camera.h>          // Photo or video camera照相或摄像机
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Stats/AP_Stats.h>     // statistics library统计资料库
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount摄像头/天线安装
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler主循环调度程序
#include <AP_Scheduler/PerfInfo.h>                  // loop perf monitoring循环性能监视

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library遥控输入映射库

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library任务命令库

#include <AP_Soaring/AP_Soaring.h>
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library电池监视器库

#include <AP_Arming/AP_Arming.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_OSD/AP_OSD.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library光流库
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library起落架库

#include "GCS_Mavlink.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include "tuning.h"

// Configuration配置
#include "config.h"

#if ADVANCED_FAILSAFE == ENABLED
#include "afs_plane.h"
#endif

// Local modules本地模块
#include "defines.h"
#include "mode.h"

#ifdef ENABLE_SCRIPTING
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel.h"     // RC Channel Library遥控通道库
#include "Parameters.h"
#include "avoidance_adsb.h"
#include "AP_Arming.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

/*
  main APM:Plane class
 */
class Plane : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Plane;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Plane;
    friend class QuadPlane;
    friend class QAutoTune;
    friend class AP_Tuning_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class AP_Avoidance_Plane;
    friend class GCS_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channels_Plane;

    friend class Mode;
    friend class ModeCircle;
    friend class ModeStabilize;
    friend class ModeTraining;
    friend class ModeAcro;
    friend class ModeFBWA;
    friend class ModeFBWB;
    friend class ModeCruise;
    friend class ModeAutoTune;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeLoiter;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeInitializing;
    friend class ModeManual;
    friend class ModeQStabilize;
    friend class ModeQHover;
    friend class ModeQLoiter;
    friend class ModeQLand;
    friend class ModeQRTL;
    friend class ModeQAcro;
    friend class ModeQAutotune;
    friend class ModeTakeoff;

    Plane(void);

    // HAL::Callbacks implementation实现.
    void setup() override;
    void loop() override;

private:

    // key aircraft parameters passed to multiple libraries
    // 关键飞机参数传递到多个库
    AP_Vehicle::FixedWing aparm;

    // Global parameters are all contained within the 'g' and 'g2' classes.
    // 全局参数都包含在'g'和'g2'类中。
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    // 主循环调度器
    AP_Scheduler scheduler;

    // mapping between input channels
    // 输入通道之间的映射
    RCMapper rcmap;

    // primary input channels
    // 主要输入通道
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;

    AP_Logger logger;

    // scaled roll limit based on pitch
    // 根据俯仰缩放滚动限制
    int32_t roll_limit_cd;
    int32_t pitch_limit_min_cd;

    // flight modes convenience array
    // 飞行模式便利性数组
    AP_Int8 *flight_modes = &g.flight_mode1;

    AP_Vehicle::FixedWing::Rangefinder_State rangefinder_state;

    AP_RPM rpm_sensor;

// Inertial Navigation EKF
// 惯性导航EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF2 EKF2{&ahrs, rangefinder};
    NavEKF3 EKF3{&ahrs, rangefinder};
    AP_AHRS_NavEKF ahrs{EKF2, EKF3};
#else
    AP_AHRS_DCM ahrs;
#endif

    AP_TECS TECS_controller{ahrs, aparm, landing};
    AP_L1_Control L1_controller{ahrs, &TECS_controller};

    // Attitude to servo controllers
    // 对伺服控制器的姿态
    AP_RollController rollController{ahrs, aparm};
    AP_PitchController pitchController{ahrs, aparm};
    AP_YawController yawController{ahrs, aparm};
    AP_SteerController steerController{ahrs};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Training mode
    // 训练模式
    bool training_manual_roll;  // user has manual roll control用户具有手动滚转控制
    bool training_manual_pitch; // user has manual pitch control用户具有手动俯仰控制

    /*
      keep steering and rudder control separated until we update servos,
      to allow for a separate wheel servo from rudder servo
      保持方向舵和转弯的控制分开，直到我们更新舵机，允许独立于方向舵与转弯舵机
    */
    struct {
        bool ground_steering; // are we doing ground steering?我们在进行地面转向吗？
        int16_t steering; // value for nose/tail wheel前鼻/尾轮的值
        int16_t rudder;   // value for rudder方向舵的值
    } steering_control;

    // should throttle be pass-thru in guided?油门应该在引导下通过吗？
    bool guided_throttle_passthru;

    // are we doing calibration? This is used to allow heartbeat to
    // external failsafe boards during baro and airspeed calibration
    // 我们在做校准吗？ 用于在气压和空速校准期间允许心跳到外部故障保护板
    bool in_calibration;

    // GCS selectionGCS选择
    GCS_Plane _gcs; // avoid using this; use gcs()
    GCS_Plane &gcs() { return _gcs; }

    // selected navigation controller选择导航控制器
    AP_Navigation *nav_controller = &L1_controller;

    // selected navigation controller选择导航控制器
    AP_SpdHgtControl *SpdHgt_Controller = &TECS_controller;

    // Camera相机
#if CAMERA == ENABLED
    AP_Camera camera{MASK_LOG_CAMERA, current_loc};
#endif

#if OPTFLOW == ENABLED
    // Optical flow sensor光流传感器
    OpticalFlow optflow;
#endif

    // Rally Points
    // 备降点
    AP_Rally rally;

#if OSD_ENABLED == ENABLED
    AP_OSD osd;
#endif
    
    ModeCircle mode_circle;
    ModeStabilize mode_stabilize;
    ModeTraining mode_training;
    ModeAcro mode_acro;
    ModeFBWA mode_fbwa;
    ModeFBWB mode_fbwb;
    ModeCruise mode_cruise;
    ModeAutoTune mode_autotune;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
    ModeLoiter mode_loiter;
    ModeAvoidADSB mode_avoidADSB;
    ModeGuided mode_guided;
    ModeInitializing mode_initializing;
    ModeManual mode_manual;
    ModeQStabilize mode_qstabilize;
    ModeQHover mode_qhover;
    ModeQLoiter mode_qloiter;
    ModeQLand mode_qland;
    ModeQRTL mode_qrtl;
    ModeQAcro mode_qacro;
    ModeQAutotune mode_qautotune;
    ModeTakeoff mode_takeoff;

    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    // 这是飞控系统的状态,定义了多种状态，例如MANUAL，FBW-A，AUTO
    Mode *control_mode = &mode_initializing;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;
    Mode *previous_mode = &mode_initializing;
    ModeReason previous_mode_reason = ModeReason::UNKNOWN;

    // time of last mode change
    // 最后一次模式更改的时间
    uint32_t last_mode_change_ms;

    // Used to maintain the state of the previous control switch position
    // This is set to 254 when we need to re-read the switch
    // 用于维持先前控制开关位置的状态，需要重新读取开关时将其设置为254
    uint8_t oldSwitchPosition = 254;

    // This is used to enable the inverted flight feature
    // 这用于启用倒飞功能
    bool inverted_flight;

    // last time we ran roll/pitch stabilization
    // 上一次滚转/俯仰稳定的时间
    uint32_t last_stabilize_ms;
    
    // Failsafe
    // 故障保护
    struct {
        // Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
        // RC receiver should be set up to output a low throttle value when signal is lost
        // 用于跟踪通道3（油门）上的值是否已低于故障安全阈值
        // RC接收器应设置为在信号丢失时输出低油门值
        bool rc_failsafe;

        // has the saved mode for failsafe been set?
        // 设置的模式是否设置了故障保护
        bool saved_mode_set;

        // true if an adsb related failsafe has occurred
        // 如果发生与ads-b相关的故障保护，则为true
        bool adsb;

        // saved flight mode
        // 保存的飞行模式
        enum Mode::Number saved_mode_number;

        // A tracking variable for type of failsafe active
        // Used for failsafe based on loss of RC signal or GCS signal
        // 主动故障保护类型的跟踪变量,用于基于遥控信号或地面站信号丢失的故障保护
        int16_t state;

        // number of low throttle values
        // 低油门数值
        uint8_t throttle_counter;

        // the time when the last HEARTBEAT message arrived from a GCS
        // 最后一则心跳消息从地面站到达的时间
        uint32_t last_heartbeat_ms;
        
        // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
        // 一个计时器，用于跟踪由于遥控信号丢失而处于“短故障保护”状态的时间
        uint32_t short_timer_ms;
        
        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        //保留与AFS系统相关的最后一个有效遥控的跟踪
        //如果启用了标准故障保护功能，则不将遥控输入视为有效
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

    enum Landing_ApproachStage {
        LOITER_TO_ALT,
        ENSURE_RADIUS,
        WAIT_FOR_BREAKOUT,
        APPROACH_LINE,
        VTOL_LANDING,
    };

    // Landing降落
    struct {
        enum Landing_ApproachStage approach_stage;
        float approach_direction_deg;
    } vtol_approach_s;

    bool any_failsafe_triggered() {
        return failsafe.state != FAILSAFE_NONE || battery.has_failsafed() || failsafe.adsb;
    }

    // A counter used to count down valid gps fixes to allow the gps estimate to settle
    // before recording our home position (and executing a ground start if we booted with an air start)
    // 用来递减有效gps修正值的计数器，以使gps估算值稳定下来
    // 在记录我们的原点位置之前（如果是空中启动则执行地面启动）
    uint8_t ground_start_count = 5;

    // true if we have a position estimate from AHRS
    // 如果我们有AHRS的职位估算，则为true
    bool have_position;

    // Airspeed
    // The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
    // Also used for flap deployment criteria.  Centimeters per second.
    // 空速
    // 计算出的要在FBW-B中使用的空速。还用于更高的模式，以确保达到最小地面速度。
    // 也用于襟翼偏转标准。厘米/秒。
    int32_t target_airspeed_cm;

    // The difference between current and desired airspeed.  Used in the pitch controller.  Meters per second.
    // 当前和期望空速之间的差。用于音高控制器。米/秒。
    float airspeed_error;

    // An amount that the airspeed should be increased in auto modes based on the user positioning the
    // throttle stick in the top half of the range.  Centimeters per second.
    // 根据用户放置的位置，在自动模式下应提高空速的数量,油门摇杆位于范围的上半部分。厘米/秒。
    int16_t airspeed_nudge_cm;

    // Similar to airspeed_nudge, but used when no airspeed sensor.
    // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
    // 与airspeed_nudge相似，但是在没有空速传感器时使用。
    // 0-（throttle_max-油门行程）：在自动模式下使用油门杆行程的前1/2轻推油门
    int16_t throttle_nudge;

    // Ground speed
    // The amount current ground speed is below min ground speed.  Centimeters per second
    // 地面速度
    // 当前地面速度的数量低于最小地面速度。厘米/秒
    int32_t groundspeed_undershoot;

    // Difference between current altitude and desired altitude.  Centimeters
    // 当前海拔和期望海拔之间的差。公分
    int32_t altitude_error_cm;

    // Battery Sensors电池传感器
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    // Airspeed Sensors空速传感器
    AP_Airspeed airspeed;

    // ACRO controller state
    // ACRO控制器状态
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
    } acro_state;

    // CRUISE controller state
    // 巡航控制器状态
    struct CruiseState {
        bool locked_heading;
        int32_t locked_heading_cd;
        uint32_t lock_timer_ms;
    } cruise_state;

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
    } takeoff_state;

    // ground steering controller state
    // 地面转向控制器状态
    struct {
        // Direction held during phases of takeoff and landing centidegrees
        // A value of -1 indicates the course has not been set/is not in use
        // this is a 0..36000 value, or -1 for disabled
        // 在起飞和着陆摄氏度阶段保持的方向
        // 值-1表示课程尚未设置/未使用
        // 这是一个0..36000值，或-1表示已禁用
        int32_t hold_course_cd = -1;

        // locked_course and locked_course_cd are used in stabilize mode 
        // when ground steering is active, and for steering in auto-takeoff
        // 在稳定模式下使用locked_course和locked_course_cd
        // 当地面转向处于活动状态且用于自动起飞转向时
        bool locked_course;
        float locked_course_err;
    } steer_state;

    // flight mode specific
    // 特定于飞行模式
    struct {
        // Altitude threshold to complete a takeoff command in autonomous
        // modes.  Centimeters above home
        // 自主完成起飞命令的高度阈值模式。起飞点上多少厘米
        int32_t takeoff_altitude_rel_cm;

        // Begin leveling out the enforced takeoff pitch angle min at this height to reduce/eliminate overshoot
        // 在此高度开始拉平强制起飞俯仰角min，以减少/消除超调
        int32_t height_below_takeoff_to_level_off_cm;

        // the highest airspeed we have reached since entering AUTO. Used
        // to control ground takeoff
        // 自进入AUTO以来我们达到的最高空速。用来控制地面起飞
        float highest_airspeed;
        
        // turn angle for next leg of mission
        // 为下一个任务转弯角度
        float next_turn_angle {90};

        // filtered sink rate for landing
        // 过滤后的降落速率
        float sink_rate;

        // time when we first pass min GPS speed on takeoff
        // 起飞时我们首次通过最小GPS速度的时间
        uint32_t takeoff_speed_time_ms;
        
        // distance to next waypoint
        // 到下一个航点的距离
        float wp_distance;
        
        // proportion to next waypoint
        // 与下一个航点的比例
        float wp_proportion;
        
        // last time is_flying() returned true in milliseconds
        // 上一次is_flying（）返回true的时间（以毫秒为单位）
        uint32_t last_flying_ms;

        // time stamp of when we start flying while in auto mode in milliseconds
        // 我们在自动模式下开始飞行的时间戳（以毫秒为单位）
        uint32_t started_flying_in_auto_ms;

        // barometric altitude at start of takeoff
        // 起飞开始时的气压高度
        float baro_takeoff_alt;

        // initial pitch. Used to detect if nose is rising in a tail dragger
        // 初始音高。用于检测拖尾器中鼻子是否在上升
        int16_t initial_pitch_cd;

        // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
        // 起飞命令执行期间要保持的最小音高。百分之一度
        int16_t takeoff_pitch_cd;

        // used to 'wiggle' servos in idle mode to prevent them freezing
        // at high altitudes
        // 用于在空闲模式下“摆动”伺服器以防止其在高海拔下被冻结
        uint8_t idle_wiggle_stage;

        // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
        // 标记为使用GPS地面路线而不是INS偏航。处理起飞命令时将其设置为false。
        bool takeoff_complete;

        // are we headed to the land approach waypoint? Works for any nav type
        // 我们要前往着陆点吗？适用于任何导航类型
        bool wp_is_land_approach;

        // should we fly inverted?
        // 我们应该倒飞吗？
        bool inverted_flight;

        // should we enable cross-tracking for the next waypoint?
        // 我们应该为下一个航点启用交叉跟踪吗？
        bool next_wp_crosstrack;

        // should we use cross-tracking for this waypoint?
        // 我们是否应对此交叉点使用交叉跟踪？
        bool crosstrack;

        // in FBWA taildragger takeoff mode
        // 在尾坐飞机电传起飞模式下
        bool fbwa_tdrag_takeoff_mode;

        // have we checked for an auto-land?
        // 我们是否检查了自动降落？
        bool checked_for_autoland;

        // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
        // are we in idle mode? used for balloon launch to stop servo
        // movement until altitude is reached
        // 在自主模式下完成起飞命令的高度阈值。 厘米我们处于空闲模式吗？
        // 用于气球发射，以停止伺服运动，直到达到高度
        bool idle_mode;

        // are we in VTOL mode in AUTO?
        // 我们是否在AUTO模式下处于VTOL模式？
        bool vtol_mode;

        // are we doing loiter mode as a VTOL?
        // 我们是否正在将游荡模式作为VTOL？
        bool vtol_loiter;

        // how much correction have we added for terrain data
        // 我们为地形数据添加了多少校正
        float terrain_correction;
    } auto_state;

    struct {
        // roll pitch yaw commanded from external controller in centidegrees
        // 外部控制器命令的俯仰偏航角（以分为单位）
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        // 上次收到外部控制器的消息
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        // 外部控制器发出的油门百分比
        float forced_throttle;
        uint32_t last_forced_throttle_ms;
    } guided_state;

#if LANDING_GEAR_ENABLED == ENABLED
    // landing gear state
    // 起落架状态
    struct {
        AP_Vehicle::FixedWing::FlightStage last_flight_stage;
    } gear;
#endif
    
    struct {
        // on hard landings, only check once after directly a landing so you
        // don't trigger a crash when picking up the aircraft
        // 在硬着陆时，只需要在直接着陆后检查一次即可，这样在拿起飞机时不会触发坠机
        bool checkedHardLanding;

        // crash detection. True when we are crashed
        // 坠机检测。当我们坠毁时为真
        bool is_crashed;

        // impact detection flag. Expires after a few seconds via impact_timer_ms
        // 碰撞检测标志。几秒钟后通过impact_timer_ms过期
        bool impact_detected;

        // debounce timer
        // 反跳计时器
        uint32_t debounce_timer_ms;

        // delay time for debounce to count to
        // 延迟时间，以防抖动计数
        uint32_t debounce_time_total_ms;

        // length of time impact_detected has been true. Times out after a few seconds. Used to clip isFlyingProbability
        // 检测到的影响的时间长度是正确的。 几秒钟后超时。 用于剪辑飞行概率
        uint32_t impact_timer_ms;
    } crash_state;

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    // 如果我们处于自动油门模式，则为true，这意味着我们需要运行速度/高度控制器
    bool auto_throttle_mode:1;

    // true if we are in an auto-navigation mode, which controls whether control input is ignored
    // with STICK_MIXING=0
    // 如果我们处于自动导航模式，则为true，它控制是否使用STICK_MIXING = 0忽略控件输入
    bool auto_navigation_mode:1;
    
    // this allows certain flight modes to mix RC input with throttle depending on airspeed_nudge_cm
    // 这允许某些飞行模式根据空速微移cm将RC输入与油门混合
    bool throttle_allows_nudging:1;

    // this controls throttle suppression in auto modes
    // 在自动模式下控制油门抑制
    bool throttle_suppressed;
	
    // reduce throttle to eliminate battery over-current
    // 降低油门以消除电池过电流
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust反向推力
    uint32_t throttle_watt_limit_timer_ms;

    AP_Vehicle::FixedWing::FlightStage flight_stage = AP_Vehicle::FixedWing::FLIGHT_NORMAL;

    // probability of aircraft is currently in flight. range from 0 to
    // 1 where 1 is 100% sure we're in flight
    // 飞机目前正在飞行的概率。 范围从0到1，其中1是100％确保我们正在飞行
    float isFlyingProbability;

    // previous value of is_flying()
    // is_flying（）的先前值
    bool previous_is_flying;

    // time since started flying in any mode in milliseconds
    // 自开始以任何模式飞行以来的时间（以毫秒为单位）
    uint32_t started_flying_ms;

    // Navigation control variables
    // The instantaneous desired bank angle.  Hundredths of a degree
    // 导航控制变量，瞬时所需的倾斜角。百分之一度
    int32_t nav_roll_cd;

    // The instantaneous desired pitch angle.  Hundredths of a degree
    // 瞬时所需的俯仰角。百分之一度
    int32_t nav_pitch_cd;

    // the aerodymamic load factor. This is calculated from the demanded
    // roll before the roll is clipped, using 1/sqrt(cos(nav_roll))
    // 气动载荷系数。 这是根据滚转前所需的滚转计算得出的，使用1 / sqrt（cos（nav_roll））
    float aerodynamic_load_factor = 1.0f;

    // a smoothed airspeed estimate, used for limiting roll angle
    // 平滑的空速估算值，用于限制侧倾角
    float smoothed_airspeed;

    // Mission library
    // 任务库
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if PARACHUTE == ENABLED
    AP_Parachute parachute{relay};
#endif

    // terrain handling
    // 地形处理
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain{mission};
#endif

    AP_Landing landing{mission,ahrs,SpdHgt_Controller,nav_controller,aparm,
            FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
            FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
            FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};

    AP_ADSB adsb;

    // avoidance of adsb enabled vehicles (normally manned vheicles)
    // 避免使用启用了ads-b的车辆（通常是载人车辆）
    AP_Avoidance_Plane avoidance_adsb{adsb};

    // Outback Challenge Failsafe Support
    // 内陆挑战赛的故障保护支持
#if ADVANCED_FAILSAFE == ENABLED
    AP_AdvancedFailsafe_Plane afs {mission};
#endif

    /*
      meta data to support counting the number of circles in a loiter
      元数据以支持计算盘旋的圈数
    */
    struct {
        // previous target bearing, used to update sum_cd
        // 先前的目标方位角，用于更新sum_cd
        int32_t old_target_bearing_cd;

        // Total desired rotation in a loiter.  Used for Loiter Turns commands.
        // 盘旋中所需的总旋转次数。用于“盘旋转弯”命令。
        int32_t total_cd;

        // total angle completed in the loiter so far
        // 到目前为止，盘旋完成的总角度
        int32_t sum_cd;

        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        // 盘旋的方向。 1代表顺时针，-1代表逆时针
        int8_t direction;

        // when loitering and an altitude is involved, this flag is true when it has been reached at least once
        // 当盘旋并涉及到海拔高度时，如果至少达到一次，则此标志为true
        bool reached_target_alt;

        // check for scenarios where updrafts can keep you from loitering down indefinitely.
        // 检查上升气流可能会阻止您无限下沉的情况。
        bool unable_to_acheive_target_alt;

        // start time of the loiter.  Milliseconds.
        // 盘旋的开始时间。毫秒。
        uint32_t start_time_ms;

        // altitude at start of loiter loop lap. Used to detect delta alt of each lap.
        // only valid when sum_cd > 36000
        // 盘旋开始时的海拔高度。 用于检测每圈的增量变化。 仅在sum_cd> 36000时有效
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        // 我们应该在“盘旋时间”命令中停留在盘旋中的时间。毫秒。
        uint32_t time_max_ms;
    } loiter;


    // Conditional command条件命令
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    // 条件命令中使用的值（例如，延迟，更改alt等）.例如，在更改高度命令中，它是要更改为的高度。
    int32_t condition_value;

    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    // 用于检查条件命令状态的起始值。
    // 例如，在delay命令中，condition_start记录该延迟的开始时间
    uint32_t condition_start;
    // A value used in condition commands.  For example the rate at which to change altitude.
    // 条件命令中使用的值。例如，改变高度的速率。
    int16_t condition_rate;

    // 3D Location vectors
    // Location structure defined in AP_Common
    // 3D位置向量。在AP_Common中定义的位置结构
    const struct Location &home = ahrs.get_home();

    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    // 前一个航点的位置。用于跟踪和高度坡道计算
    Location prev_WP_loc {};

    // The plane's current location
    // 飞机的当前位置
    struct Location current_loc {};

    // The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
    // 当前/活动航路点的位置。用于高度坡道，跟踪和游荡者的计算。
    Location next_WP_loc {};

    // The location of the active waypoint in Guided mode.
    // 引导模式下活动航路点的位置。
    struct Location guided_WP_loc {};

    // Altitude control
    // 高度控制
    struct {
        // target altitude above sea level in cm. Used for barometric
        // altitude navigation
        // 目标海拔高度，以厘米为单位。 用于气压高度导航
        int32_t amsl_cm;

        // Altitude difference between previous and current waypoint in
        // centimeters. Used for glide slope handling
        // 先前和当前航路点之间的高度差（以厘米为单位）。 用于滑翔处理
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // are we trying to follow terrain?
        // 我们是否要地形跟随？
        bool terrain_following;

        // target altitude above terrain in cm, valid if terrain_following
        // is set
        // 地形上方的目标高度（以厘米为单位），如果设置了地形跟随，则有效
        int32_t terrain_alt_cm;

        // lookahead value for height error reporting
        // 高度错误报告的超前值
        float lookahead;
#endif

        // last input for FBWB/CRUISE height control
        // FBWB / CRUISE高度控制的最后一个输入
        float last_elevator_input;

        // last time we checked for pilot control of height
        // 上一次我们检查高度的先导控制
        uint32_t last_elev_check_us;
    } target_altitude {};

    float relative_altitude;

    // INS variables    INS变量
    // The main loop execution time.  Seconds
    // This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
    // 主循环执行时间。秒
    // 这是调用DCM算法之间的时间，是陀螺仪的积分时间。
    float G_Dt = 0.02f;

    // loop performance monitoring:
    // 循环性能监控：
    AP::PerfInfo perf_info;
    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    struct {
        bool done_climb;
    } rtl;

    // last time home was updated while disarmed
    // 上次在撤防时房屋已更新
    uint32_t last_home_update_ms;

    // Camera/Antenna mount tracking and stabilisation stuff
    // 摄像头/天线安装跟踪和稳定功能
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    // current_loc将baro / gps解决方案用于海拔高度，而不仅仅是gps。
    AP_Mount camera_mount{current_loc};
#endif

    // Arming/Disarming mangement class
    // 加锁/解锁管理类
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

    // time that rudder arming has been running
    // 方向舵加锁的时间
    uint32_t rudder_arm_timer;

    // support for quadcopter-plane
    // 支持四轴飞行器平面
    QuadPlane quadplane{ahrs};

    // support for transmitter tuning
    // 支持发射机调谐
    AP_Tuning_Plane tuning;

    static const struct LogStructure log_structure[];

    // rudder mixing gain for differential thrust (0 - 1)
    // 差动推力的舵混合增益（0-1）
    float rudder_dt;

    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void send_fence_status(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_wind(mavlink_channel_t chan);

    void send_aoa_ssa(mavlink_channel_t chan);

    void Log_Write_Fast(void);
    void Log_Write_Attitude(void);
    void Log_Write_Performance();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Control_Tuning();
    void Log_Write_Nav_Tuning();
    void Log_Write_Status();
    void Log_Write_Sonar();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Write_AOA_SSA();
    void Log_Write_AETR();

    void load_parameters(void);
    void convert_mixers(void);
    void adjust_altitude_target();
    void setup_glide_slope(void);
    int32_t get_RTL_altitude();
    float relative_ground_altitude(bool use_rangefinder_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_current_adjusted(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_fbwb_minimum_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc);
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void rangefinder_terrain_correction(float &height);
    void set_next_WP(const struct Location &loc);
    void set_guided_WP(void);
    void update_home();
    // set home location and store it persistently:
    // 设置家庭位置并永久存储：
    bool set_home_persistently(const Location &loc) WARN_IF_UNUSED;
    void do_RTL(int32_t alt);
    bool verify_takeoff();
    bool verify_loiter_unlim(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_time();
    bool verify_loiter_turns(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd);
    bool verify_RTL();
    bool verify_continue_and_change_alt();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_altitude_wait(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(const AP_Mission::Mission_Command &cmd);
    void do_loiter_at_location();
    bool verify_loiter_heading(bool init);
    void exit_mission_callback();
    void mavlink_delay(uint32_t ms);
    void read_control_switch();
    uint8_t readSwitch(void);
    void reset_control_switch();
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    void failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason);
    void failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason);
    void failsafe_short_off_event(ModeReason reason);
    void failsafe_long_off_event(ModeReason reason);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    uint8_t max_fencepoints(void) const;
    Vector2l get_fence_point_with_index(uint8_t i) const;
    void set_fence_point_with_index(const Vector2l &point, unsigned i);
    void geofence_load(void);
    bool geofence_present(void);
    void geofence_update_pwm_enabled_state();
    bool geofence_set_enabled(bool enable);
    bool geofence_enabled(void);
    bool geofence_set_floor_enabled(bool floor_enable);
    bool geofence_check_minalt(void);
    bool geofence_check_maxalt(void);
    void geofence_check(bool altitude_check_only);
    bool geofence_prearm_check(void);
    bool geofence_stickmixing(void);
    void geofence_send_status(mavlink_channel_t chan);
    bool geofence_breached(void);
    void geofence_disable_and_send_error_msg(const char *errorMsg);
    void disarm_if_autoland_complete();
    float tecs_hgt_afe(void);
    void set_nav_controller(void);
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void calc_airspeed_errors();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_cruise();
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);
    void set_control_channels(void);
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void rudder_arm_disarm_check();
    void read_radio();
    int16_t rudder_input(void);
    void control_failsafe();
    bool trim_radio();
    bool rc_throttle_value_ok(void) const;
    bool rc_failsafe_active(void) const;
    void read_rangefinder(void);
    void read_airspeed(void);
    void rpm_update(void);
    void init_ardupilot();
    void startup_ground(void);
    bool set_mode(Mode& new_mode, const ModeReason reason);
    bool set_mode(const uint8_t mode, const ModeReason reason) override;
    bool set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason);
    Mode *mode_from_mode_num(const enum Mode::Number num);
    void check_long_failsafe();
    void check_short_failsafe();
    void startup_INS_ground(void);
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void landing_gear_update(void);
    void complete_auto_takeoff(void);
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif
    void update_optical_flow(void);
    void one_second_loop(void);
    void airspeed_ratio_update(void);
    void compass_save(void);
    void update_logging1(void);
    void update_logging2(void);
    void avoidance_adsb_update(void);
    void update_control_mode(void);
    void stabilize();
    void set_servos_idle(void);
    void set_servos();
    void set_servos_manual_passthrough(void);
    void set_servos_controlled(void);
    void set_servos_old_elevons(void);
    void set_servos_flaps(void);
    void set_landing_gear(void);
    void dspoiler_update(void);
    void servo_output_mixers(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void throttle_voltage_comp();
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    void update_flight_stage();
    void update_navigation();
    void set_flight_stage(AP_Vehicle::FixedWing::FlightStage fs);
    bool is_flying(void);
    float get_speed_scaler(void);
    bool stick_mixing_enabled(void);
    void stabilize_roll(float speed_scaler);
    void stabilize_pitch(float speed_scaler);
    void stabilize_stick_mixing_direct();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw(float speed_scaler);
    void stabilize_training(float speed_scaler);
    void stabilize_acro(float speed_scaler);
    void calc_nav_yaw_coordinated(float speed_scaler);
    void calc_nav_yaw_course(void);
    void calc_nav_yaw_ground(void);
    void throttle_slew_limit(SRV_Channel::Aux_servo_function_t func);
    bool suppress_throttle(void);
    void update_throttle_hover();
    void channel_function_mixer(SRV_Channel::Aux_servo_function_t func1_in, SRV_Channel::Aux_servo_function_t func2_in,
                                SRV_Channel::Aux_servo_function_t func1_out, SRV_Channel::Aux_servo_function_t func2_out);
    void flaperon_update(int8_t flap_percent);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
    void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_turns(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void notify_mode(const Mode& mode);
    void log_init();
    void parachute_check();
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
    void parachute_release();
    bool parachute_manual_release();
#endif
#if OSD_ENABLED == ENABLED
    void publish_osd_info();
#endif
    void accel_cal_update(void);
#if SOARING_ENABLED == ENABLED
    void update_soaring();
#endif

    bool reversed_throttle;
    bool have_reverse_throttle_rc_option;
    bool allow_reverse_thrust(void) const;
    bool have_reverse_thrust(void) const;
    int16_t get_throttle_input(bool no_deadzone=false) const;

    // support for AP_Avoidance custom flight mode, AVOID_ADSB
    // 支持AP防撞自定义飞行模式AVOID_ADSB
    bool avoid_adsb_init(bool ignore_checks);
    void avoid_adsb_run();

    enum Failsafe_Action {
        Failsafe_Action_None      = 0,
        Failsafe_Action_RTL       = 1,
        Failsafe_Action_Land      = 2,
        Failsafe_Action_Terminate = 3,
        Failsafe_Action_QLand     = 4,
        Failsafe_Action_Parachute = 5
    };

    // list of priorities, highest priority first
    // 优先级列表，最高优先级优先
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_Parachute,
                                                      Failsafe_Action_QLand,
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                      // 优先级列表的结尾必须为-1
                                                     };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    enum class ThrFailsafe {
        Disabled    = 0,
        Enabled     = 1,
        EnabledNoFS = 2
    };

public:
    void mavlink_delay_cb();
    void failsafe_check(void);
};

extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
