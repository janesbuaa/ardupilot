#pragma once

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library	姿态控制库
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_Fence/AC_Fence.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Proximity/AP_Proximity.h>
#include "qautotune.h"

/*
  QuadPlane specific functionality
  多旋翼固定翼特定功能
 */
class QuadPlane
{
public:
    friend class Plane;
    friend class AP_Tuning_Plane;
    friend class GCS_MAVLINK_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class QAutoTune;
    friend class AP_Arming_Plane;

    friend class Mode;
    friend class ModeAuto;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeQHover;
    friend class ModeQLand;
    friend class ModeQLoiter;
    friend class ModeQRTL;
    friend class ModeQStabilize;
    friend class ModeQAutotune;
    friend class ModeQAcro;
    
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    // 用于保存参数信息的var_info
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    void control_run(void);
    void control_auto(void);
    bool init_mode(void);
    bool setup(void);

    void vtol_position_controller(void);
    void setup_target_position(void);
    void takeoff_controller(void);
    void waypoint_controller(void);

    void update_throttle_mix(void);
    
    // update transition handling
    // 更新过渡处理
    void update(void);

    // set motor arming
    // 设置电动机加锁
    void set_armed(bool armed);

    // is VTOL available?
    // 是垂直起降可用？
    bool available(void) const {
        return initialised;
    }

    // is quadplane assisting?
    // 四旋翼辅助吗？
    bool in_assisted_flight(void) const {
        return available() && assisted_flight;
    }

    /*
      return true if we are in a transition to fwd flight from hover
      如果我们正在从悬停过渡到前飞，则返回true
    */
    bool in_transition(void) const;

    /*
      return true if we are a tailsitter transitioning to VTOL flight
      如果我们正过渡到VTOL飞行，则返回true
    */
    bool in_tailsitter_vtol_transition(void) const;
    
    bool handle_do_vtol_transition(enum MAV_VTOL_STATE state);

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(void);
    bool in_vtol_auto(void) const;
    bool in_vtol_mode(void) const;
    void update_throttle_hover();

    // vtol help for is_flying()
    // vtol帮助is_flying（）
    bool is_flying(void);

    // return current throttle as a percentate
    // 返回当前的油门百分比
    uint8_t throttle_percentage(void) const {
        return last_throttle * 100;
    }

    // return desired forward throttle percentage
    // 返回期望的油门百分比
    int8_t forward_throttle_pct(void);        
    float get_weathervane_yaw_rate_cds(void);

    // see if we are flying from vtol point of view
    // 看看我们是否从vtol的角度出发
    bool is_flying_vtol(void) const;

    // return true when tailsitter frame configured
    // 当配置了尾坐构型时返回true
    bool is_tailsitter(void) const;

    // return true when flying a tailsitter in VTOL
    // 尾坐构型在垂起下飞行时返回true
    bool tailsitter_active(void);
    
    // create outputs for tailsitters
    // 为尾坐构型创建输出
    void tailsitter_output(void);

    // handle different tailsitter input types
    // 处理不同的尾巴输入类型
    void tailsitter_check_input(void);
    
    // check if we have completed transition to fixed wing
    // 检查我们是否已完成向固定翼的过渡
    bool tailsitter_transition_fw_complete(void);

    // check if we have completed transition to vtol
    // 检查是否已完成向vtol的过渡
    bool tailsitter_transition_vtol_complete(void) const;

    // account for surface speed scaling in hover
    // 考虑悬停时的表面速度缩放
    void tailsitter_speed_scaling(void);
    
    // user initiated takeoff for guided mode
    // 用户使用导引模式起飞
    bool do_user_takeoff(float takeoff_altitude);

    // return true if the wp_nav controller is being updated
    // 如果正在更新wp_nav控制器，则返回true
    bool using_wp_nav(void) const;

    // return true if the user has set ENABLE
    // 如果用户设置了ENABLE，则返回true
    bool enabled(void) const { return enable != 0; }
    
    struct PACKED log_QControl_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float    throttle_in;
        float    angle_boost;
        float    throttle_out;
        float    throttle_hover;
        float    desired_alt;
        float    inav_alt;
        int32_t  baro_alt;
        int16_t  target_climb_rate;
        int16_t  climb_rate;
        float    throttle_mix;
    };

    MAV_TYPE get_mav_type(void) const;

private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AP_Int8 frame_class;
    AP_Int8 frame_type;
    
    AP_MotorsMulticopter *motors;
    const struct AP_Param::GroupInfo *motors_var_info;
    
    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Loiter *loiter_nav;
    
    // maximum vertical velocity the pilot may request
    // 飞行员可能要求的最大垂直速度
    AP_Int16 pilot_velocity_z_max;

    // vertical acceleration the pilot may request
    // 飞行员可能要求的垂直加速度
    AP_Int16 pilot_accel_z;

    // check for quadplane assistance needed
    // 检查四翼飞机所需的协助
    bool assistance_needed(float aspeed);

    // update transition handling
    // 更新过渡处理
    void update_transition(void);

    // check for an EKF yaw reset
    // 检查扩展卡尔曼滤波偏航重设
    void check_yaw_reset(void);
    
    // hold hover (for transition)
    //保持悬停（用于过渡）
    void hold_hover(float target_climb_rate);    

    // hold stabilize (for transition)
    //保持稳定（用于过渡）
    void hold_stabilize(float throttle_in);    

    // get pilot desired yaw rate in cd/s
    // 以cd/s为单位获取飞行员期望的偏航率
    float get_pilot_input_yaw_rate_cds(void) const;

    // get overall desired yaw rate in cd/s
    //以cd / s为单位获取整体所需的偏航率
    float get_desired_yaw_rate_cds(void);
    
    // get desired climb rate in cm/s
    //以cm / s为单位获取所需的爬升速度
    float get_pilot_desired_climb_rate_cms(void) const;

    // initialise throttle_wait when entering mode
    //进入模式时初始化油门等待
    void init_throttle_wait();

    // use multicopter rate controller
    //使用多旋翼速率控制器
    void multicopter_attitude_rate_update(float yaw_rate_cds);
    
    // main entry points for VTOL flight modes
    // VTOL飞行模式的主要入口点
    void init_stabilize(void);
    void control_stabilize(void);

    void check_attitude_relax(void);
    void init_qacro(void);
    float get_pilot_throttle(void);
    void control_qacro(void);
    void init_hover(void);
    void control_hover(void);

    void init_loiter(void);
    void init_qland(void);
    void control_loiter(void);
    void check_land_complete(void);
    bool land_detector(uint32_t timeout_ms);
    bool check_land_final(void);

    void init_qrtl(void);
    void control_qrtl(void);
    
    float assist_climb_rate_cms(void) const;

    // calculate desired yaw rate for assistance
    //计算所需的偏航率以获得帮助
    float desired_auto_yaw_rate_cds(void) const;

    bool should_relax(void);
    void motors_output(bool run_rate_controller = true);
    void Log_Write_QControl_Tuning();
    float landing_descent_rate_cms(float height_above_ground) const;
    
    // setup correct aux channels for frame class
    //为帧类别设置正确的辅助通道
    void setup_default_channels(uint8_t num_motors);

    void guided_start(void);
    void guided_update(void);

    void update_throttle_suppression(void);

    void run_z_controller(void);

    void setup_defaults(void);

    // calculate a stopping distance for fixed-wing to vtol transitions
    //计算固定翼到vtol过渡的停止距离
    float stopping_distance(void);
    
    AP_Int16 transition_time_ms;

    // transition deceleration, m/s/s
    //过渡减速度，m/s/s
    AP_Float transition_decel;

    // transition failure milliseconds
    //转换失败毫秒
    AP_Int16 transition_failure;

    // Quadplane trim, degrees
    // 旋翼飞机配平，度
    AP_Float ahrs_trim_pitch;
    float _last_ahrs_trim_pitch;

    // fw landing approach radius
    //降落进近半径
    AP_Float fw_land_approach_radius;

    AP_Int16 rc_speed;

    // min and max PWM for throttle
    //油门的最小和最大PWM
    AP_Int16 thr_min_pwm;
    AP_Int16 thr_max_pwm;

    // speed below which quad assistance is given
    // 低于该速度时将提供四重旋翼辅助
    AP_Float assist_speed;

    // angular error at which quad assistance is given
    // 提供四旋翼辅助的角度误差
    AP_Int8 assist_angle;
    uint32_t angle_error_start_ms;

    // altitude to trigger assistance
    // 触发旋翼辅助的高度
    AP_Int16 assist_alt;
    uint32_t alt_error_start_ms;
    bool in_alt_assist;

    // maximum yaw rate in degrees/second
    //最大横摆率，以度/秒为单位
    AP_Float yaw_rate_max;

    // landing speed in cm/s
    //着陆速度，单位：cm / s
    AP_Int16 land_speed_cms;

    // QRTL start altitude, meters
    // QRTL起始高度，米
    AP_Int16 qrtl_alt;
    
    // alt to switch to QLAND_FINAL
    // alt切换到QLAND_FINAL
    AP_Float land_final_alt;
    AP_Float vel_forward_alt_cutoff;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;

    // control if a VTOL RTL will be used
    // 控制是否使用VTOL 返航模式
    AP_Int8 rtl_mode;

    // control if a VTOL GUIDED will be used
    // 控制是否使用VTOL 引导模式
    AP_Int8 guided_mode;

    // control ESC throttle calibration
    // 控制ESC油门校准
    AP_Int8 esc_calibration;
    void run_esc_calibration(void);

    // ICEngine control on landing
    // 着陆时的ICEngine控制
    AP_Int8 land_icengine_cut;

    // HEARTBEAT mav_type override
    // 心跳mav_type覆盖
    AP_Int8 mav_type;

    // manual throttle curve expo strength
    // 手动油门曲线的曝光强度
    AP_Float throttle_expo;

    // QACRO mode max roll/pitch/yaw rates
    // QACRO模式的最大横摇/俯仰/偏航率
    AP_Float acro_roll_rate;
    AP_Float acro_pitch_rate;
    AP_Float acro_yaw_rate;

    // time we last got an EKF yaw reset
    //我们上一次获得EKF偏航的时间
    uint32_t ekfYawReset_ms;

    struct {
        AP_Float gain;
        float integrator;
        uint32_t last_ms;
        int8_t last_pct;
    } vel_forward;

    struct {
        AP_Float gain;
        AP_Float min_roll;
        uint32_t last_pilot_input_ms;
        float last_output;
    } weathervane;
    
    bool initialised;
    
    // timer start for transition
    // 过渡开始的时间
    uint32_t transition_start_ms;
    uint32_t transition_low_airspeed_ms;

    Location last_auto_target;

    // last throttle value when active
    // 活动时的最后一个油门值
    float last_throttle;

    // pitch when we enter loiter mode
    // 进入盘旋模式时的俯仰
    int32_t loiter_initial_pitch_cd;

    // when did we last run the attitude controller?
    // 我们上一次运行姿态控制器的时间？
    uint32_t last_att_control_ms;

    // true if we have reached the airspeed threshold for transition
    // 如果我们已经达到过渡的空速阈值，则为true
    enum {
        TRANSITION_AIRSPEED_WAIT,
        TRANSITION_TIMER,
        TRANSITION_ANGLE_WAIT_FW,
        TRANSITION_ANGLE_WAIT_VTOL,
        TRANSITION_DONE
    } transition_state;

    // true when waiting for pilot throttle
    // 等待飞行员油门时为true
    bool throttle_wait:1;

    // true when quad is assisting a fixed wing mode
    //当多旋翼正在辅助固定机翼模式时为true
    bool assisted_flight:1;

    // true when in angle assist
    // 角度辅助时为true
    bool in_angle_assist:1;

    // are we in a guided takeoff?
    // 我们处于引导式起飞状态吗？
    bool guided_takeoff:1;
    
    struct {
        // time when motors reached lower limit
    	//电机达到下限的时间
        uint32_t lower_limit_start_ms;
        uint32_t land_start_ms;
        float vpos_start_m;
    } landing_detect;

    // time we last set the loiter target
    //我们上次设定盘旋目标的时间
    uint32_t last_loiter_ms;

    enum position_control_state {
        QPOS_POSITION1,
        QPOS_POSITION2,
        QPOS_LAND_DESCEND,
        QPOS_LAND_FINAL,
        QPOS_LAND_COMPLETE
    };
    struct {
        enum position_control_state state;
        float speed_scale;
        Vector2f target_velocity;
        float max_speed;
        Vector3f target;
        bool slow_descent:1;
    } poscontrol;

    struct {
        bool running;
        uint32_t start_ms;            // system time the motor test began//电机测试开始的系统时间
        uint32_t timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
        							  //测试将在motor_test_start_ms之后的几毫秒内超时
        uint8_t seq = 0;              // motor sequence number of motor being tested//被测电机的电机序号
        uint8_t throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
        							  // 马达油门类型（0 =油门百分比，1 = PWM，2 =先导油门通道通过）
        uint16_t throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type
        							  // 要发送给电动机的节气门，其值取决于其类型
        uint8_t motor_count;          // number of motors to cycle//要循环的电机数
    } motor_test;

    // time of last control log message
    //最后的控制日志消息的时间
    uint32_t last_ctrl_log_ms;

    // time of last QTUN log message
    //最后的QTUN日志消息的时间
    uint32_t last_qtun_log_ms;

    // types of tilt mechanisms
    //倾转类型
    enum {TILT_TYPE_CONTINUOUS    =0,	//连续可控的
          TILT_TYPE_BINARY        =1,	//两档的
          TILT_TYPE_VECTORED_YAW  =2,	//矢量的
          TILT_TYPE_BICOPTER      =3	//双旋翼
    };

    // tiltrotor control variables
    //倾转旋翼控制变量
    struct {
        AP_Int16 tilt_mask;				//倾转电机编码，二进制
        AP_Int16 max_rate_up_dps;		//向上倾转速率
        AP_Int16 max_rate_down_dps;		//向下倾转速率
        AP_Int8  max_angle_deg;			//最大角度
        AP_Int8  tilt_type;				//倾转类型
        AP_Float tilt_yaw_angle;		//倾转偏航角
        float current_tilt;				//当前倾角
        float current_throttle;			//当前油门
        bool motors_active:1;
    } tilt;

    enum tailsitter_input {
        TAILSITTER_INPUT_MULTICOPTER = 0,
        TAILSITTER_INPUT_PLANE       = 1,
        TAILSITTER_INPUT_BF_ROLL_M   = 2,
        TAILSITTER_INPUT_BF_ROLL_P   = 3,
    };

    enum tailsitter_mask {
        TAILSITTER_MASK_AILERON  = 1,
        TAILSITTER_MASK_ELEVATOR = 2,
        TAILSITTER_MASK_THROTTLE = 4,
        TAILSITTER_MASK_RUDDER   = 8,
    };
    
    // tailsitter control variables
    //尾坐构型的控制变量
    struct {
        AP_Int8 transition_angle;
        AP_Int8 input_type;
        AP_Int8 input_mask;
        AP_Int8 input_mask_chan;
        AP_Float vectored_forward_gain;
        AP_Float vectored_hover_gain;
        AP_Float vectored_hover_power;
        AP_Float throttle_scale_max;
        AP_Float max_roll_angle;
        AP_Int16 motor_mask;
    } tailsitter;

    // the attitude view of the VTOL attitude controller
    // VTOL姿态控制器的姿态视图
    AP_AHRS_View *ahrs_view;

    // time when motors were last active
    //电机最后一次激活的时间
    uint32_t last_motors_active_ms;

    // time when we last ran the vertical accel controller
    //我们上次运行垂直加速控制器的时间
    uint32_t last_pidz_active_ms;
    uint32_t last_pidz_init_ms;

    // time when we were last in a vtol control mode
    //我们上一次处于vtol控制模式的时间
    uint32_t last_vtol_mode_ms;
    
    void tiltrotor_slew(float tilt);
    void tiltrotor_binary_slew(bool forward);
    void tiltrotor_update(void);
    void tiltrotor_continuous_update(void);
    void tiltrotor_binary_update(void);
    void tiltrotor_vectored_yaw(void);
    void tiltrotor_bicopter(void);
    void tilt_compensate_up(float *thrust, uint8_t num_motors);
    void tilt_compensate_down(float *thrust, uint8_t num_motors);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool is_motor_tilting(uint8_t motor) const {
        return (((uint8_t)tilt.tilt_mask.get()) & (1U<<motor));
    }
    bool tiltrotor_fully_fwd(void);
    float tilt_max_change(bool up);

    void afs_terminate(void);
    bool guided_mode_enabled(void);

    // set altitude target to current altitude
    //将海拔高度目标设置为当前海拔高度

    void set_alt_target_current(void);
    
    // adjust altitude target smoothly
    //平稳调整目标高度
    void adjust_alt_target(float target_cm);

    // additional options
    // 其他选项
    AP_Int32 options;
    enum {
        OPTION_LEVEL_TRANSITION=(1<<0),
        OPTION_ALLOW_FW_TAKEOFF=(1<<1),
        OPTION_ALLOW_FW_LAND=(1<<2),
        OPTION_RESPECT_TAKEOFF_FRAME=(1<<3),
        OPTION_MISSION_LAND_FW_APPROACH=(1<<4),
        OPTION_FS_QRTL=(1<<5),
    };

    AP_Float takeoff_failure_scalar;
    AP_Float maximum_takeoff_airspeed;
    uint32_t takeoff_start_time_ms;
    uint32_t takeoff_time_limit_ms;

    /*
      return true if current mission item is a vtol takeoff
      如果当前任务项目是vtol起飞，则返回true
     */
    bool is_vtol_takeoff(uint16_t id) const;

    /*
      return true if current mission item is a vtol landing
      如果当前任务项是vtol着陆，则返回true
     */
    bool is_vtol_land(uint16_t id) const;

#if QAUTOTUNE_ENABLED
    // qautotune mode
    // qautotune模式
    QAutoTune qautotune;
#endif

    /*
      are we in the approach phase of a VTOL landing?
      我们是否正处于VTOL着陆的进近阶段？
     */
    bool in_vtol_land_approach(void) const;

    /*
      are we in the descent phase of a VTOL landing?
      我们是否处于VTOL着陆的下降阶段？
     */
    bool in_vtol_land_descent(void) const;

    /*
      are we in the final landing phase of a VTOL landing?
      我们是否处于VTOL着陆的最终着陆阶段？
     */
    bool in_vtol_land_final(void) const;

    /*
      are we in any of the phases of a VTOL landing?
      我们处于VTOL着陆的任何阶段吗？
     */
    bool in_vtol_land_sequence(void) const;
    
public:
    void motor_test_output();
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                        uint16_t throttle_value, float timeout_sec,
                                        uint8_t motor_count);
private:
    void motor_test_stop();
};
