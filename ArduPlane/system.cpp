#include "Plane.h"
#include <AP_Common/AP_FWVersion.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*        init_ardupilot函数处理空中重启所需的一切，我们稍后将确定我们是否真正在地面上并在这种情况下进行地面启动。
*
*****************************************************************************/

static void mavlink_delay_cb_static()
{
    plane.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    plane.failsafe_check();
}

void Plane::init_ardupilot()
{
    // initialise serial port
    // 初始化串口
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    // 从EEPROM加载任何参数之前，请检查EEPROM格式版本
    load_parameters();

#if STATS_ENABLED == ENABLED
    // initialise stats module
    // 初始化统计模块
    g2.stats.init();
#endif

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // set sensors to HIL mode
        // 将传感器设置为HIL模式
        ins.set_hil_mode();
        compass.set_hil_mode();
        barometer.set_hil_mode();
    }
#endif

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    set_control_channels();

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    // 注册mavlink_delay_cb，它将在您对hal.scheduler-> delay的调用中剩余超过5毫秒的任何时间运行
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // setup any board specific drivers
    // 设置任何主板专用的驱动程序
    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // initialise rc channels including setting mode
    // 初始化遥控器通道，包括设置模式
    rc().init();

    relay.init();

    // initialise notify system
    // 初始化通知系统
    notify.init();
    notify_mode(*control_mode);

    init_rc_out_main();
    
    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    // 记录发生了多少次重置。 这可用于检测飞行中的重置
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro
    // 初始气压计
    barometer.init();

    // initialise rangefinder
    // 初始化测距仪
    rangefinder.set_log_rfnd_bit(MASK_LOG_SONAR);
    rangefinder.init(ROTATION_PITCH_270);

    // initialise battery monitoring
    // 初始化电池监控
    battery.init();

    rpm_sensor.init();

    // setup telem slots with serial ports
    // 使用串口设置数传
    gcs().setup_uarts();

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    // 初始化空速传感器
    airspeed.init();

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if OPTFLOW == ENABLED
    // make optflow available to libraries
    // 使光流可做库调用
    if (optflow.enabled()) {
        ahrs.set_optflow(&optflow);
    }
#endif

    // give AHRS the airspeed sensor
    // 给AHRS空速传感器
    ahrs.set_airspeed(&airspeed);

    // GPS Initialization
    // GPS初始化
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    init_rc_in();               // sets up rc channels from radio    从无线电设置遥控频道

#if MOUNT == ENABLED
    // initialise camera mount
    // 初始化相机支架
    camera_mount.init();
#endif

#if LANDING_GEAR_ENABLED == ENABLED
    // initialise landing gear position
    // 初始化起落架位置
    g2.landing_gear.init();
#endif

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     *  设置“主循环已死”检查。 请注意，这依赖于正在初始化的RC库。
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    quadplane.setup();

    AP_Param::reload_defaults_file(true);
    
    startup_ground();

    // don't initialise aux rc output until after quadplane is setup as
    // that can change initial values of channels
    // 在设置飞机后，才初始化辅助遥控器输出，因为那样会改变通道的初始值
    init_rc_out_aux();
    
    // choose the nav controller
    // 选择导航控制器
    set_nav_controller();

    set_mode_by_number((enum Mode::Number)g.initial_mode.get(), ModeReason::UNKNOWN);

    // set the correct flight mode
    // 设置正确的飞行模式
    // ---------------------------
    reset_control_switch();

    // initialise sensor
    // 初始化传感器
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        optflow.init(-1);
    }
#endif

// init cargo gripper
// 初始货物夹爪
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

    // call AP_Vehicle setup code
    // 致电AP车辆设置代码
    vehicle_setup();

    // disable safety if requested
    // 根据要求禁用安全性
    BoardConfig.init_safety();

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//此功能可进行地面启动过程中需要的所有校准等工作。
//********************************************************************************
void Plane::startup_ground(void)
{
    set_mode(mode_initializing, ModeReason::UNKNOWN);

#if (GROUND_START_DELAY > 0)
    gcs().send_text(MAV_SEVERITY_NOTICE,"Ground start with delay");
    delay(GROUND_START_DELAY * 1000);
#else
    gcs().send_text(MAV_SEVERITY_INFO,"Ground start");
#endif

    //INS ground start
    //INS地面启动
    //------------------------
    //
    startup_INS_ground();

    // Save the settings for in-air restart
    // 保存设置以进行空中重启
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // initialise AP_Logger library
    // 初始化任务库
#if LOGGING_ENABLED == ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

#ifdef ENABLE_SCRIPTING
    g2.scripting.init();
#endif // ENABLE_SCRIPTING

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    // 重置上一个心跳时间，所以在启动缓慢时我们不会触发故障保护
    failsafe.last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    // 我们不想写入串口以使我们在飞行中暂停，因此请在准备好飞行后将串行端口设置为非阻塞
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO,"Ground start complete");
}


bool Plane::set_mode(Mode &new_mode, const ModeReason reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        // 如果我们已经处于正确的模式，请不要切换模式。
        return true;
    }

#if !QAUTOTUNE_ENABLED
    if (&new_mode == &plane.mode_qautotune) {
        gcs().send_text(MAV_SEVERITY_INFO,"QAUTOTUNE disabled");
        set_mode(plane.mode_qhover, ModeReason::UNAVAILABLE);
        return false;
    }
#endif

    // backup current control_mode and previous_mode
    // 备份当前控制模式和先前模式
    Mode &old_previous_mode = *previous_mode;
    Mode &old_mode = *control_mode;
    const ModeReason previous_mode_reason_backup = previous_mode_reason;

    // update control_mode assuming success
    // 假设成功更新控制模式
    // TODO: move these to be after enter() once start_command_callback() no longer checks control_mode
    // 一旦start_command_callback（）不再检查控制模式，请将它们移到enter（）之后
    previous_mode = control_mode;
    control_mode = &new_mode;
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

    // attempt to enter new mode
    // 尝试进入新模式
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        // 记录我们未能进入所需飞行模式的错误
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");

        // we failed entering new mode, roll back to old
        // 我们无法进入新模式，请回滚到旧模式
        previous_mode = &old_previous_mode;
        control_mode = &old_mode;

        control_mode_reason = previous_mode_reason;
        previous_mode_reason = previous_mode_reason_backup;

        // currently, only Q modes can fail enter(). This will likely change in the future and all modes
        // should be changed to check dependencies and fail early before depending on changes in Mode::set_mode()
        // 当前，只有Q模式可以使enter（）失败。 将来这可能会改变，并且应根据Mode :: set_mode（）的更改，应更改所有模式以检查依赖关系并尽早失败。
        if (control_mode->is_vtol_mode()) {
            // ignore result because if we fail we risk looping at the qautotune check above
            // 忽略结果，因为如果失败，我们可能会在上面的qautotune检查中循环
            control_mode->enter();
        }
        return false;
    }

    if (previous_mode == &mode_autotune) {
        // restore last gains
        // 恢复最后的增益
        autotune_restore();
    }

    // exit previous mode
    // 退出上一个模式
    old_mode.exit();

    // record reasons
    // 记录原因
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

    // log and notify mode change
    // 记录并通知模式更改
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    notify_mode(*control_mode);
    gcs().send_message(MSG_HEARTBEAT);

    return true;
}

bool Plane::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    Mode *mode = plane.mode_from_mode_num(static_cast<Mode::Number>(new_mode));
    if (mode == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Error: invalid mode number: %u", (unsigned)new_mode);
        return false;
    }
    return set_mode(*mode, reason);
}

bool Plane::set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason)
{
    Mode *new_mode = plane.mode_from_mode_num(new_mode_number);
    if (new_mode == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Error: invalid mode number: %d", new_mode_number);
        return false;
    }
    return set_mode(*new_mode, reason);
}

void Plane::check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if (failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        uint32_t radio_timeout_ms = failsafe.last_valid_rc_ms;
        if (failsafe.state == FAILSAFE_SHORT) {
            // time is relative to when short failsafe enabled
            radio_timeout_ms = failsafe.short_timer_ms;
        }
        if (failsafe.rc_failsafe &&
            (tnow - radio_timeout_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_LONG, ModeReason::RADIO_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == &mode_auto &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs().chan(0) != nullptr &&
                   gcs().chan(0)->last_radio_status_remrssi_ms != 0 &&
                   (tnow - gcs().chan(0)->last_radio_status_remrssi_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        }
    } else {
        uint32_t timeout_seconds = g.fs_timeout_long;
        if (g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
            // avoid dropping back into short timeout
            timeout_seconds = g.fs_timeout_short;
        }
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - failsafe.last_heartbeat_ms) < timeout_seconds*1000) {
            failsafe_long_off_event(ModeReason::GCS_FAILSAFE);
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.rc_failsafe) {
            failsafe_long_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}

void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if (g.fs_action_short != FS_ACTION_SHORT_DISABLED &&
       failsafe.state == FAILSAFE_NONE &&
       flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // The condition is checked and the flag rc_failsafe is set in radio.cpp
        if(failsafe.rc_failsafe) {
            failsafe_short_on_event(FAILSAFE_SHORT, ModeReason::RADIO_FAILSAFE);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.rc_failsafe || g.fs_action_short == FS_ACTION_SHORT_DISABLED) {
            failsafe_short_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}


void Plane::startup_INS_ground(void)
{
#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        while (barometer.get_last_update() == 0) {
            // the barometer begins updating when we get the first
            // HIL_STATE message
            gcs().send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
            hal.scheduler->delay(1000);
        }
    }
#endif

    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
    } else {
        gcs().send_text(MAV_SEVERITY_ALERT, "Skipping INS calibration");
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        airspeed.calibrate(true);
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING,"No airspeed");
    }
}

// sets notify object flight mode information
void Plane::notify_mode(const Mode& mode)
{
    notify.flags.flight_mode = mode.mode_number();
    notify.set_flight_mode_str(mode.name4());
}

/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    return logger.should_log(mask);
#else
    return false;
#endif
}

/*
  return throttle percentage from 0 to 100 for normal use and -100 to 100 when using reverse thrust
 */
int8_t Plane::throttle_percentage(void)
{
    if (quadplane.in_vtol_mode()) {
        return quadplane.throttle_percentage();
    }
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!have_reverse_thrust()) {
        return constrain_int16(throttle, 0, 100);
    }
    return constrain_int16(throttle, -100, 100);
}
