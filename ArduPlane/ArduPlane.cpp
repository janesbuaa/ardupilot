/*
首席开发人员：Andrew Tridgell

   作者：道格.韦伯（Doug Weibel），何塞.朱利奥（Jose Julio），乔迪.穆尼兹（Jordi Munoz），杰森.肖特（Jason Short），
   兰迪.麦凯（Randy Mackay），帕特.希基（Pic Hickey），约翰.阿恩.伯克兰（John Arne Birkeland），奥利维尔.阿德勒（Alicar Lucas），
   格里高里.弗莱彻（Gregory Fletcher），保罗.里斯伯勒（Paul Riseborough），布兰登.琼斯（Bondon Jones），
   乔恩.查林格（Jon Challinger），汤姆.皮滕格（Tom Pittenger）
   感谢：克里斯.安德森（Chris Anderson），迈克尔.奥本（Michael Oborne），保罗.马瑟（Paul Mather），比尔.普雷梅拉尼（Bill Premerlani），
   詹姆斯.科恩（James Cohen），rotorFX的JB，Automatik，Fefenin，彼得.迈斯特（Peter Meister），雷姆比比，尤里.斯米尔诺夫（Yurry Smirnov），
   桑德罗.贝尼尼奥（Sandro Benigno），马克斯.莱文（Robert Levine），罗伯托.纳沃尼（Roberto Navoni），洛伦兹.迈耶（Lorenz Meier），
   尤里.蒙赞（Yury MonZon）

   请贡献您的想法！有关详细信息，请参见http://dev.ardupilot.org。

   该程序是免费软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款（许可证的版本3）或（根据您的选择）任何更高版本来重新分发和/或修改它。

   分发该程序是希望它会有用，但是没有任何保证；甚至没有对适销性或特定用途适用性的暗示保证。有关更多详细信息，请参见GNU通用公共许可证。

   您应该已经与该程序一起收到了GNU通用公共许可证的副本。如果不是，请参见<http://www.gnu.org/licenses/>。
 */

#include "Plane.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros)


/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
  调度程序表-此处列出了所有常规任务，以及应调用它们的频率（以Hz为单位）以及预计将花费的最长时间（以微秒为单位）
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    SCHED_TASK(ahrs_update,           400,    400),
    SCHED_TASK(read_radio,             50,    100),
    SCHED_TASK(check_short_failsafe,   50,    100),
    SCHED_TASK(update_speed_height,    50,    200),
    SCHED_TASK(update_control_mode,   400,    100),
    SCHED_TASK(stabilize,             400,    100),
    SCHED_TASK(set_servos,            400,    100),
    SCHED_TASK(update_throttle_hover, 100,     90),
    SCHED_TASK(read_control_switch,     7,    100),
    SCHED_TASK(update_GPS_50Hz,        50,    300),
    SCHED_TASK(update_GPS_10Hz,        10,    400),
    SCHED_TASK(navigate,               10,    150),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(read_airspeed,          10,    100),
    SCHED_TASK(update_alt,             10,    200),
    SCHED_TASK(adjust_altitude_target, 10,    200),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    100),
#endif
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750),
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events,          50,  150),
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read, 10, 300),
    SCHED_TASK_CLASS(AP_Baro, &plane.barometer, accumulate, 50, 150),
    SCHED_TASK_CLASS(AP_Notify,      &plane.notify,  update, 50, 300),
    SCHED_TASK(read_rangefinder,       50,    100),
    SCHED_TASK_CLASS(AP_ICEngine, &plane.g2.ice_control, update, 10, 100),
    SCHED_TASK_CLASS(Compass,          &plane.compass,              cal_update, 50, 50),
    SCHED_TASK(accel_cal_update,       10,    50),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow, &plane.optflow, update,    50,    50),
#endif
    SCHED_TASK(one_second_loop,         1,    400),
    SCHED_TASK(check_long_failsafe,     3,    400),
    SCHED_TASK(rpm_update,             10,    100),
    SCHED_TASK(airspeed_ratio_update,   1,    100),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100),
#endif // MOUNT == ENABLED
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update_trigger, 50, 100),
#endif // CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100),
    SCHED_TASK(compass_save,          0.1,    200),
    SCHED_TASK(Log_Write_Fast,         25,    300),
    SCHED_TASK(update_logging1,        25,    300),
    SCHED_TASK(update_logging2,        25,    300),
#if SOARING_ENABLED == ENABLED
    SCHED_TASK(update_soaring,         50,    400),
#endif
    SCHED_TASK(parachute_check,        10,    200),
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200),
#endif // AP_TERRAIN_AVAILABLE	可提供AP地形
    SCHED_TASK(update_is_flying_5Hz,    5,    100),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger, &plane.logger, periodic_tasks, 50, 400),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins, periodic, 50, 50),
    SCHED_TASK(avoidance_adsb_update,  10,    100),
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200),
    SCHED_TASK_CLASS(AP_Button, &plane.g2.button, update, 5, 100),
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats, &plane.g2.stats, update, 1, 100),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper, &plane.g2.gripper, update, 10, 75),
#endif
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info, 1, 10),
#endif
#if LANDING_GEAR_ENABLED == ENABLED
    SCHED_TASK(landing_gear_update, 5, 50),
#endif
};

constexpr int8_t Plane::_failsafe_priorities[7];

void Plane::setup() 
{
    // load the default values of variables listed in var_info[]
	// 加载var_info []中列出的变量的默认值
    AP_Param::setup_sketch_defaults();

    rssi.init();

    init_ardupilot();

    // initialise the main loop scheduler
    // 初始化主循环调度程序
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

void Plane::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
}

// update AHRS system
// 更新AHRS系统
void Plane::ahrs_update()
{
    arming.update_soft_armed();

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // update hil before AHRS update
    	// 在AHRS更新之前更新hil
        gcs().update_receive();
    }
#endif

    ahrs.update();

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
    }

    // calculate a scaled roll limit based on current pitch
    // 根据当前螺距计算缩放的滚动限制
    roll_limit_cd = aparm.roll_limit_cd;
    pitch_limit_min_cd = aparm.pitch_limit_min_cd;

    if (!quadplane.tailsitter_active()) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min_cd *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    // 更新了用于地面转向和自动起飞的总陀螺仪。 DCM.c与陀螺矢量的点积给出地球框架偏航率
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

    // check if we have had a yaw reset from the EKF
    // 检查我们是否已经从EKF进行了偏航复位
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    // 更新惯性导航为四翼飞机
    quadplane.inertial_nav.update();
}

/*
  update 50Hz speed/height controller
  更新50Hz速度/高度控制器
 */
void Plane::update_speed_height(void)
{
    if (auto_throttle_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
    	// 调用TECS 50Hz更新。 请注意，无论油门是否被抑制，我们都将其称为“起飞”，因为它需要运行以进行起飞检测
        SpdHgt_Controller->update_50hz();
    }

    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
}


/*
  read and update compass
  读取和更新指南针
 */
void Plane::update_compass(void)
{
    if (AP::compass().enabled() && compass.read()) {
        ahrs.set_compass(&compass);
    }
}

/*
  do 10Hz logging
  进行10Hz记录
 */
void Plane::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        logger.Write_IMU();

    if (should_log(MASK_LOG_ATTITUDE_MED))
        logger.Write_AOA_SSA(ahrs);
}

/*
  do 10Hz logging - part2
  进行10Hz记录
 */
void Plane::update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        logger.Write_Vibration();
}


/*
  check for AFS failsafe check
  检查AFS故障安全检查
 */
#if ADVANCED_FAILSAFE == ENABLED
void Plane::afs_fs_check(void)
{
    // perform AFS failsafe checks
	// 执行AFS故障安全检查
    afs.check(failsafe.last_heartbeat_ms, geofence_breached(), failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
	// 使在运行时更改控制通道顺序成为可能
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

    // make it possible to change orientation at runtime
    // 使在运行时更改方向成为可能
    ahrs.update_orientation();

    adsb.set_stall_speed_cm(aparm.airspeed_min);
    adsb.set_max_speed(aparm.airspeed_max);

    // sync MAVLink system ID
    // 同步MAVLink系统ID
    mavlink_system.sysid = g.sysid_this_mav;

    SRV_Channels::enable_aux_servos();

    // update notify flags
    // 更新通知标志
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    // 如果未加锁且GPS位置已更改，请更新原始位置。 最多每5s更新一次
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            // 重置着陆高度校正
            landing.alt_offset = 0;
    }
}

void Plane::compass_save()
{
    if (AP::compass().enabled() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
          仅在解锁时保存补偿
         */
        compass.save_offsets();
    }
}

/*
  once a second update the airspeed calibration ratio
  每秒更新一次空速校准率
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
    	// 不动时请勿校准
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
    	// 低于最低空速飞行时请勿校准。 我们同时检查空速和地面速度，以发现空速比过低的情况，这可能导致空速比不再上升
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
    	// 超出正常飞行范围时请勿校准
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}


/*
  read the GPS and update position
  读取GPS并更新位置
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    // get position from AHRS
    // 从AHRS获得位置
    have_position = ahrs.get_position(current_loc);
    ahrs.get_relative_position_D_home(relative_altitude);
    relative_altitude *= -1.0f;
}

/*
  read update GPS position - 10Hz update
  读取更新GPS位置-10Hz更新
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // 我们倒数N个良好的GPS定位，以使高度更准确
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                	// 默默地忽略失败...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        // 看看我们是否违反了地理围栏
        geofence_check(false);

#if CAMERA == ENABLED
        camera.update();
#endif

        // update wind estimate
        // 更新风估计
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
    	// 丢失了3D修正，重新开始
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main control mode dependent update code
  主控制模式相关的更新代码
 */
void Plane::update_control_mode(void)
{
    Mode *effective_mode = control_mode;
    if (control_mode == &mode_auto && g.auto_fbw_steer == 42) {
        effective_mode = &mode_fbwa;
    }

    if (effective_mode != &mode_auto) {
        // hold_course is only used in takeoff and landing
    	// 保持航线仅用于起飞和降落
        steer_state.hold_course_cd = -1;
    }

    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
    // 确保当我们作为纯固定翼飞机飞行时能够向前飞行。 这可以帮助EKF做出更好的状态估计，因为它可以做出更强的假设
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        ahrs.set_fly_forward(false);
    } else if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
    } else {
        ahrs.set_fly_forward(true);
    }

    effective_mode->update();
}

void Plane::update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // 导航点距离以米为单位，而不是我们从GPS获得的* 100米

    uint16_t radius = 0;
    uint16_t qrtl_radius = abs(g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(aparm.loiter_radius);
    }
    
    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        if (ahrs.home_is_set()) {
            mission.update();
        }
        break;
            
    case Mode::Number::RTL:
        if (quadplane.available() && quadplane.rtl_mode == 1 &&
            (nav_controller->reached_loiter_target() ||
             current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc) ||
             auto_state.wp_distance < MAX(qrtl_radius, quadplane.stopping_distance())) &&
            AP_HAL::millis() - last_mode_change_ms > 1000) {
            /*
              for a quadplane in RTL mode we switch to QRTL when we
              are within the maximum of the stopping distance and the
              RTL_RADIUS
              对于处于RTL模式的四翼飞机，当我们在停止距离和RTL_RADIUS的最大值内时，我们会切换到QRTL
             */
            set_mode(mode_qrtl, ModeReason::UNKNOWN);
            break;
        } else if (g.rtl_autoland == 1 &&
            !auto_state.checked_for_autoland &&
            reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
        	// 我们已经到达RTL点，看看我们是否有降落顺序
            if (mission.jump_to_landing_sequence()) {
                // switch from RTL -> AUTO
            	// 从RTL切换成AUTO
                mission.set_force_resume(true);
                set_mode(mode_auto, ModeReason::UNKNOWN);
            }

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            // 防止在每个循环上运行昂贵的跳转到着陆程序
            auto_state.checked_for_autoland = true;
        }
        else if (g.rtl_autoland == 2 &&
            !auto_state.checked_for_autoland) {
            // Go directly to the landing sequence
        	// 直接进入着陆顺序
            if (mission.jump_to_landing_sequence()) {
                // switch from RTL -> AUTO
            	// 从RTL切换成AUTO
                mission.set_force_resume(true);
                set_mode(mode_auto, ModeReason::UNKNOWN);
            }

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            // 防止在每个循环上运行昂贵的跳转到着陆程序
            auto_state.checked_for_autoland = true;
        }
        radius = abs(g.rtl_radius);
        if (radius > 0) {
            loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        }
        // fall through to LOITER
        // 陷入困境
        FALLTHROUGH;

    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::TAKEOFF:
        update_loiter(radius);
        break;

    case Mode::Number::CRUISE:
        update_cruise();
        break;

    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::TRAINING:
    case Mode::Number::INITIALISING:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CIRCLE:
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::QAUTOTUNE:
    case Mode::Number::QACRO:
        // nothing to do
        break;
    }
}

/*
  set the flight stage
  设定飞行阶段
 */
void Plane::set_flight_stage(AP_Vehicle::FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_Vehicle::FixedWing::FLIGHT_LAND);

    if (fs == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
    Log_Write_Status();
}

void Plane::update_alt()
{
    barometer.update();

    if (quadplane.available()) {
        quadplane.motors->set_air_density_ratio(barometer.get_air_density_ratio());
    }

    // calculate the sink rate.
    // 计算下沉率。
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    // 下沉率使用低通滤波器消除一些噪声
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if PARACHUTE == ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif
    geofence_check(true);

    update_flight_stage();

    if (auto_throttle_mode && !throttle_suppressed) {        

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND && current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        bool soaring_active = false;
#if SOARING_ENABLED == ENABLED
        if (g2.soaring_controller.is_active() && g2.soaring_controller.get_throttle_suppressed()) {
            soaring_active = true;
        }
#endif
        
        float target_alt = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && g2.rtl_climb_min > 0) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
        	// 确保我们在RTL中进行了初始爬坡。 我们在要求的高度上增加了10m，以推动TECS快速爬升
            target_alt = MAX(target_alt, prev_WP_loc.alt + (g2.rtl_climb_min+10)*100);
        }

        SpdHgt_Controller->update_pitch_throttle(target_alt,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor,
                                                 soaring_active);
    }
}

/*
  recalculate the flight_stage
  重新计算飞行阶段
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
	// 更新速度和高度控制器状态
    if (auto_throttle_mode && !throttle_suppressed) {        
        if (control_mode == &mode_auto) {
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
            } else if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                	// 中止模式为粘滞模式，必须在执行NAV LAND时完成
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND);
                } else {
                    set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_LAND);
                }
            } else if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
            } else {
                set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
            }
        } else if (control_mode != &mode_takeoff) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
        	// 如果不在AUTO模式中，则假定正常运行以进行正常的TECS操作。 如果在着陆，中止着陆或起飞期间从AUTO切换为FBWB，这可以防止TECS停留在错误的阶段。
            set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
        }
    } else if (quadplane.in_vtol_mode() ||
               quadplane.in_assisted_flight()) {
        set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_VTOL);
    } else {
        set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_NORMAL);
    }
}




/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires

    only called from AP_Landing, when the landing library is ready to disarm
    如果启用了陆地解锁延迟（非零），请检查着陆情况，然后在时间到期后自动解锁
	仅在着陆库准备解锁时才从AP登陆调用
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed
         我们启用了自动解锁。 查看是否已经过去了足够的时间 */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm()) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}



/*
  the height above field elevation that we pass to TECS
  我们传递给TECS的场高以上的高度
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
      在着陆时将高于地面标高的高度作为地面上方的高度传递，这意味着TECS可以获取测距仪信息，从而可以知道耀斑何时来临。
    */
    float hgt_afe;
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        hgt_afe = height_above_target();
        hgt_afe -= rangefinder_correction();
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
    	// 在正常飞行中，我们将hgt_afe作为相对高度返航
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

#if OSD_ENABLED == ENABLED
void Plane::publish_osd_info()
{
    AP_OSD::NavInfo nav_info;
    nav_info.wp_distance = auto_state.wp_distance;
    nav_info.wp_bearing = nav_controller->target_bearing_cd();
    nav_info.wp_xtrack_error = nav_controller->crosstrack_error();
    nav_info.wp_number = mission.get_current_nav_index();
    osd.set_nav_info(nav_info);
}
#endif

AP_HAL_MAIN_CALLBACKS(&plane);
