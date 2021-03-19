#include "Plane.h"

Mode::Mode()
{
}

void Mode::exit()
{
    // call sub-classes exit
    // 调用子类退出
    _exit();
}

bool Mode::enter()
{
    // cancel inverted flight
    // 取消倒飞
    plane.auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    // 开始任务时不要越过电子围栏
    plane.auto_state.next_wp_crosstrack = false;

    // reset landing check
    // 重置登陆检查
    plane.auto_state.checked_for_autoland = false;

    // zero locked course
    // 零锁定路线
    plane.steer_state.locked_course_err = 0;

    // reset crash detection
    // 重置坠机检测
    plane.crash_state.is_crashed = false;
    plane.crash_state.impact_detected = false;

    // reset external attitude guidance
    // 重置外部高度指导
    plane.guided_state.last_forced_rpy_ms.zero();
    plane.guided_state.last_forced_throttle_ms = 0;

#if CAMERA == ENABLED
    plane.camera.set_is_auto_mode(this == &plane.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    // 零初始俯仰和最高空速在模式改变时
    plane.auto_state.highest_airspeed = 0;
    plane.auto_state.initial_pitch_cd = plane.ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    // 在模式更改时禁用尾座起飞
    plane.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    // 从当前位置的上一个导航点开始
    plane.prev_WP_loc = plane.current_loc;

    // new mode means new loiter
    // 新模式意味着悬停
    plane.loiter.start_time_ms = 0;

    // record time of mode change
    // 记录模式更改时间
    plane.last_mode_change_ms = AP_HAL::millis();

    // assume non-VTOL mode
    // 假设为非垂起模式
    plane.auto_state.vtol_mode = false;
    plane.auto_state.vtol_loiter = false;

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags
        // 这些必须AFTER _enter()完成，因为它们使用结果来设置更多标

        // start with throttle suppressed in auto_throttle modes
        // 在自动油门模式下以抑制油门开始
        plane.throttle_suppressed = plane.auto_throttle_mode;

        plane.adsb.set_is_auto_mode(plane.auto_navigation_mode);

        // reset steering integrator on mode change
        // 在模式更改时重置转向积分器
        plane.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        // 更新遥控器故障保护，因为模式更改可能需要更改故障安全油门
        plane.control_failsafe();
    }

    return enter_result;
}

