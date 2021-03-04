#include "Plane.h"

/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 *  我们的故障安全策略是检测主回路锁定并切换为直接将输入从RC输入传递到RC输出。
 */

/*
 *  this failsafe_check function is called from the core timer interrupt
 *  at 1kHz.
 *  从内核定时器中断以1kHz调用此failsafe_check函数。
 */
void Plane::failsafe_check(void)
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        // 主循环正在运行，一切正常
        last_ticks = ticks;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        // 自主循环运行以来，我们至少运行了0.2秒。 这意味着我们有麻烦，或者正在初始化例程或日志擦除中。 开始将遥控器输入传递到输出 
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {

        // ensure we have the latest RC inputs
        // 确保我们拥有最新的遥控器输入
        rc().read_input();

        last_timestamp = tnow;

        rc().read_input();

#if ADVANCED_FAILSAFE == ENABLED
        if (in_calibration) {
            // tell the failsafe system that we are calibrating
            // sensors, so don't trigger failsafe
            // 告诉故障保护系统我们正在校准传感器，所以不要触发故障安全
            afs.heartbeat();
        }
#endif

        if (RC_Channels::get_valid_channel_count() < 5) {
            // we don't have any RC input to pass through
            // 我们没有任何遥控器输入要传递
            return;
        }

        // pass RC inputs to outputs every 20ms
        // 每20ms将遥控器输入传递给输出
        RC_Channels::clear_overrides();

        int16_t roll = channel_roll->get_control_in_zero_dz();
        int16_t pitch = channel_pitch->get_control_in_zero_dz();
        int16_t throttle = get_throttle_input(true);
        int16_t rudder = channel_rudder->get_control_in_zero_dz();

        if (!hal.util->get_soft_armed()) {
            throttle = 0;
        }
        
        // setup secondary output channels that don't have
        // corresponding input channels
        // 设置没有相应输入通道的辅助输出通道
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        // 这是为了允许故障保护模块故意使飞机坠毁。 仅在满足OBC规则的极端情况下使用
#if ADVANCED_FAILSAFE == ENABLED
        if (afs.should_crash_vehicle()) {
            afs.terminate_vehicle();
            if (!afs.terminating_vehicle_via_landing()) {
                return;
            }
        }
#endif

        // setup secondary output channels that do have
        // corresponding input channels
        // 设置具有相应输入通道的辅助输出通道
        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 0);

        // setup flaperons
        // 设置襟副翼
        flaperon_update(0);

        servos_output();
    }
}
