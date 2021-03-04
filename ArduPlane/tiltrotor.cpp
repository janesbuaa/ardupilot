#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
  倾转旋翼和俯仰机翼的控制代码。 通过将Q_TILT_MASK设置为非零值来启用
 */


/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
  以0到1的倾斜度比例计算最大倾斜度变化
 */
float QuadPlane::tilt_max_change(bool up)
{
    float rate;
    if (up || tilt.max_rate_down_dps <= 0) {
        rate = tilt.max_rate_up_dps;
    } else {
        rate = tilt.max_rate_down_dps;
    }
    if (tilt.tilt_type != TILT_TYPE_BINARY && !up) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (hal.util->get_soft_armed() && !in_vtol_mode() && !assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            // 允许至少90 DPS的手动操作，或者如果我们不稳定，则可以进行快速控制
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt / 90.0f;
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
  输出一个回转受限的rotorrotor角度。倾斜度是从0到1
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<tilt.current_tilt);
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    // 转换为0..1000范围并输出
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);
}

/*
  update motor tilt for continuous tilt servos
  更新电机倾斜度以实现连续倾斜伺服
 */
void QuadPlane::tiltrotor_continuous_update(void)
{
    // default to inactive
	// 默认为不活动
    tilt.motors_active = false;

    // the maximum rate of throttle change
    // 最大油门变化率
    float max_change;
    
    if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
    	// 我们处于纯固定翼模式。 可倾斜电机一直向前移动，并将其作为前进电机运行
        tiltrotor_slew(1);

        max_change = tilt_max_change(false);
        
        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (tilt.current_tilt < 1) {
            tilt.current_throttle = constrain_float(new_throttle,
                                                    tilt.current_throttle-max_change,
                                                    tilt.current_throttle+max_change);
        } else {
            tilt.current_throttle = new_throttle;
        }
        if (!hal.util->get_soft_armed()) {
            tilt.current_throttle = 0;
        } else {
            // the motors are all the way forward, start using them for fwd thrust
        	// 电机一直向前，开始将其用于正向推力
            uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            motors->output_motor_mask(tilt.current_throttle, mask, plane.rudder_dt);
            // prevent motor shutdown
            // 防止电机停机
            tilt.motors_active = true;
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    // 记住我们用于VTOL飞行的油门水平
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<tilt.current_throttle);
    tilt.current_throttle = constrain_float(motors_throttle,
                                            tilt.current_throttle-max_change,
                                            tilt.current_throttle+max_change);
    
    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 3 strategies we will use:

      1) in QSTABILIZE or QHOVER the angle will be set to zero. This
         enables these modes to be used as a safe recovery mode.

      2) in fixed wing assisted flight or velocity controlled modes we
         will set the angle based on the demanded forward throttle,
         with a maximum tilt given by Q_TILT_MAX. This relies on
         Q_VFWD_GAIN being set

      3) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
      我们处于VTOL模式。 我们需要弄清楚需要多少倾斜度。我们将使用3种策略：
      1）在QSTABILIZE或QHOVER中，角度将设置为零。这样可以将这些模式用作安全恢复模式。
      2）在固定翼辅助飞行或速度控制模式下，我们将根据所需的前向油门设置角度，最大倾斜度由Q_TILT_MAX给出。 这取决于Q_VFWD_GAIN的设置
      3）如果我们处于TRANSITION_TIMER模式，那么我们正在过渡到向前飞行，应该将旋翼一直向前
    */
    if (plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qautotune) {
        tiltrotor_slew(0);
        return;
    }

    if (assisted_flight &&
        transition_state >= TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
    	// 我们正在过渡到固定机翼-将电动机一直向前倾斜
        tiltrotor_slew(1);
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
    	// 在完成过渡之前，我们将倾斜度限制为Q_TILT_MAX。 超过50％的推力只能获得Q_TILT_MAX。
    	// 低于50％的油门，我们会线性下降。 这依赖于正确设置Q_VFWD_GAIN。
        float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);
        tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
  输出一个回转受限的rotorrotor角度。倾斜度为0或1
 */
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
	// 伺服输出为二进制，不受摆率限制
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    // 限速current_tilt的作用是延迟tiltrotor_binary_update中的油门
    float max_change = tilt_max_change(!forward);
    if (forward) {
        tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
    } else {
        tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
  更新二进制倾斜伺服器的电动机倾斜
 */
void QuadPlane::tiltrotor_binary_update(void)
{
    // motors always active
	// 电机始终处于活动状态
    tilt.motors_active = true;

    if (!in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
    	// 我们处于纯固定翼模式。移动可倾斜的电机一路向前，并将它们作为前进马达运行
        tiltrotor_binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (tilt.current_tilt >= 1) {
            uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            // 电机一直向前，开始将其用于正向推力
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        tiltrotor_binary_slew(false);
    }
}


/*
  update motor tilt
  更新电机倾斜
 */
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
    	// 没有电机倾斜
        return;
    }

    if (tilt.tilt_type == TILT_TYPE_BINARY) {
        tiltrotor_binary_update();
    } else {
        tiltrotor_continuous_update();
    }

    if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        tiltrotor_vectored_yaw();
    }
}


/*
  compensate for tilt in a set of motor outputs

  Compensation is of two forms. The first is to apply _tilt_factor,
  which is a compensation for the reduces vertical thrust when
  tilted. This is supplied by set_motor_tilt_factor().

  The second compensation is to use equal thrust on all tilted motors
  when _tilt_equal_thrust is true. This is used when the motors are
  tilted by a large angle to prevent the roll and yaw controllers from
  causing instability. Typically this would be used when the motors
  are tilted beyond 45 degrees. At this angle it is assumed that roll
  control can be achieved using fixed wing control surfaces and yaw
  control with the remaining multicopter motors (eg. tricopter tail).

  By applying _tilt_equal_thrust the tilted motors effectively become
  a single pitch control motor.

  Note that we use a different strategy for when we are transitioning
  into VTOL as compared to from VTOL flight. The reason for that is
  we want to lean towards higher tilted motor throttle when
  transitioning to fixed wing flight, in order to gain airspeed,
  whereas when transitioning to VTOL flight we want to lean to towards
  lower fwd throttle. So we raise the throttle on the tilted motors
  when transitioning to fixed wing, and lower throttle on tilted
  motors when transitioning to VTOL
	补偿一组电机输出中的倾斜
	补偿有两种形式。第一种是应用_tilt_factor，它是倾斜时减小的垂直推力的补偿。这由set_motor_tilt_factor（）提供。
	第二种补偿是在_tilt_equal_thrust为true时，在所有倾斜的电动机上使用相等的推力。当电动机大角度倾斜时使用此功
	能，以防止侧倾和偏航控制器引起不稳定。通常，当电动机倾斜超过45度时，将使用此功能。在此角度下，假定可以使
	用固定的机翼控制面和使用剩余的多旋翼电机（例如，旋翼机尾翼）进行偏航控制来实现侧倾控制。
	通过应用_tilt_equal_thrust，倾斜的电动机有效地变成了单个变桨控制电动机。
	请注意，与从VTOL飞行相比，在过渡到VTOL时，我们使用了不同的策略。这样做的原因是，我们希望在过渡到固定翼
	飞行时向较高倾斜的发动机油门倾斜，以获取空速，而在过渡到VTOL飞行时，我们希望向较低的前向油门倾斜。因此，
	当过渡到固定翼时，我们升高倾斜发动机上的油门，而在过渡到VTOL时，降低倾斜发动机上的油门
 */
void QuadPlane::tilt_compensate_down(float *thrust, uint8_t num_motors)
{
    float inv_tilt_factor;
    if (tilt.current_tilt > 0.98f) {
        inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
    } else {
        inv_tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
    }

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    // 当我们经过Q_TILT_MAX时，我们将倾斜的电机绑在一起产生相等的推力。这使它们像一个音高一样
    // 控制电机，同时防止他们尝试滚动和偏航倾斜时进行控制。这大大提高了转换的最后阶段的稳定性
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply inv_tilt_factor first
    // 首先应用inv_tilt_factor
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= inv_tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    // 现在如果启用，则约束并应用_tilt_equal_thrust
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the tilted motors then reduce all
    // motors to keep them in proportion to the original thrust. This
    // helps maintain stability when tilted at a large angle
    // 如果我们使其中一个倾斜的电机饱和，则减少所有电动机以使其与原始推力成比例。这
    // 大角度倾斜时有助于保持稳定性
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}


/*
  tilt compensation when transitioning to VTOL flight
  过渡到VTOL飞行时的倾斜补偿
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
    float tilt_factor = cosf(radians(tilt.current_tilt*90));

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    // 当我们经过Q_TILT_MAX时，我们将倾斜的电机绑在一起产生相等的推力。这使它们像一个音高一样
    // 控制电机，同时防止他们尝试滚动和偏航倾斜时进行控制。这大大提高了转换的最后阶段的稳定性
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factor first
    // 首先应用tilt_factor
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        } else {
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    // now constrain and apply _tilt_equal_thrust if enabled
    // 现在如果启用，则约束并应用_tilt_equal_thrust
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
  根据飞行模式选择向上或向下倾斜补偿到固定机翼模式时，我们使用VTOL模式，我们使用tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (tilt.current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
    	// 电机不倾斜，无需补偿
        return;
    }
    if (in_vtol_mode()) {
        // we are transitioning to VTOL flight
    	// 我们正在过渡到VTOL飞行
        tilt_compensate_up(thrust, num_motors);
    } else {
        tilt_compensate_down(thrust, num_motors);
    }
}

/*
  return true if the rotors are fully tilted forward
  如果转子完全向前倾斜，则返回true
 */
bool QuadPlane::tiltrotor_fully_fwd(void)
{
    if (tilt.tilt_mask <= 0) {
        return false;
    }
    return (tilt.current_tilt >= 1);
}

/*
  control vectored yaw with tilt multicopters
  倾斜多旋翼飞机控制矢量偏航
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
    // total angle the tilt can go through
	// 倾斜可以通过的总角度
    float total_angle = 90 + tilt.tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    // 输出值（0到1）以使电机指向正上方
    float zero_out = tilt.tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    // 根据current_tilt计算基本倾斜量
    float base_output = zero_out + (tilt.current_tilt * (1 - zero_out));
    
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool no_yaw = (tilt.current_tilt > tilt_threshold);
    if (no_yaw) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * base_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * base_output);
    } else {
        float yaw_out = motors->get_yaw();
        float yaw_range = zero_out;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * (base_output + yaw_out * yaw_range));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
    }
}

/*
  control bicopter tiltrotors
  控制双旋翼倾转旋翼
 */
void QuadPlane::tiltrotor_bicopter(void)
{
    if (tilt.tilt_type != TILT_TYPE_BICOPTER) {
        return;
    }

    if (!in_vtol_mode() && tiltrotor_fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (assisted_flight) {
        hold_stabilize(throttle * 0.01f);
        motors_output(true);
    } else {
        motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    // 双旋翼假设修剪增加，因此我们按比例缩小以匹配
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt.tilt_yaw_angle / 90.0f;
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt.tilt_yaw_angle / 90.0f;
    }

    // reduce authority of bicopter as motors are tilted forwards
    // 降低电动机向前倾斜的能力，从而降低双翼飞机的使用权限
    const float scaling = cosf(tilt.current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    // 添加当前倾斜和约束
    tilt_left  = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}
