#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
  ��ת����͸�������Ŀ��ƴ��롣 ͨ����Q_TILT_MASK����Ϊ����ֵ������
 */


/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
  ��0��1����б�ȱ������������б�ȱ仯
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
            // ��������90 DPS���ֶ�����������������ǲ��ȶ�������Խ��п��ٿ���
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt / 90.0f;
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
  ���һ����ת���޵�rotorrotor�Ƕȡ���б���Ǵ�0��1
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<tilt.current_tilt);
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    // ת��Ϊ0..1000��Χ�����
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);
}

/*
  update motor tilt for continuous tilt servos
  ���µ����б����ʵ��������б�ŷ�
 */
void QuadPlane::tiltrotor_continuous_update(void)
{
    // default to inactive
	// Ĭ��Ϊ���
    tilt.motors_active = false;

    // the maximum rate of throttle change
    // ������ű仯��
    float max_change;
    
    if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
    	// ���Ǵ��ڴ��̶���ģʽ�� ����б���һֱ��ǰ�ƶ�����������Ϊǰ���������
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
        	// ���һֱ��ǰ����ʼ����������������
            uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            motors->output_motor_mask(tilt.current_throttle, mask, plane.rudder_dt);
            // prevent motor shutdown
            // ��ֹ���ͣ��
            tilt.motors_active = true;
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    // ��ס��������VTOL���е�����ˮƽ
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
      ���Ǵ���VTOLģʽ�� ������ҪŪ�����Ҫ������б�ȡ����ǽ�ʹ��3�ֲ��ԣ�
      1����QSTABILIZE��QHOVER�У��ǶȽ�����Ϊ�㡣�������Խ���Щģʽ������ȫ�ָ�ģʽ��
      2���ڹ̶��������л��ٶȿ���ģʽ�£����ǽ����������ǰ���������ýǶȣ������б����Q_TILT_MAX������ ��ȡ����Q_VFWD_GAIN������
      3��������Ǵ���TRANSITION_TIMERģʽ����ô�������ڹ��ɵ���ǰ���У�Ӧ�ý�����һֱ��ǰ
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
    	// �������ڹ��ɵ��̶�����-���綯��һֱ��ǰ��б
        tiltrotor_slew(1);
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
    	// ����ɹ���֮ǰ�����ǽ���б������ΪQ_TILT_MAX�� ����50��������ֻ�ܻ��Q_TILT_MAX��
    	// ����50�������ţ����ǻ������½��� ����������ȷ����Q_VFWD_GAIN��
        float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);
        tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
  ���һ����ת���޵�rotorrotor�Ƕȡ���б��Ϊ0��1
 */
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
	// �ŷ����Ϊ�����ƣ����ܰ�������
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    // ����current_tilt���������ӳ�tiltrotor_binary_update�е�����
    float max_change = tilt_max_change(!forward);
    if (forward) {
        tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
    } else {
        tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
  ���¶�������б�ŷ����ĵ綯����б
 */
void QuadPlane::tiltrotor_binary_update(void)
{
    // motors always active
	// ���ʼ�մ��ڻ״̬
    tilt.motors_active = true;

    if (!in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
    	// ���Ǵ��ڴ��̶���ģʽ���ƶ�����б�ĵ��һ·��ǰ������������Ϊǰ���������
        tiltrotor_binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (tilt.current_tilt >= 1) {
            uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            // ���һֱ��ǰ����ʼ����������������
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        tiltrotor_binary_slew(false);
    }
}


/*
  update motor tilt
  ���µ����б
 */
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
    	// û�е����б
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
	����һ��������е���б
	������������ʽ����һ����Ӧ��_tilt_factor��������бʱ��С�Ĵ�ֱ�����Ĳ���������set_motor_tilt_factor�����ṩ��
	�ڶ��ֲ�������_tilt_equal_thrustΪtrueʱ����������б�ĵ綯����ʹ����ȵ����������綯����Ƕ���бʱʹ�ô˹�
	�ܣ��Է�ֹ�����ƫ�������������ȶ���ͨ�������綯����б����45��ʱ����ʹ�ô˹��ܡ��ڴ˽Ƕ��£��ٶ�����ʹ
	�ù̶��Ļ���������ʹ��ʣ��Ķ������������磬�����β������ƫ��������ʵ�ֲ�����ơ�
	ͨ��Ӧ��_tilt_equal_thrust����б�ĵ綯����Ч�ر���˵����佰���Ƶ綯����
	��ע�⣬���VTOL������ȣ��ڹ��ɵ�VTOLʱ������ʹ���˲�ͬ�Ĳ��ԡ���������ԭ���ǣ�����ϣ���ڹ��ɵ��̶���
	����ʱ��ϸ���б�ķ�����������б���Ի�ȡ���٣����ڹ��ɵ�VTOL����ʱ������ϣ����ϵ͵�ǰ��������б����ˣ�
	�����ɵ��̶���ʱ������������б�������ϵ����ţ����ڹ��ɵ�VTOLʱ��������б�������ϵ�����
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
    // �����Ǿ���Q_TILT_MAXʱ�����ǽ���б�ĵ������һ�������ȵ���������ʹ������һ������һ��
    // ���Ƶ����ͬʱ��ֹ���ǳ��Թ�����ƫ����бʱ���п��ơ����������ת�������׶ε��ȶ���
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply inv_tilt_factor first
    // ����Ӧ��inv_tilt_factor
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= inv_tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    // ����������ã���Լ����Ӧ��_tilt_equal_thrust
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
    // �������ʹ����һ����б�ĵ�����ͣ���������е綯����ʹ����ԭʼ�����ɱ�������
    // ��Ƕ���бʱ�����ڱ����ȶ���
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}


/*
  tilt compensation when transitioning to VTOL flight
  ���ɵ�VTOL����ʱ����б����
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
    float tilt_factor = cosf(radians(tilt.current_tilt*90));

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    // �����Ǿ���Q_TILT_MAXʱ�����ǽ���б�ĵ������һ�������ȵ���������ʹ������һ������һ��
    // ���Ƶ����ͬʱ��ֹ���ǳ��Թ�����ƫ����бʱ���п��ơ����������ת�������׶ε��ȶ���
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factor first
    // ����Ӧ��tilt_factor
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        } else {
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    // now constrain and apply _tilt_equal_thrust if enabled
    // ����������ã���Լ����Ӧ��_tilt_equal_thrust
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
  ���ݷ���ģʽѡ�����ϻ�������б�������̶�����ģʽʱ������ʹ��VTOLģʽ������ʹ��tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (tilt.current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
    	// �������б�����貹��
        return;
    }
    if (in_vtol_mode()) {
        // we are transitioning to VTOL flight
    	// �������ڹ��ɵ�VTOL����
        tilt_compensate_up(thrust, num_motors);
    } else {
        tilt_compensate_down(thrust, num_motors);
    }
}

/*
  return true if the rotors are fully tilted forward
  ���ת����ȫ��ǰ��б���򷵻�true
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
  ��б������ɻ�����ʸ��ƫ��
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
    // total angle the tilt can go through
	// ��б����ͨ�����ܽǶ�
    float total_angle = 90 + tilt.tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    // ���ֵ��0��1����ʹ���ָ�����Ϸ�
    float zero_out = tilt.tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    // ����current_tilt���������б��
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
  ����˫������ת����
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
    // ˫��������޼����ӣ�������ǰ�������С��ƥ��
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt.tilt_yaw_angle / 90.0f;
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt.tilt_yaw_angle / 90.0f;
    }

    // reduce authority of bicopter as motors are tilted forwards
    // ���͵綯����ǰ��б���������Ӷ�����˫��ɻ���ʹ��Ȩ��
    const float scaling = cosf(tilt.current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    // ��ӵ�ǰ��б��Լ��
    tilt_left  = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}
