#pragma once

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.
//	通用PID算法，具有EEPROM支持的常数存储。

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>

#define AC_PID_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency	默认输入滤波器频率
#define AC_PID_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_RESET_TC          0.16f   // Time constant for integrator reset decay to zero	积分器复位衰减为零的时间常数

/// @class	AC_PID
/// @brief	Copter PID control class
class AC_PID {
public:

    // Constructor for PID	PID的构造函数
    AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float dt);

    // set_dt - set time step in seconds	设置时间步长（以秒为单位）
    void set_dt(float dt);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    //	update_all-将目标和测量输入设置为PID控制器并计算输出
    //	过滤目标和错误
    //	然后计算并过滤导数
    //	然后根据极限标志的设置更新积分
    float update_all(float target, float measurement, bool limit = false);

    //  update_error - set error input to PID controller and calculate outputs
    //  target is set to zero and error is set and filtered
    //  the derivative then is calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    //  Target and Measured must be set manually for logging purposes.
    // 	todo: remove function when it is no longer used.
    // 	update_error-设置输入到PID控制器的错误并计算输出
    //	目标设置为零，错误设置并过滤
    //	然后对导数进行计算和过滤
    //	然后根据极限标志的设置更新积分
    //	必须手动设置“目标”和“测量”以用于记录目的。
    float update_error(float error, bool limit = false);

    //  update_i - update the integral
    //  if the limit flag is set the integral is only allowed to shrink
    // 	update_i-更新积分
    //  如果设置了限制标志，则仅允许缩小积分
    void update_i(bool limit);

    // get_pid - get results from pid controller
    // get_pid-从pid控制器获取结果
    float get_pid() const;
    float get_pi() const;
    float get_p() const;
    float get_i() const;
    float get_d() const;
    float get_ff();

    // todo: remove function when it is no longer used.
    float get_ff(float target);

    // reset_I - reset the integrator
    // reset_I-重置积分器
    void reset_I();

    // reset_I - reset the integrator smoothly to zero within 0.5 seconds
    // reset_I-在0.5秒内将积分器平滑地重置为零
    void reset_I_smoothly();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    // reset_filter-输入过滤器将重置为提供给set_input（）的下一个值
    void reset_filter() {
        _flags._reset_filter = true;
    }

    // load gain from eeprom
    // 从eeprom读取增益
    void load_gains();

    // save gain to eeprom
    // 保存增益到eeprom
    void save_gains();

    /// operator function call for easy initialisation
    ///	操作员函数调用，易于初始化
    void operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt);

    // get accessors
    // 获取访问器
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    AP_Float &kD() { return _kd; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    float imax() const { return _kimax.get(); }
    float get_filt_alpha(float filt_hz) const;
    float get_filt_T_alpha() const;
    float get_filt_E_alpha() const;
    float get_filt_D_alpha() const;

    // set accessors
    // 设置访问器
    void kP(const float v) { _kp.set(v); }
    void kI(const float v) { _ki.set(v); }
    void kD(const float v) { _kd.set(v); }
    void ff(const float v) { _kff.set(v); }
    void imax(const float v) { _kimax.set(fabsf(v)); }
    void filt_T_hz(const float v);
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);

    // set the desired and actual rates (for logging purposes)
    // 设置所需的期望和实际速率（用于记录目的）
    void set_target_rate(float target) { _pid_info.target = target; }
    void set_actual_rate(float actual) { _pid_info.actual = actual; }

    // integrator setting functions
    // 积分器设置功能
    void set_integrator(float target, float measurement, float i);
    void set_integrator(float error, float i);
    void set_integrator(float i);

    const AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

    // parameter var table
    // 参数变量表
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _filt_T_hz;         // PID target filter frequency in Hz		PID目标滤波器频率，单位为Hz
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz		PID误差过滤器的频率，单位为Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz	PID微分滤波器的频率，单位为Hz

    // flags	标志
    struct ac_pid_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
        // 						  当下次调用set_input时应重置输入过滤器时为true
    } _flags;

    // internal variables	内部变量
    float _dt;                // timestep in seconds								时间步（以秒为单位）
    float _integrator;        // integrator value									积分值
    float _target;            // target value to enable filtering					目标值以启用过滤
    float _error;             // error value to enable filtering					误差值以启用过滤
    float _derivative;        // derivative value to enable filtering				微分值以启用过滤
    uint16_t _reset_counter;  // loop counter for reset decay						循环计数器，用于复位衰减
    uint64_t _reset_last_update; //time in microseconds of last update to reset_I
    							 //上次更新到reset_I的时间（以微秒为单位）

    AP_Logger::PID_Info _pid_info;
};
