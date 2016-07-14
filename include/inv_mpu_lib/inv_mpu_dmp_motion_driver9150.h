#ifdef __cplusplus
extern "C"{
#endif 
/* Set up functions. */
uint8_t dmp_load_motion_driver_firmware(void);
uint8_t dmp_set_fifo_rate(uint16_t rate);
uint8_t dmp_get_fifo_rate(uint16_t *rate);
uint8_t dmp_enable_feature(uint16_t mask);
uint8_t dmp_get_enabled_features(uint16_t *mask);
uint8_t dmp_set_interrupt_mode(uint8_t mode);
uint8_t dmp_set_orientation(uint16_t orient);
uint8_t dmp_set_gyro_bias(int32_t *bias);
uint8_t dmp_set_accel_bias(int32_t *bias);

/* Tap functions. */
uint8_t dmp_register_tap_cb(void (*func)(uint8_t, uint8_t));
uint8_t dmp_set_tap_thresh(uint8_t axis, uint16_t thresh);
uint8_t dmp_set_tap_axes(uint8_t axis);
uint8_t dmp_set_tap_count(uint8_t min_taps);
uint8_t dmp_set_tap_time(uint16_t time);
uint8_t dmp_set_tap_time_multi(uint16_t time);
uint8_t dmp_set_shake_reject_thresh(int32_t sf, uint16_t thresh);
uint8_t dmp_set_shake_reject_time(uint16_t time);
uint8_t dmp_set_shake_reject_timeout(uint16_t time);

/* Android orientation functions. */
uint8_t dmp_register_android_orient_cb(void (*func)(uint8_t));

/* LP quaternion functions. */
uint8_t dmp_enable_lp_quat(uint8_t enable);
uint8_t dmp_enable_6x_lp_quat(uint8_t enable);

/* Pedometer functions. */
uint8_t dmp_get_pedometer_step_count(uint32_t *count);
uint8_t dmp_set_pedometer_step_count(uint32_t count);
uint8_t dmp_get_pedometer_walk_time(uint32_t *time);
uint8_t dmp_set_pedometer_walk_time(uint32_t time);

/* DMP gyro calibration functions. */
uint8_t dmp_enable_gyro_cal(uint8_t enable);

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */
uint8_t dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, int16_t *sensors, uint8_t *more);

#ifdef __cplusplus
}
#endif