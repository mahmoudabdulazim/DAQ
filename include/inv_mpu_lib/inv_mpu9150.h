#ifdef __cplusplus
extern "C"{
#endif 

/* Set up APIs */
struct int_param_s
{
#if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430
	void (*cb)(void);
	uint16_t pin;
	uint8_t lp_exit;
	uint8_t active_low;
#elif defined EMPL_TARGET_UC3L0
	uint32_t pin;
	void (*cb)(volatile void*);
	void *arg;
#endif
};

uint8_t mpu_init(struct int_param_s *int_param);
uint8_t mpu_set_bypass(uint8_t bypass_on);

/* Configuration APIs */
uint8_t mpu_lp_accel_mode(uint8_t rate);
uint8_t mpu_lp_motion_interrupt(uint16_t thresh, uint8_t time,
																uint8_t lpa_freq);
uint8_t mpu_set_int_level(uint8_t active_low);
uint8_t mpu_set_int_latched(uint8_t enable);

uint8_t mpu_set_dmp_state(uint8_t enable);
uint8_t mpu_get_dmp_state(uint8_t *enabled);

uint8_t mpu_get_lpf(uint16_t *lpf);
uint8_t mpu_set_lpf(uint16_t lpf);

uint8_t mpu_get_gyro_fsr(uint16_t *fsr);
uint8_t mpu_set_gyro_fsr(uint16_t fsr);

uint8_t mpu_get_accel_fsr(uint8_t *fsr);
uint8_t mpu_set_accel_fsr(uint8_t fsr);

uint8_t mpu_get_compass_fsr(uint16_t *fsr);

uint8_t mpu_get_gyro_sens(float *sens);
uint8_t mpu_get_accel_sens(uint16_t *sens);

uint8_t mpu_get_sample_rate(uint16_t *rate);
uint8_t mpu_set_sample_rate(uint16_t rate);
uint8_t mpu_get_compass_sample_rate(uint16_t *rate);
uint8_t mpu_set_compass_sample_rate(uint16_t rate);

uint8_t mpu_get_fifo_config(uint8_t *sensors);
uint8_t mpu_configure_fifo(uint8_t sensors);

uint8_t mpu_get_power_state(uint8_t *power_on);
uint8_t mpu_set_sensors(uint8_t sensors);

uint8_t mpu_set_accel_bias(const int32_t *accel_bias);

/* Data getter/setter APIs */
uint8_t mpu_get_gyro_reg(int16_t *data);
uint8_t mpu_get_accel_reg(int16_t *data);
uint8_t mpu_get_compass_reg(int16_t *data);
uint8_t mpu_get_temperature(int32_t *data);

uint8_t mpu_get_int_status(int16_t *status);
uint8_t mpu_read_fifo(int16_t *gyro, int16_t *accel, uint8_t *sensors, uint8_t *more);
uint8_t mpu_read_fifo_stream(uint16_t length, uint8_t *data, uint8_t *more);
uint8_t mpu_reset_fifo(void);

uint8_t mpu_write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);
uint8_t mpu_read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);
uint8_t mpu_load_firmware(uint16_t length, const uint8_t *firmware, uint16_t start_addr, uint16_t sample_rate);

uint8_t mpu_reg_dump(void);
uint8_t mpu_read_reg(uint8_t reg, uint8_t *data);
uint8_t mpu_run_self_test(int32_t *gyro, int32_t *accel);

#ifdef __cplusplus
}
#endif