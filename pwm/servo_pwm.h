/**
	 * @brief Initializing the pwm module
	 *
	 * Will set the pwm chanel of desired GPIO Pin
     * @param gpio_pin_num GPIO Pin number
     * 
	 * @return Slice number of pwm module
	 */
uint pwm_chan_init(int gpio_pin_num);

/**
	 * @brief The BTS7960 H-Bridge DC motor module 
	 * needs 2 separate PWMs for forward and reverse motion
	 *
     * @param pwm_slice_0 PWM Pin number - Will set the pwm value 
	 * to desired GPIO 
     * @param pwm_slice_1 PWM Pin number - Will set the pwm value 
	 * to desired GPIO
     * @param speed PWM Speed - it can be a positive or negative number
     * 
	 * @return void
	 */
void set_two_chans_pwm(uint slice_num, int speed);