/**
	 * @brief Initializing the pwm module
	 *
	 * Will set the pwm chanel of desired GPIO Pin
     * @param gpio_pin_num GPIO Pin number
     * 
	 * @return Slice number of pwm module
	 */
uint pwm_chan_init(const int gpio_pin_num);