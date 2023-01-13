#ifndef QUAD_ENCODER_H
#define QUAD_ENCODER_H

#ifdef	__cplusplus
extern "C" {
#endif

	uint pwm_chan_init(int gpio_pin_num);
	void set_two_chans_pwm(uint slice_num, int speed);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file