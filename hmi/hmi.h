#ifndef HMI_H
#define HMI_H

typedef struct lcd_screen lcd_screen_t;
typedef struct hmi hmi_t;

// Public API: Starts Core 1 and sets up the display task
hmi_t* hmi_create(void);
void hmi_compute(hmi_t* hmi);

#endif // UI_MANAGER_H