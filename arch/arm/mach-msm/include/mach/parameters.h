#if (defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE))

struct ariesve_parameter {	unsigned int usb_sel;	unsigned int uart_sel;	unsigned int usb_mode;};
extern void msm_read_param(struct ariesve_parameter *param_data);extern void msm_write_param(struct ariesve_parameter *param_data);
#endif