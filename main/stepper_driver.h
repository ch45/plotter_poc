
void stepper_motors_stop();
void stepper_motors_run();
void line_init();
int dump_stepper_state(char *txtbuf, size_t bufsz);
int dump_stepper_timing(char *txtbuf, size_t bufsz);
int dump_interrupt_timing(char *txtbuf, size_t bufsz);
void set_initial_interrupt_timing(uint64_t x_space, uint64_t x_mark, uint64_t y_space, uint64_t y_mark);
