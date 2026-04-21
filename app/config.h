#ifndef CONFIG_H
#define CONFIG_H

//Change here your mcu base (HSI clk) and its supported highest (PLL clk) freq

#define BASE_CLOCK_SPEED        16000000U
#define HIGHEST_CLOCK_SPEED     84000000U

/* Choose one of the clock profiles defined */

//  CLOCK_PROFILE_LOW
//  CLOCK_PROFILE_MEDIUM
//  CLOCK_PROFILE_HIGH
//  CLOCK_PROFILE_MAX

#define SYSTEM_CLOCK_PROFILE CLOCK_PROFILE_MAX

/* Enable/disable the FPU at startup. */
#define ENABLE_FPU 1

/* Enable/disable the MPU at startup. */
#define ENABLE_MPU 1

/* Enable/disable STOP mode. */
#define ENABLE_STOP_MODE 0

/* Enable/disable Debug mode for each task (each task will have a const string as task name)*/
// Will enable name memeber in tcb of a task stack that holds task name to make debug ease
#define SCHED_DEBUG 0 

/* Maximum no of tasks that will be created*/
#define MAX_TASKS              16U

/* Min Time slice scheduler give for each task in run time(5ms is idle) */
#define SCHED_TIME_SLICE       5U

/* Maximum no mutex that are going to use in a single task(mutex nesting) */
#define HELD_MUTEX_MAX         4U

#endif /* CONFIG_H */
