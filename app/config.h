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

#endif /* CONFIG_H */
