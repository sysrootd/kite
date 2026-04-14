#ifndef CONFIG_H
#define CONFIG_H

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

#endif /* CONFIG_H */
