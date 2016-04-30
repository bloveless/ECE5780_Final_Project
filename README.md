# Stair Climbing Robot

Our project is a stair climbing robot. It can detect a wall and turn around, then detect a stair, extend it's legs, climb the stair, and detect it has completed the stair retracting it's legs.

We used FreeRTOS as a scheduler and added each section of our code to a new task. Essentially this
modularizes our code and allowed us to working freely and independently of each other.

## Source Files

    src/
        // Standard configuration files for the project
        adc.c                           // Configure the adc pins and peripheral
        freertos.c                      // Add the required tasks to FreeRTOS
        gpio.c                          // Configure the gpio pins
        i2c.c                           // Configure the i2c pins and peripherals
        main.c                          // Call all peripheral configurations and start FreeRTOS
        stm32f3xx_hal_msp.c             // Add default interrupts
        stm32f3xx_hal_timebase_TIM.c    // Configure FreeRTOS timebase
        stm32f3xx_it.c                  // Handle all interrupts (these call back to our code)
        tim.c                           // Configure all timers, modes, prescale, and ARR
        --
        // Our tasks
        Heartbeat.c                     // Blinking light for error detection
        MPU6050.c                       // Determine the current angle of the robot
        PIDController.c                 // Drive and spin the robot
        Proximity.c                     // Detect when the robot has come close to an object
        Servo.c                         // Control the outer tracks position