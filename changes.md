# Changes from master
This file will explain the changes made that differs from the master.

For a comparison between the codes look [here](https://github.com/FuelFighter/Motor-drive-2018/compare/master...feat-variable_max_speed).

### Digicom.c

Digicom.c is used to interface with the other components of the car, such as the CAN bus, UART and the blinking of lights.
In [Digicom.c](https://github.com/FuelFighter/Motor-drive-2018/blob/4a34b9616d1ef1c2f01456ea91af67b0f3ea2aec/DigiCom.c)
there were some minor changes.

#### Summarized:
- `108-111` Added setting startvalues when the actuator is detected.(is connected to `103-104` in main.c
- `114-115` Added ability to receive the new values `closest_gear` and `near_gear` from actuator.
- `131, 199` For better debugging, just some added printing of messages to UART and comments
- `244-273` Added lightsymbols for the new added states.

### controller.c

[Controller.c](https://github.com/FuelFighter/Motor-drive-2018/blob/4a34b9616d1ef1c2f01456ea91af67b0f3ea2aec/controller.c) is used to calculate and output the PWM signal to the motor.

#### Summarized:
- `16-17` Added constants instead of hardcoding the limits in `93-101`.
- `38` Added `f32_DutyCycle` to be able to switch direction of the motor. Second gear requires motion in the other direction. `f32_DutyCycle` is always a value between 50-95, whilst `f32_DutyCycleCmd` can be equal to `f32_DutyCycle` or `100-f32_DutyCycle`, depending on the motor direction.
- `41` Changed type of throttle command.
- `60-62` Slowing the motor down with regeneration when it is not in gear.
- `103-111` Setting the dutycyclecmd to the correct direction according to the required gear.

### motor_controller_selection.h
This is for selecting the right board, as explained in the comments of the file. `NUM_MAGNETS` should be two in both cases, not only in the new version.

### speed.c
For speed calculations. Here we added the new gear ratios and added logic to compute the syncronized duty cycle for the second gear.

### state_machine.h
- `18-24, 35` added a bunch of states for the new gear system.
- `80` added function to calculate the required gear. This is a pure function.

### state_machine.c
This is the one with major changes. [state machine picture](statemachine.png). Red lines are when braking occurs, green are acceleration. Black are watchdog timers. The red and green lines sometime have additional conditions to them. Max means max speed is achieved in that gear(by looking at dutycycle and motor_current, line `169-174`. The `LOW_CURRENT_THRESHOLD` may need to be tuned.) and it will change to gear 2. Otherwise the conditions are velocity values which are also needed to be tuned.
