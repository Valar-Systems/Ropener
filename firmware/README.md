# ESPHome YAML

#### Warning:
***Do not use StallGuard*** unless you set it up correctly. StallGuard is finicky and requires exact parameters. I am working on creating preset settings that should fix this, but for now, do not use it. It is also not required for this device to work properly.

After pasting the YAML into ESPHome, ***only*** change the device names. Do not change anything else.

```
esphome:
  name: ropener-bedroom-1                 # change this name
  friendly_name: "Ropener Bedroom 1"      # change this name
```

After uploading, go to the settings page here and update the things in red.

<img width="826" height="848" alt="settings" src="https://github.com/user-attachments/assets/e0dff310-e754-498a-93b4-5ce42e12c749" />

The most important settings are the centimeters (this sets the max travel distance) and the motor direction.

IRUN sets the motor current. 31 is the max value. The downside of a higher setting is that the driver will heat up quickly and will shut down when too hot. Use as low of a value as possible. A value too low however will make the motor too weak and it will stall. It's best to experiment to find the lowest value possible while not stalling the motor. Stalling does not damage the motor.

Be sure the state is correct in the settings when the device is moving.

<img width="337" height="497" alt="state" src="https://github.com/user-attachments/assets/a2ca4c58-d079-48ff-a29f-1bfab5c8ef15" />

Once everything is set, you have to manually set the home position. There are 2 ways to do this.

1. In the settings, under Diagnostic, there is a "Start-Stop Homing" button. Press it and it will begin to close the curtain. Press it again to stop the curtain. This will now be the zero position.
2. Option 2. Long press and hold the left button on the motor for 3-10 seconds. Once you let go, it will begin to close the curtain. Single-press the same button again to stop it, this will stop the motor and set the position to zero. 

It should now be homed to position 0, and when you press the open button, it should travel the centimeters distance you set in the settings. 
