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

Once everything is set, you have to manually set the home position. There are 3 ways to do this.

1. In the settings, under Diagnostic, there is a "Start-Stop Homing" button. Press it and it will begin to close the curtain. Press it again to stop the curtain. This will now be the zero position.
2. Tap the left button (Button 1) on the motor **seven times quickly**. It will begin to close the curtain. Press either button once to stop it — this stops the motor and sets the position to zero.
3. Move the curtain to the spot you want to call "closed" (use hold-to-run, below), then **hold both buttons at once for 3 seconds**. The current position is set as home (0) without driving into the end stop.

It should now be homed to position 0, and when you press the open button, it should travel the centimeters distance you set in the settings.

## Button controls

| Gesture | Action |
| --- | --- |
| Tap Button 1 (close) | Close the curtain |
| Tap Button 2 (open) | Open the curtain |
| Press & hold Button 1 | Close **only while held** — stops the instant you let go, and stops automatically at the closed end |
| Press & hold Button 2 | Open **only while held** — stops the instant you let go, and stops automatically at the open end |
| Tap Button 1 seven times quickly | Start homing |
| Hold both buttons for 3 seconds | Set the current position as home (0) |
| Any press while the curtain is moving | Stops it instantly |
| Any press while homing | Stops it and sets the position to zero |

Hold-to-run is the child- and elderly-friendly mode: the curtain only moves while you keep the button pressed, so it can't run on its own into a person or a cord, and it never drives past the open or closed ends.

## Updating the firmware

Two ways to update:

- **Over the air, from the device:** open the settings page and, under Diagnostics, press **"Update Firmware (GitHub)"**. The device downloads the latest release for its model straight from GitHub, flashes itself, and reboots — no computer needed. (Requires an internet connection.)
- **Manually:** download the `*.ota.bin` (for a device already on your network) or `*.factory.bin` (for a fresh USB flash) for your model from the [latest release](https://github.com/Valar-Systems/Ropener/releases/latest) and upload it as before.
