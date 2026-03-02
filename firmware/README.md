# ESPHome YAML

Update the two variables:
1. Distance: In centimeters
2. Direction of travel: Enter "cw" or "ccw" to change opening direction

The Matter-Compatible firmware reverses the 100% and 0% direction, so the cover matches the Matter standard.

In Home Assistant, 0 percent is closed, and 100 percent is open. But in Matter, it's the opposite. 100 is closed and 0 is open.

If using the Matterbridge plugin, use the Matter-Compatible version, this way Apple Home, Google Home, and Alexa all open/close the device correctly.