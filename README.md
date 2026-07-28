# Ropener: Automated Curtain Opener

<img src="/media/main-image-2.jpg" width="70%"/>

![window opener GIF](/media/curtain-gif-2.gif)

The Ropener is a smart curtain opener that uses two buttons and ESPHome to open and close your curtain. Combine it with Node-RED, Home Assistant (via ESPhome) to get your curtain to open and close automatically.

## Kits
This device was created by me, [Daniel Frenkel](https://github.com/daniel-frenkel), and I have a small US-based shop that you can purchase a [hardware kit](https://valarsystems.com/products/ropener) from. If it's sold out, **PLEASE** sign up to the waitlist. It's the only indication I have to make more. If you don't, then I'll **NEVER** make more.

Without the kit, it's a pain to source all the parts, and costs a lot more because you'll be forced to buy 100 screws (when you only need 2) and 100ft of rope (when you only need 30ft) at a time.

<img src="/media/ropener_kit.jpg" width="70%" />

## How it works

The motor is attached to your wall just behind one of the curtains

![window opener GIF](/media/motor-gif.gif)

There is a carriage attached to the back of each curtain panel. A string runs through each carriage and through the motor which pulls it open and close.

![window opener GIF](/media/carriage-gif-1.gif)

At the heart of the device is our custom PCB which uses an ESP32 and Trinamic TMC2209 stepper driver. The motor is dead silent.

The latest **v1.1** hardware revision uses a smaller-diameter MK8 drive gear that boosts the pull strength from 11 lb to **15 lb**, swaps to a single larger bearing per arm (fewer parts), and uses pre-cut PTFE tubes. See the [hardware notes](hardware/README.md) for details.

## Will it work on your curtains?

This works on the following curtains:

<img src="/media/curtain-types.jpg" width="50%" />

## How to build it

The build guide can be found [here](https://youtu.be/uowt6jLxNso).

The installation guide can be found [here](https://youtu.be/p8078ezLZHw).

We've created a hardware kit that includes everything. If you source the parts individually, it will cost 5X more just because all of the hardware need to purchased in bulk from places like McMaster-Carr. You may optionally 3D print your parts.

* Total print time: ~9 hours (You can also purchase the 3D prints)
* Total assembly time: ~20 minutes

[Link to kit](https://valarsystems.com/products/ropener)


## How to 3D print it

The current revision is **v1.1**. Open the print plate at [hardware/plastics/v1.1/platter-all-v1.1.3mf](hardware/plastics/v1.1/platter-all-v1.1.3mf), or print the individual `.step` files in that folder. The earlier [v1.0](hardware/plastics/v1.0) files remain for anyone who already built one.


## How to install it

Either use the included foam mounting squares (recommended), small nails (for drywall only), or two screws/anchors. It's very easy to install.


## Sending commands

ESPHome is the preferred way of controlling it. Pair it with Matterbridge and you can control your curtains with Alexa, Apple, and Google Home.
