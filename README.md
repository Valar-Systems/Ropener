## Kits
You can purchase a hardware kit [here](https://valarsystems.com/products/morningrope). If it's sold out, **PLEASE** sign up to the waitlist. It's the only indication we have to make more. If you don't, then we'll **NEVER** make more.

Without the kit, it's a pain to source all the parts, and costs a lot more because you'll be forced to buy 100 screws (when you only need 2) and 100ft of rope (when you only need 30ft) at a time.

![kit photo](/media/ropener_kit.jpg)


# Ropener: Automated Curtain Opener

The Ropener is a smart curtain opener that uses two buttons and HTTP requests or ESPHome to open and close your curtain. Combine it with Node-RED, Home Assistant (via ESPhome), or any other system capable of sending HTTP requests and get your curtain to open and close automatically.

![window opener GIF](/media/curtain-gif.gif)

## How it works

The motor is attached to your wall just behind one of the curtains

![window opener GIF](/media/motor-gif.gif)

There is a carriage attached to the back of each curtain panel. A string runs through each carriage and through the motor which pulls it open and close.

![window opener GIF](/media/string-gif.gif)

At the heart of the device is our custom PCB which uses an ESP32 and Trinamic TMC2209 stepper driver. 

The motor is dead silent.

If a stall occurs, the advanced TMC2209 will detect the increased back-EMF from the motor and stop it automatically, like magic.

![window opener](/media/model-s-pcb.jpg)

## Will it work on your curtains?

This works on backtab curtains that looks like this:
![window opener GIF](/media/backtab-example.jpg)

Or with curtains with rings, like this:
![window opener GIF](/media/curtain-rings2.jpg)


## How to build it

The build guide can be found [here](https://www.canva.com/design/DAHAzX1956w/QnXkFw6Gx1b4t2T4kpTvWQ/view?utm_content=DAHAzX1956w&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=hb3c45bcf45).

We've created a hardware kit that includes everything. If you source the parts individually, it will cost 5X more just because all of the hardware need to purchased in bulk from places like McMaster-Carr. You may optionally 3D print your parts.

* Total print time: ~12 hours (You can also purchase the 3D prints)
* Total assembly time: ~20 minutes

[Link to kit](https://valarsystems.com/products/morningrope)


## How to 3D print it

Go to the repo folder "hardware" -> "Plastics" -> "platter"


## How to install it

Either use command strips (which is recommended) or 4 small nails (for drywall only), or two screws/anchors. It's very easy to install.


## Sending commands

ESPHome is the preferred way of controlling it. Pair it with Matterbridge and you can control your curtains with Alexa, Apple, and Google Home.
