# WS2812B Addressable LED Control

Sylib provides an easy-to-use API for controlling WS2812B (Neopixel) LEDs

![pretty lights](https://user-images.githubusercontent.com/54775775/196319983-b07b49c9-51d7-4f72-9aa1-8d6471a6ab59.png)


## Hardware
To use these lights with a V5 brain, some custom wiring is needed. It is fairly simple, but some soldering is needed. 

I reccomend cutting up a VEX 3-wire extension cable in order to aquire a cable that can easily plug into the brain's ADI ports. 

1) Solder the 5V (red/middle pin on the V5 ports) wire to the 5V line on your light strip
2) Solder the ground (black/bottom pin on the V5 ports) wire to the ground line on your light strip
3) Solder the data (white/top pin on the V5 ports) wire to the data line on your lights


Make sure to attach the wires to the correct side of the light strip, because they are directioned

![wiring](https://user-images.githubusercontent.com/54775775/196319956-f08a94a9-01dd-4cc4-9559-bf5ddf4fa985.png)

## Software

Full documentation can be found [here]

In order to control the lights, first create an Addrled object. 

```cpp
auto myLights = sylib::Addrled(1, 3, 64);
```

This creates an addressable led object configured as plugged into
ADI port 3 (C) on an ADI expander plugged into smart port 1 that is 64 pixels long. 
64 pixels is the maximum number that the V5 of this library supports.

In order to use the build-in three wire ports on the V5, use `22` as the smart port value. 


___

To set the entire strip to a certain color, use the `set_all` command

```cpp
myLights.set_all(0xE62169);
```

This function takes an integer as an argument, and it is interpreted as a hexadecimal color code. 
The easiest way to use this is to supply a number already in hexadecimal form. 

___

In order to set the lights to a gradient of colors, use `gradient`

```cpp
myLights.gradient(0x00FF00, 0x0000FF);
```

This starts the first pixel of the strip to `#00FF00`, and the last pixel to `#0000FF`,
and uses linear interpolation in RGB color space to find values for the pixels in between.

___

In order to set a single pixel to a certain color, use `set_pixel`

```cpp
myLights.set_pixel(0x123456, 5);
```

This sets the 6th pixel on the strip to `#123456`

The pixel number is zero-indexed. 

___

To set every pixel at once, use `set_buffer`

```cpp
auto colors = std::vector<std::uint32_t>();
colors.resize(16);
for(int i = 0; i < colors.size(); i++){
	colors[i] = sylib::Addrled::rgb_to_hex(64, 64, i * 15);
}
myLights.set_buffer(colors);
```

___

To shift all colors by one pixel position, use `rotate`

```cpp
myLights.rotate(1, false);
```

This shifts all colors down the strip by one pixel, in the positive direction, and loops
the pixels at the end back around to the start.

___

To continuously shift all colors down the strip, use `cycle`

```cpp
myLights.cycle(*myLights, 15);
```

This sets the lights to what they were already at, then instructs sylib to automatically
rotate the colors at a rate described by the `speed` parameter.

Instead of using what the lights were already at with `*myLights`, you can supply your own buffer,
which can be larger than the maximum strip size.


___

You can turn an individual light strip off with `clear`

```cpp
myLights.clear();
```

___

In order to send a single pulse of color down the strip, use `pulse`

```cpp
myLights.pulse(0xF5A9B8, 2, 10);
```

This sends a pulse 2 pixels wide of color `#F5A9B8` down the strip at speed 10. 

___

To shift the red, green, or blue values applied to the lights, use `color_shift`

```cpp
myLights.color_shift(64, 0, -127);
```

This will make every pixel on the strip `64/255` more red, leave green unchanged, and make
every pixel `-127/255` less blue. 

`color_shift` and `pulse` work *on top* of whatever other control is currently set for the lights.
They do not modify the buffer stored by the `Addrled` object directly, only augment it before
it is sent to the lights. 
