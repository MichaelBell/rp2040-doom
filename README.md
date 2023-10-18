# RP2040 Doom

This is a port of Doom for RP2040 devices, derived from [Chocolate Doom](https://github.com/chocolate-doom/chocolate-doom).

This version for PicoVision is a work in progress, only made possible at all by kilograham's amazing work to get Doom running on
RP2040 in the first place.  He made very substantial changes to support running the 
entire shareware `DOOM1.WAD` which is 4M big on a Raspberry Pi Pico with only 2M flash!

This version will run the shareware version, full Ultimate Doom and Doom 2 on the PicoVision hardware, by using the on board 8MB PSRAM
to store the compressed WAD file loaded from an SD card.

You can read many details on the original RP2040 port in the blog post [here](https://kilograham.github.io/rp2040-doom/).

Note that I have not been maintaining the `chocolate-doom` executable for non-RP2040 - it's probably broken.

The original Chocolate Doom README is [here](README-chocolate.md).

## PicoVision Doom Quickstart

Grab a [PicoVision](https://shop.pimoroni.com/products/picovision), put the doom1.whx file in the root directory of an SD card and insert that in the PicoVision.
Download doom_tiny_usb from the latest [release](https://github.com/MichaelBell/rp2040-doom/releases/latest), load it onto the PicoVision, and away you go!

## Code State

Thus far, the focus has been entirely on getting RP2040 Doom running. Not a lot of time has been 
spent 
cleaning 
the code up. There are a bunch of defunct `#ifdefs` and other code that was useful at some point, 
but no longer are, and indeed changing them may result in non-functional code. This is particularly 
true of 
the 
`whd_gen` tool 
used to 
convert/compress WADs 
who's code is 
likely completely incomprehensible!  

## Artifacts

You can find a RP2040 Doom UF2s based on the standard VGA/I2S pins in the 
releases of this repository. There are also versions with the shareware DOOM1.WAD already embedded.

Note you can always use `picotool info -a <UF2 file>` to see the pins used by a particular build.

## Goals

The main goals for the RP2040 port were:

1. Everything should match the original game experience, i.e. all the graphics at classic 320x200 resolution, stereo
   sound,
   OPL2 music, save/load, demo playback, cheats, network multiplayer... basically it should feel like the original game.
2. `DOOM1.WAD` should run on a Raspberry Pi Pico. There was also to be no sneaky discarding of splash screens, altering of levels, down-sampling of
   textures or whatever. RP2040 boards with 8M should be able to play at least the full *Ultimate Doom* and *DOOM II*
   WADs.
3. The RP2040 should output directly to VGA (16 color pins for RGB565 along with HSync/VSync - now replaced by RGB777 digital video output) along with stereo sound

## Results

PicoVision version features:

* Shareware `DOOM1.WAD`, *Ultimate Doom* and *Doom II* are playable.
* 320x200 DV output (really 720x400@70Hz), 21-bit colour.
* 9 Channel OPL2 Sound at 49716Hz.
* 9 Channel Stereo Sound Effects.
* I2C networking for up to 4 players.
* Save/Load of games.
* All cheats supported.
* Demos from original WADs run correctly.
* USB Keyboard Input support.
* All end scenes, intermissions, help screens etc. supported. (Note end scene likely currently broken - I haven't tested it)
* Good frame rate; generally 25-35 FPS.
* Uses 266Mhz overclock

# Building

RP2040 Doom should build fine on Linux and macOS. The RP2040 targeting builds should also work on Windows, though I 
haven't tried.

The build uses `CMake`.

## Regular chocolate-doom/native builds

To build everything, assuming you have SDL2 dependencies installed, you can create a build directory:

```bash
mkdir build
cd build
cmake ..
```

And then run `make` or `make -j<num_cpus>` from that directory. To build a particular target e.g. `chocolate-doom`, 
do `make chocolate-doom`

Note this is the way you build the `whd_gen` tool too.

## RP2040 Doom builds

You must have [pico-sdk](https://github.com/raspberrypi/pico-sdk) and 
**the latest version of** [pico-extras](https://github.com/raspberrypi/pico-extras) installed, along with the regular 
pico-sdk requisites (e.g.
`arm-none-eabi-gcc`). If in doubt, see the Raspberry Pi
[documentation](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).

**NOTE: I have been testing with arm-none-eabi-gcc 9.3.1 .. it seems like other versions may cause problems with binary 
size, so use [gcc 9](https://developer.arm.com/downloads/-/gnu-rm) for now.** 

You can create a build directly like this:

```bash
mkdir rp2040-build
cd rp2040-build
cmake -DPICO_BOARD=pimoroni_picovision -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_EXTRAS_PATH=/path/to/pico-extras -DCMAKE_C_COMPILER:FILEPATH=/path/to/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc -DCMAKE_CXX_COMPILER:FILEPATH=/path/to/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-g++ ..
```

The Pimoroni Picovision board file is in this repo for now, copy it to `pico-sdk/src/boards/include/boards`.

As before, use `make` or `make <target>` to build. 

The RP2040 version has four targets, each of which create a similarly named `UF2` file (e.g. `doom_tiny.uf2`). 
These UF2 files contain the executable code/data, but they do not contain the WAD data which is converted into a 
RP2040 Doom 
specific WHD/WHX format by `whd_gen` (for more see below). The WHD/WHX file must also be loaded onto the device at a 
specific address which varies by binary. 

"super-tiny" refers to RP2040 Doom builds that use the more compressed WHX format, and 
required for`DOOM1.
WAD` to 
run 
on a 2M Raspberry Pi Pico. "Non super-tiny" refers to RP2040 Doom builds that use the WHD format which is larger, but 
also is 
required for *Ultimate Doom* and *Doom II* WADs. These binaries are distinct as supporting both formats in the same 
binary would just have made things bigger and slower.


* **doom_tiny** This is a "super tiny" version with no USB keyboard support. You can use
[SDL Event Forwarder](https://github.com/kilograham/sdl_event_forwarder) to tunnel keyboard input from your host 
  computer over UART.
* **doom_tiny_usb** This is a "super tiny" version with additional USB keyboard support.
* **doom_tiny_nost** This is a "non super tiny" version of `doom_tiny` supporting larger WADs stored as WHD.
* **doom_tiny_nost_usb** This is a "non super tiny" version of `doom_tiny_usb` supporting larger WADs stored as 
  WHD.

PicoVision loads the WAD from the SD card, so you need to copy your whd or wxd file into the root directory of the SD card, named doom1.whd/wxd.  The maximum WHD size is currently 7.5MB.

For PicoVision, there is no particular need to use the super tiny version - you can always use `doom_tiny_nost_usb`.

See `whd_gen` further below for generating `WHX` or `WHD` files.

#### USB keyboard support

Note that TinyUSB host mode support for keyboard may not work with all keyboards especially since the RP2040 Doom 
has been built with small limits for number/sizes of hubs etc. I know that Raspberry Pi keyboards work fine, as 
did my ancient 
Dell keyboard. Your keyboard may just do nothing, or may cause a crash. If so, for now, you are stuck forwarding 
keys from another PC via sdl_event_forwarder.

### RP2040 Doom builds not targeting an RP2040 device

You can also build the RP2040 Doom to run on your host computer (Linux or macOS) by using
[pico_host_sdl](https://github.com/raspberrypi/pico-host-sdl) which simulates RP2040 based video/audio output using SDL.

This version currently embeds the WHD/WHX in `src/tiny.whd.h` so you must generate this file.

You can do this via `./cup.sh <whd/whx_file>`

```bash
mkdir host-build
cd host-build
cmake -DPICO_PLATFORM=host -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_EXTRAS_PATH=/path/to/pico-extras -DPICO_SDK_PRE_LIST_DIRS=/path/to/pico_host_sdl ..
```

... and then `make` as usual.

## whd_gen

`doom1.whx` is included in this repository, otherwise you need to build `whd_gen` using the regular native build 
instructions above.

To generate a WHX file (you must use this to convert DOOM1.WAD to run on a 2M Raspberry Pi Pico)

```bash
whd_gen <wad_file> <whx_file>
```

The larger WADs (e.g. *Ultimate Doom* or *Doom II*) have levels which are too complex to convert into a super tiny 
WHX file. These larger WADs are not going to fit in a 2M flash anywy, so the less compressed WHD format can be used 
given that the device now probably has 8M of flash.

```bash
whd_gen <wad_file> <whd_file> -no-super-tiny
```

Note that `whd_gen` has not been tested with a wide variety of WADs, so whilst it is possible that non Id WADs may 
work, it is by no means guaranteed!

NOTE: You should use a release build of `whd_gen` for the best sound effect fidelity, as the debug build 
deliberately lowers the encoding quality for the sake of speed.

# Future

*Evilution* and *Plutonia* are not yet supported. There is an issue tracking it 
[here](https://github.com/kilograham/rp2040-doom/issues/1).

# RP2040 Doom Licenses

* Any code derived from chocolate-doom matinains its existing license (generally GPLv2). 
* New RP2040 Doom specific code not implementing existing chocolate-doom interfaces is licensed BSD-3.
* ADPCM-XA is unmodified and is licensed BSD-3.
* Modified emu8950 derived code retains its MIT license.

