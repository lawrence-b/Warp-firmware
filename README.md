# Warp
Baseline firmware for the Warp hardware platform.


## Prerequisites
You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake`.


## Building the Warp firmware
First, make sure the environment variable `ARMGCC_DIR` is set correctly. if your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`

	setenv ARMGCC_DIR <full path to the directory containing bin/arm-none-eabi-gcc>

Second, edit the jlink command file, `tools/scripts/jlink.commands` to include the correct path.

Third, you should be able to build the Warp firmware by

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. See 	`Warp/src/boot/ksdk1.1.0/README.md` for more.

Fourth, you will need two terminal windows. In one, run the firmware downloader. On MacOS, this will be

	/Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 4000 -CommanderScript ../../tools/scripts/jlink.commands

In the other, launch the JLink RTT client. On MacOS, this will be

	/Applications/SEGGER/JLink/JLinkRTTClient


## Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c`.
