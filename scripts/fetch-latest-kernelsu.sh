#!/bin/bash

if [ ! -d "drivers/kernelsu" ]; then
	git clone https://github.com/KernelSU-Next/KernelSU-Next
	cp -r KernelSU-Next/kernel drivers/kernelsu
	rm -rf KernelSU-Next
	sed -i 's/#source \"drivers\/kernelsu\/Kconfig\"/source \"drivers\/kernelsu\/Kconfig\"/g' drivers/Kconfig
else
	exit 0
fi
