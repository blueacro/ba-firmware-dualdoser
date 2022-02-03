.PHONY: all app bootloader tool clean

all: bootloader app tool bundle
clean:
	rm -rf build
	make -C bootloader/make clean
	make -C app clean
	make -C bootloader/dx1elf2dfu clean

bootloader:
	make -C bootloader/make

tool:
	make -C bootloader/dx1elf2dfu

app:
	make -C app

bundle: app tool bootloader
	mkdir -p build
	cp app/build/*.elf build
	cp app/build/*.hex build
	cp bootloader/make/build/*.elf build
	bootloader/dx1elf2dfu/dx1elf2dfu build/dualdoser.elf build/dualdoser.dfu
