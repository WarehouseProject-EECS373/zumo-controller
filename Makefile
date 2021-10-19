all : build debug flash
.PHONY: build debug flash

build :
	cmake -Bbuild && $(MAKE) -C build
	
debug :
	cmake -DCMAKE_BUILD_TYPE=Debug -Bdebug && $(MAKE) -C debug

flash :
	st-flash write debug/zumo-controller.bin 0x08000000
