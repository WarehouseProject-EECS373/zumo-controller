all : clean build debug flash
.PHONY: clean build debug flash

fullclean :
	rm -rf build/ debug/

clean :
	$(MAKE) clean -C debug
	$(MAKE) clean -C build

build :
	cmake -Bbuild && $(MAKE) -C build
	
debug :
	cmake -DCMAKE_BUILD_TYPE=Debug -Bdebug && $(MAKE) -C debug

flash :
	st-flash write debug/zumo-controller.bin 0x08000000
