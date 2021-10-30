all : clean build debug flash
.PHONY: clean build debug flash

fullclean :
	rm -rf build/ debug/

clean :
	$(MAKE) clean -C debug
	$(MAKE) clean -C build

build :
	cmake -DCMAKE_MODULE_PATH=../modules/stm32-cmake/cmake -DCMAKE_TOOLCHAIN_FILE=../modules/stm32-cmake/cmake/gcc_stm32.cmake -Bbuild && $(MAKE) -C build
	
debug :
	cmake -DCMAKE_MODULE_PATH=../modules/stm32-cmake/cmake -DCMAKE_TOOLCHAIN_FILE=../modules/stm32-cmake/cmake/gcc_stm32.cmake -DCMAKE_BUILD_TYPE=Debug -Bdebug && $(MAKE) -C debug

flash :
	st-flash write debug/zumo-controller.bin 0x08000000
