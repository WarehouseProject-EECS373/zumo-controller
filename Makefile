.PHONY: purge clean build debug flash format

build:
	cmake -DCMAKE_MODULE_PATH=../modules/stm32-cmake/cmake -DCMAKE_TOOLCHAIN_FILE=../modules/stm32-cmake/cmake/gcc_stm32.cmake -Bbuild && $(MAKE) -C build
	
debug:
	cmake -DCMAKE_MODULE_PATH=../modules/stm32-cmake/cmake -DCMAKE_TOOLCHAIN_FILE=../modules/stm32-cmake/cmake/gcc_stm32.cmake -DCMAKE_BUILD_TYPE=Debug -Bdebug && $(MAKE) -C debug

purge:
	rm -rf build/ debug/

clean:
	$(MAKE) clean -C debug
	$(MAKE) clean -C build

flash:
	st-flash write debug/zumo-controller.bin 0x08000000
	st-flash reset

format:
	./tools/format.sh
