############################################################################
#
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ATLFLight nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

all: dspal_tester

.PHONY: ENV_VARS
ENV_VARS:
	@[ ! -z "${HEXAGON_SDK_ROOT}" ] || (echo "HEXAGON_SDK_ROOT not set" && false)
	@[ ! -z "${HEXAGON_TOOLS_ROOT}" ] || (echo "HEXAGON_TOOLS_ROOT not set" && false)
	@[ `which arm-linux-gnueabihf-gcc` ] || (echo "Add ${HEXAGON_SDK_ROOT}/gcc-linaro-arm-linux-gnueabihf-4.8-2013.08_linux/bin to PATH" && false)

cmake_hexagon/bundle.cmake:
	cd ../../ && git submodule init
	cd ../../ && git submodule update

.PHONY: dspal_tester
dspal_tester: ENV_VARS cmake_hexagon/toolchain/Toolchain-qurt.cmake
	mkdir -p build && cd build && cmake -Wno-dev .. -DCMAKE_TOOLCHAIN_FILE=cmake_hexagon/toolchain/Toolchain-qurt.cmake
	cd build && make
	
load: dspal_tester
	cd build && make dspal_tester-load

clean:
	rm -rf build
