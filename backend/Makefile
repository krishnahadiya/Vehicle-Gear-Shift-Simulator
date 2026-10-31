# Vehicle Gear Shift Simulator - backend build
# Usage (Linux/macOS/MSYS2/MinGW):  make test   |  make wasm  |  make clean
# On plain Windows use build.bat and test.bat instead.

SRC      = vehicle.c engine.c transmission.c simulation.c
CFLAGS   = -std=c99 -Wall -Wextra -Wpedantic -O2
EXPORTS  = -sMODULARIZE=1 -sEXPORT_NAME=createSimModule -sALLOW_MEMORY_GROWTH=1 \
           -sEXPORTED_RUNTIME_METHODS=cwrap,ccall,UTF8ToString -sNO_EXIT_RUNTIME=1

test:
	gcc $(CFLAGS) $(SRC) main.c -o simtest -lm
	./simtest

wasm:
	emcc $(SRC) main.c -O2 $(EXPORTS) -o ../frontend/sim.js

clean:
	rm -f simtest simtest.exe session_report.txt best_score.dat ../frontend/sim.js ../frontend/sim.wasm
