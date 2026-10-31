@echo off
rem Compiles the C backend to WebAssembly. Run from the backend folder in a
rem terminal where the Emscripten environment is active (emsdk_env.bat).
where emcc >nul 2>nul
if errorlevel 1 (
  echo emcc was not found. Run "emsdk_env.bat" from your emsdk folder first.
  exit /b 1
)
emcc vehicle.c engine.c transmission.c simulation.c main.c -O2 ^
  -sMODULARIZE=1 -sEXPORT_NAME=createSimModule -sALLOW_MEMORY_GROWTH=1 ^
  -sEXPORTED_RUNTIME_METHODS=cwrap,ccall,UTF8ToString -sNO_EXIT_RUNTIME=1 ^
  -o ..\frontend\sim.js
if errorlevel 1 (
  echo Build failed.
  exit /b 1
)
echo Built ..\frontend\sim.js and ..\frontend\sim.wasm
