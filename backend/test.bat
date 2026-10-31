@echo off
rem Builds and runs the native C self-test (needs gcc, e.g. MinGW-w64 or MSYS2).
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 vehicle.c engine.c transmission.c simulation.c main.c -o simtest.exe -lm
if errorlevel 1 exit /b 1
simtest.exe
