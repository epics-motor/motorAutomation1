@echo off

call dllPath.bat

..\..\bin\%EPICS_HOST_ARCH%\automation1.exe st.cmd

REM Allow error messages to be read before the cmd window disappears
pause

