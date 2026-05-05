
Why image_view did not show your sim
------------------------------------
image_view.exe does NOT open output.bmp. It only displays frames your program
sends through the image_transfer shared-memory API (view_rgb_image), the same
way as Week7 bluetooth_robot_1.1_vision.

What was wrong
--------------
program.cpp only wrote output.bmp to disk. image_view never reads that file,
so its window stayed empty or stale.

How to get the sim on image_view
--------------------------------
1. Copy image_transfer.lib from your course Week7 project (bluetooth_robot
   windows_program folder or SDK path used there) into this folder or add its
   directory to the linker path.

2. Build with USE_IMAGE_VIEW so the composited frame is pushed to image_view:
   - Visual Studio: add preprocesser define USE_IMAGE_VIEW; link image_transfer.lib
     (and if needed: strmbasd.lib mfplat.lib mf.lib mfreadwrite.lib per Week7).
   - Command line (MSVC), example:
     cl /EHsc /O2 /DUSE_IMAGE_VIEW program.cpp user32.lib image_transfer.lib

3. Start image_view.exe FIRST (from Project\2D_Sim\image_view or Week7 copy),
   then run program.exe from the same folder as background.bmp / robot_A.bmp /
   robot_B.bmp.

4. Optional: keep writeFrame() – it still writes output.bmp for debugging or
   opening in Paint; image_view uses the shared-memory path only.

High-DPI
--------
If the image_view window is huge, use image_view_readme.txt steps (Compatibility
> Disable display scaling on high DPI).

Without image_transfer.lib
--------------------------
Leave USE_IMAGE_VIEW undefined and open output.bmp manually after run, or use
any BMP viewer; it will not auto-refresh each frame unless you use image_view
with the lib.

LNK4272 x86 vs x64
------------------
If the linker says image_transfer.lib machine type 'x86' conflicts with 'x64',
the library is 32-bit. Build with the x86 toolchain only:
  - Open "x86 Native Tools Command Prompt for VS" (not x64), cd to 2D_Sim, run build_msvc_image_view.bat
  - Or run vcvars32.bat then cl ...
Your program.exe will be 32-bit; image_view.exe from the course is also 32-bit.

g++ vs MSVC
-----------
- g++ cannot reliably link image_transfer.lib (MSVC static lib / ABI). Use g++
  for the BMP-only build: build_g++.bat or
    g++ -O2 -std=c++17 program.cpp -o program.exe -luser32
- For image_view live display you must build with cl + image_transfer.lib.
  "cl" is not in normal PowerShell -- open "x64 Native Tools Command Prompt
  for VS" (or run vcvars64.bat), cd to 2D_Sim, then build_msvc_image_view.bat
