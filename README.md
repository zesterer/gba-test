Written in Rust, of course. I didn't end up getting round to implementing texture mapping.

[![Software Rasterisation on the GBA](https://img.youtube.com/vi/qpUQId1efQU/0.jpg)](https://www.youtube.com/watch?v=qpUQId1efQU)

The GBA has a 16 MHz processor and a 240x160 screen. With mode 5 rendering and a graphics hardware scaling hack, you can
do full-screen 16-bit colour rendering (with an internal resolution of 160x128). The GBA doesn't have a GPU, so
everything needs to be done with software. I wrote an optimised rasteriser that supports blitting N-gons to the screen,
along with a full rendering pipeline to go with it. At 16 MHz and with a screen with of 160x128, you only get
`16000000 / (60 * 160 * 28) = 60` cycles for each pixel, and that's not even considering the world required to transform
vertices, clear the screen for the next frame, listen to user inputs, etc. In practice, most instructions require
several cycles and the GBA has no hardware parallelism (except for some very primitive block memory copy hardware). All
things considered, the frame budget can dip down to about ~20 instructions per pixel. In addition, the GBA has no
floating-point instructions so everything needs to be done using fixed-point maths. A fun challenge!
