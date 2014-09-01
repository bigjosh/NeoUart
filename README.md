NeoUart
=======

Drives a WS2812B NeoPixel connected to Raspberry Pi pin P1-08

Connect a single NeoPixel to your PI as shown in the included image. 


Syntax:

Accepts pixelspecs in the format [DD]RRGGBB where...
[DD] is an optional durration in 1/100s of seconds, and
and RR, GG, and BB are the briness level for red, green, and blue respecively.

All values are hex numbers in the range 00-ff.

Examples of pixelspecs:
000000=Black, 05FFFF=white for .05s, 800080=50% blue for 1.28s

You can specify one or more pixelspecs on the command line, or run the
program without parameters and it will read and execute pixelspecs from
stdin.

Example:

neouart <demos/disco.nua

