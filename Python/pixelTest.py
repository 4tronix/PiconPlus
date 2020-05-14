#! /usr/bin/env python
# Picon Plus Fireled Test
# Flashes Fireleds attached to Fireled Output
# Press Ctrl-C to stop

from __future__ import print_function
from piconplus import PiconPlus
from time import sleep

pp = PiconPlus(0x24)

rev = pp.getRevision()
print ('Board:', rev[1], "  Revision:", rev[0])

for i in range(7):
    pp.setPixel(i, 255, 0, 0)
    sleep(0.5)

try:
    while True:
        pp.setAllPixels(255, 0, 0)
        sleep(0.5)
        pp.setAllPixels(0, 255, 0)
        sleep(0.5)
        pp.setAllPixels(0,0,255)
        sleep(0.5)
        pp.setAllPixels(255,255,255)
        sleep(0.5)
        pp.setAllPixels(0,0,0)
        sleep(0.5)
except KeyboardInterrupt:
    print()
finally:
    pp.cleanup()

