#! /usr/bin/env python

# GNU GPL V3
# Test code for 4tronix Picon Plus

from __future__ import print_function
from piconplus import PiconPlus

pp = PiconPlus(0x25)
vsn = pp.getRevision()
if (vsn[1] == 4):
    print ("Board Type:", "Picon Plus")
else:
    print ("Board Type:", vsn[1])
print ("Firmware version:", vsn[0])
print()
pp.cleanup()

