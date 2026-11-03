"""Hardware the two-part dummy motor needs beyond the stock list: an M3 large (DIN 9021) washer, 9 mm OD.
The ordinary M3 washer (7 mm OD) is no bigger than the 687 bore, so it cannot hold the inner race."""
from common import *
sw=connect()
try:ring(sw,'washer_M3_large',3.2,9,.8,False,'M3 large washer (DIN 9021)')
finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
