from common import *
sw=connect()
try:
    parts=[('bearing_627',7,22,7,True,'627 bearing envelope'),('bearing_6804',20,32,7,True,'6804 bearing envelope'),
           ('bearing_687',7,14,5,True,'687 bearing envelope'),('pivot_shim',7.2,11,.5,False,'inner pivot shim'),
           ('pivot_outer_spacer',7.2,11,2.5,False,'outer pivot spacer'),('washer_M3',3.2,7,.5,False,'M3 washer')]
    for spec in parts:ring(sw,*spec)
finally:
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
