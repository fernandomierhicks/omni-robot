"""Native editable representations of stock hardware used by the pod."""
import capture_recipe as R
import pod_design as V
import hw

if __name__=='__main__':
    for size,length in [('M3',v) for v in (8,10,12,14,16,18,20,25,50)]+[('M2.5',6),('M2.5',8),('M2.5',10)]:
        name=f'bolt_{size.replace(".","p")}_{length}_PAN'
        R.write_part(name,hw._screw_shape('PAN',length,size)[0])
    R.write_part('bolt_M3_12_FLAT',hw._screw_shape('FLAT',12,'M3')[0])
    R.write_part('nut_M3_hex',hw._nut_shape('hex'))
    R.write_part('nut_M3_nyloc',hw._nut_shape('nyloc'))
    R.write_part('washer_M3',hw._washer_shape())
