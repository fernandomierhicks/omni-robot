"""Readable material colours, including magenta stock fasteners."""
from sw_native import *

def style(sw,path):
    d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0);m=wrap(d,sld.IModelDoc2);name=path.stem.removeprefix('V10_')
    colour=([.86,.08,.58] if name.startswith(('bolt_','nut_','washer_')) else
            [.58,.63,.69] if name.startswith('bearing_') else [.10,.12,.15] if name=='roller_tread' else
            [.96,.50,.12] if name.startswith('dummy_motor_') else [.36,.48,.62] if name.startswith('shock_') else [.88,.90,.93])
    m.MaterialPropertyValues=arr(colour+[1.,.5,.25,.15,0.,0.])
    ok,e,w=m.Save3(1,0,0)
    if not ok or e:raise RuntimeError('Style save failed '+path.name)
    sw.CloseDoc(m.GetTitle());print('Styled',path.name,flush=True)

if __name__=='__main__':
    sw=connect()
    paths=[ROOT/'parts'/('V10_'+n+'.SLDPRT') for n in sys.argv[1:]] if sys.argv[1:] else sorted((ROOT/'parts').glob('V10_*.SLDPRT'))
    for path in paths:
        if 'probe' not in path.stem and 'master_layout' not in path.stem:style(sw,path)
