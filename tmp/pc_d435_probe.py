from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/v11 sil'))
def one(filtered):
    from bridge_core import CoreConfig,toy_objects
    import arm_objects
    objs=toy_objects(CoreConfig(payload='arm',n_objects=6,seed=0))
    original=arm_objects.scatter
    arm_objects.scatter=lambda *a,**k:objs
    from sil_pick import SilPick
    g=SilPick(6,seed=0,world='house',depth_camera='d435')
    arm_objects.scatter=original
    if filtered:
        from omni_arm import floor_vision as fv
        import numpy as np
        detect=fv.detect
        def median(frame,*a,**k):
            padded=np.pad(frame.depth,1,mode='edge')
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter('ignore',RuntimeWarning)
                depth=np.nanmedian(np.lib.stride_tricks.sliding_window_view(padded,(3,3)),axis=(-2,-1))
            depth[~np.isfinite(frame.depth)]=np.nan
            return detect(fv.DepthFrame(frame.t,depth,frame.K,frame.T_base_cam),*a,**k)
        fv.detect=median
    result=g.run(400)
    g.cams.close()
    Path(__file__).with_name('pc_d435_probe_'+str(filtered)+'.json').write_text(json.dumps(result,indent=1))
    print(filtered,result,flush=True)
if __name__=='__main__':
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=2) as p:list(p.map(one,[False,True]))
