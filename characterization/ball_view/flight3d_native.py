"""Optional compiled twin of the numpy 3D fitter; cache outside the checkout."""
import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import numpy as np

_lib=[]


def load():
    if _lib:
        return _lib[0]
    src=Path(__file__).with_suffix('.cpp')
    cache=Path.home()/'.cache/omni_bucket'/hashlib.sha256(src.read_bytes()).hexdigest()[:16]
    cache.mkdir(parents=True,exist_ok=True)
    out=cache/('fit.dll' if sys.platform=='win32' else 'fit.so')
    try:
        if not out.exists():
            flags=['-std=c++17','-O3','-shared','-fno-exceptions','-fno-rtti']
            cmd=([sys.executable,'-m','ziglang','c++','-target','x86_64-windows-gnu'] if sys.platform=='win32'
                 else ['g++','-fPIC','-march=native'])
            tmp=out.with_name(out.name+f'.{os.getpid()}.tmp')
            subprocess.run(cmd+flags+[str(src),'-o',str(tmp)],check=True,capture_output=True)
            os.replace(tmp,out)
        lib=ctypes.CDLL(str(out))
        ptr=ctypes.POINTER(ctypes.c_double)
        lib.bucket_fit.argtypes=[ptr,ctypes.c_int,ptr,ptr,ctypes.c_double,ptr]
        lib.bucket_fit.restype=ctypes.c_int
        lib.bucket_path.argtypes=[ptr,ctypes.c_int,ptr,ctypes.c_int,ptr,ctypes.c_double,ptr]
        lib.bucket_path.restype=None
        _lib.append(lib)
    except (OSError,subprocess.CalledProcessError) as e:
        print(f'Bucket native fitter unavailable: {e}',file=sys.stderr)
        _lib.append(None)
    return _lib[0]


def solve(obs,intr,gravity,drag):
    lib=load()
    if lib is None:
        return None
    ptr=ctypes.POINTER(ctypes.c_double)
    arrays=[np.ascontiguousarray(x,dtype=np.float64) for x in (obs,intr,gravity)]
    out=np.zeros(7)
    ok=lib.bucket_fit(arrays[0].ctypes.data_as(ptr),len(obs),arrays[1].ctypes.data_as(ptr),
                      arrays[2].ctypes.data_as(ptr),drag,out.ctypes.data_as(ptr))
    return (out[:6],float(out[6])) if ok else None


def path(states,times,gravity,drag):
    lib=load()
    if lib is None:
        return None
    ptr=ctypes.POINTER(ctypes.c_double)
    ss=np.ascontiguousarray(np.atleast_2d(states),dtype=np.float64)
    ts=np.ascontiguousarray(times,dtype=np.float64)
    gg=np.ascontiguousarray(gravity,dtype=np.float64)
    out=np.empty((len(ss),len(ts),3))
    lib.bucket_path(ss.ctypes.data_as(ptr),len(ss),ts.ctypes.data_as(ptr),len(ts),
                    gg.ctypes.data_as(ptr),drag,out.ctypes.data_as(ptr))
    return out[0] if np.asarray(states).ndim==1 else out
