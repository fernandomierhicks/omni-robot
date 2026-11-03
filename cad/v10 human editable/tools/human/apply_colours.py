"""Apply 15_finalize_parts' palette to every managed part file (appearance only: no rebuild, no STEP export)."""
from common import *
from importlib import import_module
F=import_module('15_finalize_parts')
sw=connect()
keys=sys.argv[1:] or sorted(k for k in json.loads(REGISTRY.read_text()) if k!='master_layout')
for key in keys:
    m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0);F.style(m,key)
    ok,e,w=m.Save3(1,0,0)
    if not ok or e:raise RuntimeError('Cannot save '+key+' '+str((e,w)))
    sw.CloseDoc(m.GetTitle());print(key,F.colour(key),flush=True)
