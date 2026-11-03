"""Capture v9 constructive operations, never convert a BREP to an imported feature.

Run with the omni-robot-sim interpreter. Exports a portable JSON recipe and an
independent STEP reference per part. The SolidWorks emitter uses cadenv.
"""
import sys, json, math, functools, contextlib
from pathlib import Path
import numpy as np
import build123d as b

ROOT=Path(__file__).resolve().parents[1]
V9=ROOT.parent/'v9'
sys.path.insert(0,str(V9))
REG={}; NODES=[]; DEPTH=0

def matrix(loc):
    t=loc.wrapped.Transformation()
    return [[t.Value(i,j) for j in range(1,5)] for i in range(1,4)]+[[0,0,0,1]]

def record(obj,node):
    if obj is not None and getattr(obj,'_dim',None)==3:node['volume_mm3']=obj.volume
    node['id']=len(NODES);NODES.append(node);REG[id(obj)]=(obj,node['id'])
    return obj

def node(obj):
    x=REG.get(id(obj))
    if not x: raise RuntimeError('Unrecorded '+str(type(obj)))
    return x[1]

def guarded(fn,*args,**kw):
    global DEPTH
    DEPTH+=1
    try:return fn(*args,**kw)
    finally:DEPTH-=1

def primitive(cls,kind):
    @functools.wraps(cls)
    def call(*args,**kw):
        result=guarded(cls,*args,**kw)
        # Recreate the primitive's pre-rotation, aligned local frame.
        dims=list(args[:3 if kind=='box' else (3 if kind=='cone' else 2)])
        if kind=='box':lo=np.array([0.,0.,0.]);hi=np.array(dims,float)
        else:
            r=max(dims[:2]) if kind=='cone' else dims[0]
            lo=np.array([-r,-r,0.]);hi=np.array([r,r,dims[-1]],float)
        align=kw.get('align',(b.Align.CENTER,)*3)
        if not isinstance(align,tuple):align=(align,)*3
        shift=[(-lo[k] if a==b.Align.MIN else -hi[k] if a==b.Align.MAX else -(hi[k]+lo[k])/2) for k,a in enumerate(align)]
        rot=kw.get('rotation',(0,0,0));rot=b.Rot(*rot) if isinstance(rot,tuple) else rot
        trans=matrix(rot*b.Pos(*shift))
        return record(result,{'op':kind,'dims':dims,'matrix':trans})
    return call

for name,kind in [('Box','box'),('Cylinder','cylinder'),('Cone','cone')]:
    setattr(b,name,primitive(getattr(b,name),kind))

def bool_wrapper(fn,op):
    @functools.wraps(fn)
    def call(self,other):
        capture=DEPTH==0 and id(self) in REG and id(other) in REG
        result=guarded(fn,self,other)
        if capture:
            if op=='sub' and abs(result.volume-self.volume)<1e-6:
                REG[id(result)]=(result,node(self))
            else:
                data={'op':op,'a':node(self),'b':node(other)}
                data['a_centers']=[list(s.center()) for s in sorted(self.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))]
                data['b_centers']=[list(s.center()) for s in sorted(other.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))]
                if op=='add':
                    aa=sorted(self.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    bb=sorted(other.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    pieces=aa+bb;groups=[{i} for i in range(len(pieces))]
                    for i in range(len(aa)):
                        for j in range(len(aa),len(pieces)):
                            if pieces[i].distance_to(pieces[j])<1e-5:
                                g=next(g for g in groups if i in g);h=next(g for g in groups if j in g)
                                if g is not h:g.update(h);groups.remove(h)
                    data['groups']=[sorted(g) for g in groups]
                if op=='sub':
                    aa=sorted(self.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    bb=sorted(other.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    data['cut_pairs']=[]
                    if len(aa)>1 or len(bb)>1:
                        for i,a in enumerate(aa):
                            for j,b_ in enumerate(bb):
                                hit=a&b_
                                if hit is not None and hit.volume>1e-7:data['cut_pairs'].append([i,j])
                if op=='common':
                    aa=sorted(self.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    bb=sorted(other.solids(),key=lambda s:tuple(round(v,5) for v in s.center()))
                    data['common_pairs']=[]
                    if len(aa)>1 or len(bb)>1:
                        for i,a in enumerate(aa):
                            for j,b_ in enumerate(bb):
                                hit=a&b_
                                if hit is not None and hit.volume>1e-7:data['common_pairs'].append([i,j])
                record(result,data)
        return result
    return call

for cls in (b.Shape,b.Compound):
    for method,op in [('__add__','add'),('__sub__','sub'),('__and__','common')]:
        if method in cls.__dict__:setattr(cls,method,bool_wrapper(getattr(cls,method),op))

_moved=b.Shape.moved
def moved(self,loc):
    capture=DEPTH==0 and id(self) in REG
    result=guarded(_moved,self,loc)
    if capture:record(result,{'op':'move','a':node(self),'matrix':matrix(loc.location if isinstance(loc,b.Plane) else loc)})
    return result
b.Shape.moved=moved

def loops(shape):
    face=shape.faces()[0]
    out=[]
    for wire in [face.outer_wire(),*face.inner_wires()]:
        pts=[]
        for e in wire.order_edges():
            if e.geom_type!=b.GeomType.LINE:raise RuntimeError('Nonlinear profile '+str(e.geom_type))
            pts.append(list(e.position_at(0)))
        out.append(pts)
    return out

_extrude=b.extrude
def extrude(shape,amount=None,**kw):
    result=guarded(_extrude,shape,amount,**kw)
    d=list(kw.get('dir',shape.faces()[0].normal_at()))
    return record(result,{'op':'extrude','loops':loops(shape),'amount':amount,'dir':d,'both':kw.get('both',False)})
b.extrude=extrude
_revolve=b.revolve
def revolve(shape,axis=b.Axis.Z,**kw):
    result=guarded(_revolve,shape,axis,**kw)
    return record(result,{'op':'revolve','loops':loops(shape),'axis_origin':list(axis.position),'axis_dir':list(axis.direction)})
b.revolve=revolve

def edge_sig(e):
    from OCP.BRep import BRep_Tool
    parent=e.topo_parent
    seam=any(BRep_Tool.IsClosed_s(e.wrapped,f.wrapped) for f in parent.faces()) if parent is not None else False
    return {'points':[list(e.position_at(t)) for t in (.137,.413,.731)],'length':e.length,'kind':e.geom_type.name,'seam':seam}

def edge_op(fn,op):
    def call(edges,radius,*args,**kw):
        es=list(edges) if not isinstance(edges,b.Edge) else [edges]
        # build123d edges carry their owning solid in topological_parent.
        parent=es[0].topo_parent
        while parent is not None and id(parent) not in REG:parent=getattr(parent,'topo_parent',None)
        if parent is None:raise RuntimeError('Cannot find edge owner')
        result=guarded(fn,edges,radius,*args,**kw)
        return record(result,{'op':op,'a':node(parent),'radius':radius,'edges':[edge_sig(e) for e in es]})
    return call
b.fillet=edge_op(b.fillet,'fillet');b.chamfer=edge_op(b.chamfer,'chamfer')

def write_part(name,shape):
    path=ROOT/'reference'/(name+'.step');b.export_step(shape,path)
    data={'name':name,'root':node(shape),'nodes':NODES,'volume_mm3':shape.volume,'solids':len(shape.solids())}
    (ROOT/'reference'/(name+'.json')).write_text(json.dumps(data))
    print(name,shape.volume,'nodes',len(NODES),flush=True)

if __name__=='__main__':
    import pod, wheel, hw
    import pod_design
    targets={'fork_bar':pod.fork_bar,'pivot_shaft':pod.shaft,'motor_cup':pod.motor_cup,
             'pod_mount':pod.pod_mount,'wheel_guard':pod.wheel_guard,
             'roller_core':wheel.roller_core,'roller_tread':wheel.roller_tread,
             'wheel_cartridge':wheel.bearing_cartridge,
             'wheel_hub_A':lambda:wheel.hub('A'),'wheel_hub_B':lambda:wheel.hub('B'),'wheel_hub_C':lambda:wheel.hub('C')}
    targets.update(pod_design.PARTS)
    import vendor
    targets['ak40_reference']=vendor.ak40_10
    for name in (sys.argv[1:] or targets):write_part(name,targets[name]())
