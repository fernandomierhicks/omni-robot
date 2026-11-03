"""Emit captured constructive recipes as dimensioned native SolidWorks features."""
from sw_native import *
import numpy as np
from scipy.spatial.transform import Rotation

I=np.eye(4)

def darray(bs):return wc.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_DISPATCH,[b._oleobj_ for b in bs])

class Emitter(NativePart):
    def __init__(self,sw,data):
        super().__init__(sw,data['name']);self.data=data;self.nodes=data['nodes'];self.count=0
    def named(self):return {b.Name:b for b in bodies(self.model)}
    def tag_new(self,before,label):
        out=[]
        for b in bodies(self.model):
            if b.Name not in before:
                self.count+=1;b.Name=f'body_{self.count:04d}_{label}';out.append(b.Name)
        if not out: raise RuntimeError('No new body: '+label)
        return self.sort_names(out)
    def sort_names(self,names):
        bs=self.named()
        return sorted(names,key=lambda n:tuple(round(v*1000,5) for v in bs[n].GetMassProperties(1)[:3]))
    def local_sort(self,names,T):
        bs=self.named();inv=np.linalg.inv(T)
        return sorted(names,key=lambda name:tuple(round(v,5) for v in (inv@np.array([v*1000 for v in bs[name].GetMassProperties(1)[:3]]+[1]))[:3]))
    def reference_order(self,names,centers,T):
        from scipy.optimize import linear_sum_assignment
        if len(names)!=len(centers):raise RuntimeError('Native/source body count mismatch')
        bs=self.named();inv=np.linalg.inv(T)
        actual=np.array([(inv@np.array([v*1000 for v in bs[name].GetMassProperties(1)[:3]]+[1]))[:3] for name in names])
        cost=np.linalg.norm(np.array(centers)[:,None,:]-actual[None,:,:],axis=2)
        rows,cols=linear_sum_assignment(cost)
        if max(cost[rows,cols],default=0)>.05:raise RuntimeError('Native/source body location mismatch '+str(cost[rows,cols]))
        return [names[i] for i in cols]
    def copy_body(self,name,label):
        m=self.model;before=set(self.named());m.ClearSelection2(True)
        sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=1
        self.named()[name].Select2(False,sd)
        f=m.FeatureManager.InsertMoveCopyBody2(0,0,0,0,0,0,0,0,0,0,True,1)
        if f is None:raise RuntimeError('copy tool')
        f.Name=label+'_Copy';return self.tag_new(before,label)[0]
    def delete(self,names,label):
        if not names:return
        m=self.model;m.ClearSelection2(True)
        for i,name in enumerate(names):self.named()[name].Select2(i>0,None)
        f=m.FeatureManager.InsertDeleteBody2(False)
        if f is None:raise RuntimeError('delete unused tools')
        f.Name=label+'_RemoveTools'
    def move(self,names,T,label):
        if np.max(abs(T-I))<1e-9:return names
        m=self.model;m.ClearSelection2(True)
        sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=1
        for i,name in enumerate(names):self.named()[name].Select2(i>0,sd)
        angles=Rotation.from_matrix(T[:3,:3]).as_euler('XYZ')
        if np.max(abs(angles))>1e-9:
            # This installation maps positional rotation args Z,Y,X (verified
            # from the returned feature definition and the resulting body).
            f=m.FeatureManager.InsertMoveCopyBody2(0,0,0,0,0,0,0,*[float(v) for v in angles[::-1]],False,1)
            if f is None:raise RuntimeError('Native rotate failed: '+label)
            f.Name=label+'_Rotate'
            m.EditRebuild3();m.ClearSelection2(True)
            for i,name in enumerate(names):self.named()[name].Select2(i>0,sd)
        if np.max(abs(T[:3,3]))>1e-9:
            f=m.FeatureManager.InsertMoveCopyBody2(*[float(v)*MM for v in T[:3,3]],0,0,0,0,0,0,0,False,1)
            if f is None:raise RuntimeError('Native translate failed: '+label)
            f.Name=label+'_Translate'
            m.EditRebuild3()
        # Move feature preserves body names.
        return names
    def combine(self,op,a,bb,label):
        m=self.model;before=set(self.named())-set(a)-set(bb);bs=self.named()
        if op=='sub':
            if len(a)!=1:raise RuntimeError('Multiple main bodies')
            f=m.FeatureManager.InsertCombineFeature(c.SWBODYCUT,bs[a[0]]._oleobj_,darray([bs[n] for n in bb]))
        else:
            f=m.FeatureManager.InsertCombineFeature(c.SWBODYADD if op=='add' else c.SWBODYINTERSECT,None,darray([bs[n] for n in a+bb]))
        if f is None:raise RuntimeError('Native combine failed: '+label)
        f.Name=label
        return self.tag_new(before,label)
    def revolve(self,profile,name):
        m=self.model;sk=self.sketch(profile,name+'_Sketch',axis=([0,-100],[0,100]))
        m.ClearSelection2(True);sk.Select2(False,0)
        sketch=wrap(sk.GetSpecificFeature2(),sld.ISketch)
        axis=next(wrap(s,sld.ISketchSegment) for s in sketch.GetSketchSegments() if wrap(s,sld.ISketchSegment).ConstructionGeometry)
        sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=4;axis.Select4(True,sd)
        f=m.FeatureManager.FeatureRevolve2(True,True,False,False,False,False,0,0,math.tau,0,False,False,0,0,0,0,0,False,True,True)
        if f is None:raise RuntimeError(name+' revolve failed')
        f.Name=name
    def emit(self,index,T=I):
        n=self.nodes[index];op=n['op']; label=f'{index:04d}_{op}'
        def checked(names):
            expected=n.get('volume_mm3')
            if expected is not None:
                bs=self.named();actual=sum(bs[name].GetMassProperties(1)[3]*1e9 for name in names)
                # Integration tolerances accumulate on disconnected, trimmed
                # bodies. Final exported geometry is checked independently.
                if abs(actual-expected)>max(.5,expected*5e-5):raise RuntimeError(f'{label}: native volume differs by {actual-expected}')
            return names
        if op=='existing':return n['bodies']
        if op=='move':return self.emit(n['a'],T@np.array(n['matrix']))
        if op in ('add','sub','common'):
            a=self.emit(n['a'],T); bb=self.emit(n['b'],T)
            if 'a_centers' in n:
                a=self.reference_order(a,n['a_centers'],T);bb=self.reference_order(bb,n['b_centers'],T)
            if op=='add' and 'groups' in n:
                # Source groups are in local coordinates; sort groups by the
                # source body's centres transformed back to the local frame.
                all_names=a+bb if 'a_centers' in n else self.local_sort(a,T)+self.local_sort(bb,T);out=[]
                for g in n['groups']:
                    group=[all_names[i] for i in g]
                    out.extend(group if len(group)==1 else self.combine('add',group[:1],group[1:],label))
                return checked(self.sort_names(out))
            if op=='sub' and 'cut_pairs' in n and (len(a)>1 or len(bb)>1):
                if 'a_centers' not in n:a=self.local_sort(a,T);bb=self.local_sort(bb,T)
                out=[]
                for i,main in enumerate(a):
                    current=[main]
                    for j,tool in enumerate(bb):
                        if [i,j] in n['cut_pairs']:
                            cp=self.copy_body(tool,label)
                            current=self.combine('sub',current,[cp],label)
                    out.extend(current)
                self.delete(bb,label);return checked(self.sort_names(out))
            if op=='common' and 'common_pairs' in n and (len(a)>1 or len(bb)>1):
                if 'a_centers' not in n:a=self.local_sort(a,T);bb=self.local_sort(bb,T)
                out=[]
                for i,j in n['common_pairs']:
                    ca=self.copy_body(a[i],label);cb=self.copy_body(bb[j],label)
                    out.extend(self.combine('common',[ca],[cb],label))
                self.delete(a+bb,label);return checked(self.sort_names(out))
            return checked(self.combine(op,a,bb,label))
        if op in ('fillet','chamfer'):
            a=self.emit(n['a'],T);m=self.model;m.ClearSelection2(True)
            candidates=[]
            for name in a:
                candidates.extend(edges(self.named()[name]))
            descriptors=[]
            for edge in candidates:
                cv=curve(edge)
                if cv.IsLine():descriptors.append((edge,'line',np.array(cv.LineParams)))
                elif cv.IsCircle():descriptors.append((edge,'circle',np.array(cv.CircleParams)))
                else:descriptors.append((edge,'curve',cv))
            for sig in n['edges']:
                # OCC may include a periodic cylinder seam in the requested
                # edge set. It is not a physical corner and is not filleted.
                if sig.get('seam'):continue
                pts=[(T@np.array(p+[1]))[:3]*MM for p in sig['points']]
                def score(desc):
                    edge,kind,geometry=desc
                    if kind in ('line','circle'):
                        origin=geometry[:3];axis=geometry[3:6];delta=np.array(pts)-origin;along=delta@axis
                        perpendicular=np.linalg.norm(delta-np.outer(along,axis),axis=1)
                        distance=perpendicular if kind=='line' else np.hypot(perpendicular-geometry[6],along)
                        return float(max(distance))
                    return max(np.linalg.norm(np.array(geometry.GetClosestPointOn(*p)[:3])-p) for p in pts)
                scored=sorted([(score(d),i,d[0]) for i,d in enumerate(descriptors)],key=lambda x:x[0])
                if scored[0][0]>2e-6:raise RuntimeError('No matching native edge '+str(scored[0][0]))
                sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=1
                wrap(scored[0][2],sld.IEntity).Select4(True,sd)
            before=set(self.named())-set(a)
            if op=='fillet':f=m.FeatureManager.FeatureFillet(2,n['radius']*MM,0,0,None,None,None)
            else:
                # The direct distance-distance insertion is refused here. A
                # selection-preserving feature followed by its native data
                # definition gives a real equal-distance chamfer (not the
                # zero-distance feature returned by flag 16 alone).
                f=m.FeatureManager.InsertFeatureChamfer(4,16,0,0,n['radius']*MM,0,0,0)
                if f is not None:
                    cd=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
                    if not cd.AccessSelections(m,None):raise RuntimeError('Chamfer selection access')
                    cd.Type=2;cd.EqualDistance=True;cd.SetEdgeChamferDistance(0,n['radius']*MM);cd.SetEdgeChamferDistance(1,n['radius']*MM);cd.TangentPropagation=False
                    if not f.ModifyDefinition(cd,m,None):raise RuntimeError('Chamfer parameter definition')
            if f is None:raise RuntimeError(label+' failed')
            f.Name=label
            if op=='chamfer':
                cd=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
                if cd.Type==0 or cd.GetEdgeChamferDistance(0)<=0:raise RuntimeError('Empty chamfer definition')
            return checked(self.tag_new(before,label))
        before=set(self.named())
        M=I.copy()
        if op in ('box','cylinder','cone'):
            M=T@np.array(n['matrix']);d=n['dims']
            if op=='box':
                profile={'kind':'polygon','loops':[[[0,0],[d[0],0],[d[0],d[1]],[0,d[1]]]]}
                self.extrude(profile,0,d[2],label)
            elif op=='cylinder':self.extrude({'kind':'circle','radius':d[0]},0,d[1],label)
            else:
                self.revolve({'kind':'polygon','loops':[[[0,0],[d[0],0],[d[1],d[2]],[0,d[2]]]]},label)
                # revolve axis is sketch Y, primitive cone axis is Z.
                q=I.copy();q[:3,:3]=Rotation.from_euler('x',90,degrees=True).as_matrix();M=M@q
        elif op in ('extrude','revolve'):
            pts=np.array(n['loops'][0]);origin=pts[0]
            if op=='revolve':
                origin=np.array(n['axis_origin']);v=np.array(n['axis_dir'])
                u=next(q-origin-v*np.dot(q-origin,v) for q in pts if np.linalg.norm(q-origin-v*np.dot(q-origin,v))>1e-8)
                u/=np.linalg.norm(u);w=np.cross(u,v)
            else:
                w=np.array(n['dir'],float);w/=np.linalg.norm(w);u=pts[1]-pts[0];u/=np.linalg.norm(u);v=np.cross(w,u)
            Q=I.copy();Q[:3,:3]=np.column_stack([u,v,w]);Q[:3,3]=origin;M=T@Q
            profile={'kind':'polygon','loops':[[[float(np.dot(np.array(p)-origin,u)),float(np.dot(np.array(p)-origin,v))] for p in loop] for loop in n['loops']]}
            if op=='revolve':self.revolve(profile,label)
            else:
                amount=n['amount'];lo=-amount if n['both'] else 0;hi=amount
                if amount<0:lo,hi=amount,0
                self.extrude(profile,lo,hi,label)
        else:raise RuntimeError('Unknown op '+op)
        names=self.tag_new(before,label);self.move(names,M,label)
        if index%10==0:print(self.name,label,flush=True)
        return checked(names)

def tracked(fn):
    def wrapped(self,index,T=I):
        result=fn(self,index,T)
        self.safe={'count':self.count,'sketches':[dict(s) for s in self.sketches],
                   'last_feature':last(self.model).Name,'bodies':list(self.named())}
        return result
    return wrapped

Emitter.emit=tracked(Emitter.emit)

def checkpoint(p,error):
    safe=getattr(p,'safe',None)
    if safe is None:return
    m=p.model;sm=m.SketchManager
    if sm.ActiveSketch is not None:
        sm.AddToDB=False;sm.InsertSketch(True)
    ff=[];f=wrap(m.FirstFeature(),sld.IFeature)
    while f:ff.append(f);f=wrap(f.GetNextFeature(),sld.IFeature)
    names=[f.Name for f in ff]
    if safe['last_feature'] not in names:raise RuntimeError('Cannot find safe feature')
    after=ff[names.index(safe['last_feature'])+1:]
    m.ClearSelection2(True)
    for i,f in enumerate(after):f.Select2(i>0,0)
    if after and not m.Extension.DeleteSelection2(3):raise RuntimeError('Cannot clean partial operation')
    m.EditRebuild3()
    if set(p.named())!=set(safe['bodies']):raise RuntimeError('Checkpoint body mismatch')
    frames=[];tb=error.__traceback__
    while tb:
        loc=tb.tb_frame.f_locals
        if tb.tb_frame.f_code.co_name=='emit' and loc.get('self') is p:
            frames.append({k:loc[k] for k in ('index','a','bb') if k in loc})
        tb=tb.tb_next
    nodes=[dict(n) for n in p.nodes];child=None
    def add(n):
        n=dict(n);n['id']=len(nodes);nodes.append(n);return n['id']
    for frame in reversed(frames):
        n=dict(p.nodes[frame['index']]);op=n['op']
        if op in ('add','sub','common'):
            if 'a' in frame:n['a']=add({'op':'existing','bodies':frame['a']})
            elif child is not None:n['a']=child
            if 'bb' in frame:n['b']=add({'op':'existing','bodies':frame['bb']})
            elif 'a' in frame and child is not None:n['b']=child
        elif op in ('fillet','chamfer'):
            n['a']=add({'op':'existing','bodies':frame['a']}) if 'a' in frame else child
        elif op=='move' and child is not None:n['a']=child
        child=add(n)
    if child is None:child=add({'op':'existing','bodies':getattr(p,'final_result',safe['bodies'])})
    data=dict(p.data);data.update(root=child,nodes=nodes,resume=safe)
    path=ROOT/'validation'/('INCOMPLETE_'+p.name+'.SLDPRT')
    ok,e,w=m.Extension.SaveAs3(str(path),0,1,None,None,0,0)
    if not ok or e:raise RuntimeError('Checkpoint save failed')
    (ROOT/'validation'/('RESUME_'+p.name+'.json')).write_text(json.dumps(data))
    print('RECOVERABLE CHECKPOINT',p.name,'remaining continuation',len(frames),flush=True)

def main():
    sw=connect()
    for name in sys.argv[1:]:
        data=json.loads((ROOT/'reference'/(name+'.json')).read_text())
        resume=ROOT/'validation'/('RESUME_'+name+'.json')
        if resume.exists():
            data=json.loads(resume.read_text());p=object.__new__(Emitter)
            path=ROOT/'validation'/('INCOMPLETE_'+name+'.SLDPRT')
            d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0);p.model=wrap(d,sld.IModelDoc2);sw.ActivateDoc3(p.model.GetTitle(),False,0,0)
            p.sw=sw;p.name=name;p.data=data;p.nodes=data['nodes'];p.count=data['resume']['count'];p.sketches=data['resume']['sketches'];p.safe=data['resume']
            print('RESUMING',name,flush=True)
        else:p=Emitter(sw,data)
        try:
            p.final_result=p.emit(data['root']); result=p.save(expected_volume=data['volume_mm3'])
            error=result['volume_mm3']-data['volume_mm3']; print('VOLUME DIFFERENCE',error,flush=True)
            if abs(error)>result['mass_integration_tolerance_mm3']:raise RuntimeError('Volume mismatch '+str(error))
            if resume.exists():resume.unlink()
            sw.CloseDoc(p.model.GetTitle())
        except Exception as error:
            # Preserve a recoverable checkpoint, explicitly named incomplete.
            path=ROOT/'validation'/('INCOMPLETE_'+name+'.SLDPRT')
            p.model.Extension.SaveAs3(str(path),0,1,None,None,0,0)
            (ROOT/'validation'/('INCOMPLETE_'+name+'.json')).write_text(json.dumps({'sketches':p.sketches,'source':data['name']},indent=2))
            checkpoint(p,error)
            raise

if __name__=='__main__':main()
