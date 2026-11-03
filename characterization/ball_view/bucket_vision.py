"""Small numpy-only measurement helpers shared by live bucket and replay."""
import math
import numpy as np

from flight3d import Observation


def observe(t,c,intr,radius,depth=None,scale=.001,depth_intr=None,rotation=None,translation=None):
    u,v,r=c[:3]
    fx,fy,cx,cy=intr
    zsize=fx*radius/max(r,.5)
    z=zsize
    sigma=max(.15,.20*zsize)
    valid=False
    if depth is not None:
        dfx,dfy,dcx,dcy=depth_intr or intr
        R=np.eye(3) if rotation is None else rotation
        tr=np.zeros(3) if translation is None else translation
        pc=np.array([(u-cx)/fx*zsize,(v-cy)/fy*zsize,zsize])
        pd=R.T@(pc-tr)
        if pd[2]>.1:
            ud,vd=dfx*pd[0]/pd[2]+dcx,dfy*pd[1]/pd[2]+dcy
            rr=max(1.6,.55*r*dfx/fx)
            x0,x1=max(0,int(ud-rr)),min(depth.shape[1],int(ud+rr+1))
            y0,y1=max(0,int(vd-rr)),min(depth.shape[0],int(vd+rr+1))
            if x1>x0 and y1>y0:
                yy,xx=np.mgrid[y0:y1,x0:x1]
                mask=(xx+.5-ud)**2+(yy+.5-vd)**2<=rr*rr
                values=np.asarray(depth[y0:y1,x0:x1],float)[mask]*scale
                good=(values>.3)&(values<10)
                # Strongly nearer foreground is a skin/paint patch, not this sphere.
                if good.sum()>=3 and np.mean(values[good]<.50*zsize)>.65:
                    return None
                good&=(values>.55*zsize)&(values<1.55*zsize)
                if good.sum()>=max(3,.35*mask.sum()):
                    q=float(np.median(values[good]))
                    mad=float(np.median(np.abs(values[good]-q)))
                    if mad<max(.05,.04*q):
                        # Robust inner cap is near the front: approximate centre
                        # correction, with its uncertainty included in sigma_z.
                        pdc=np.array([(ud-dcx)/dfx*q,(vd-dcy)/dfy*q,q])+np.array([0.,0.,.88*radius])
                        z=float((R@pdc+tr)[2])
                        sigma=max(.025,.012*z*z,1.5*mad)
                        valid=True
    if not .25<z<12:
        return None
    return Observation(t,float(u),float(v),float(r),float(z),float(sigma),valid,1.4+.035*r)


def circle_proposals(rgb,max_candidates=32):
    """Colour-independent, half-resolution contour proposals for gesture setup.

    A loose contour cue only: partial hand/ball contours may survive. No Hough
    scan, no hard saturation floor that would make the purple ball invisible.
    OpenCV is optional here, not imported by the numpy-only flight/recorder core.
    """
    import cv2
    small=np.ascontiguousarray(rgb[::2,::2])
    gray=cv2.cvtColor(small,cv2.COLOR_RGB2GRAY)
    edges=cv2.Canny(cv2.GaussianBlur(gray,(5,5),.8),20,55)
    # Equal-luma orange and grey can have almost no grey edge. Preserve chromatic
    # boundaries too, without requiring any particular hue or saturation.
    for channel in range(3):
        edges|=cv2.Canny(cv2.GaussianBlur(small[:,:,channel],(5,5),.8),20,55)
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    proposals=[]
    for contour in contours:
        area=abs(cv2.contourArea(contour))
        if area<9:
            continue
        perimeter=cv2.arcLength(contour,True)
        x,y,w,h=cv2.boundingRect(contour)
        if max(w,h)>125 or min(w,h)<4 or max(w,h)/min(w,h)>1.65:
            continue
        circularity=4*math.pi*area/max(perimeter*perimeter,1)
        if circularity<.40:
            continue
        (u,v),r=cv2.minEnclosingCircle(contour)
        if area/(math.pi*r*r)<.45:
            continue
        if any((2*u-c[0])**2+(2*v-c[1])**2 < max(16,(.7*r)**2) for c in proposals):
            continue
        proposals.append((2*u,2*v,2*r,int(area*4),circularity))
    proposals.sort(key=lambda c:-c[4])
    return [c[:4] for c in proposals[:max_candidates]]


class BlobProposer:
    """blob_proposals with a slow background model (Lab, half resolution, running average): peaks over moving pixels
    are refined first. The gesture needs the ball to move; a small far ball loses on appearance alone to static
    blob-like things (sofa holes, lamps, decorations), which never complete a square anyway."""
    def __init__(self,alpha=.08,threshold=14.):
        self.alpha,self.threshold=alpha,threshold
        self.bg=None

    def __call__(self,rgb,max_candidates=32,keep=None,pad=32,max_rois=6):
        """Proposals inside the regions that moved, plus a window round each `keep` = (u, v, r) or list of them (the
        ball being tuned, the gesture's leading tracks), which may hold still: the Pi 4 took ~350 ms for the whole
        frame, the moving regions are a few percent. (Nov 1: a pause mid-gesture stopped the ball being proposed and
        its track, one square done, was dropped.)"""
        import cv2
        moving=self.motion(rgb)
        h,w=rgb.shape[:2]
        rois=[]
        n,_,stats,_=cv2.connectedComponentsWithStats(cv2.dilate(moving,np.ones((9,9),np.uint8)),connectivity=8)
        for i in range(1,n):
            x,y,bw,bh=(int(s)*2 for s in stats[i,:4])
            rois.append([x-pad,y-pad,x+bw+pad,y+bh+pad])
        if keep is not None and len(keep) and np.isscalar(keep[0]):
            keep=[keep]
        keeps=[]
        for u,v,r,*_ in keep or []:
            half=max(pad,3.5*r)
            keeps.append([u-half,v-half,u+half,v+half])
        clip=lambda boxes:[[max(0,int(a))&~1,max(0,int(b))&~1,min(w,int(c)),min(h,int(d))] for a,b,c,d in boxes]
        # Even corners keep the half-resolution motion mask aligned with each crop. Keep windows stay separate, so
        # a still ball isn't one of a big moving region's few static picks.
        rois=[(b,6,2) for b in _merge_boxes(clip(rois),max_rois)]+[(b,2,4) for b in clip(keeps)]
        out=[]
        for (x0,y0,x1,y1),mb,sb in rois:
            if x1-x0<16 or y1-y0<16:
                continue
            # Moving peaks first inside the region too (static edges share it); a few static ones for a still ball.
            for c in blob_proposals(rgb[y0:y1,x0:x1],max_candidates,moving=moving[y0//2:(y1+1)//2,x0//2:(x1+1)//2],
                                    moving_budget=mb,static_budget=sb):
                if not any((c[0]+x0-o[0])**2+(c[1]+y0-o[1])**2<max(9.,(.7*max(c[2],o[2]))**2) for o in out):
                    out.append((c[0]+x0,c[1]+y0,c[2],c[3]))
        return out[:max_candidates]

    def motion(self,rgb):
        import cv2
        lab=cv2.cvtColor(np.ascontiguousarray(rgb[::2,::2]),cv2.COLOR_RGB2Lab).astype(np.float32)
        lab[...,1:]*=1.5
        lab=cv2.GaussianBlur(lab,(0,0),1.)
        if self.bg is None or self.bg.shape!=lab.shape:
            self.bg=lab.copy()
        d=lab-self.bg
        diff=cv2.sqrt(cv2.transform(d*d,np.ones((1,3),np.float32)))
        # Gain-128 sensor noise put ~5% of raw pixels over a fixed threshold (Nov 1 clip): blur, a per-frame
        # noise-scaled threshold (from every 4th pixel), and speck removal before growing the mask.
        thr=max(self.threshold,4*float(np.median(diff[::4,::4])))
        moving=cv2.morphologyEx((diff>thr).astype(np.uint8),cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        moving=cv2.dilate(moving,np.ones((5,5),np.uint8))
        self.moving_fraction=float(moving.mean())
        cv2.accumulateWeighted(lab,self.bg,self.alpha)
        return moving


def colour_features(rgb,brightness_weight=.5):
    """Log-RGB (40 x natural log, +6 levels against dark-pixel noise), with the overall-brightness direction weighted
    by `brightness_weight`: distances are channel ratios, so a dark but coloured ball stays far from a dark neutral
    shirt (Nov 1, backlit: ball ~(35, 18, 8), shirt ~(8, 8, 10): ~18 apart in Lab, ~45 here), and shading on the
    ball counts half."""
    import cv2
    f=cv2.log(np.ascontiguousarray(rgb).astype(np.float32)+6.)*40.
    mean=f.mean(axis=2,keepdims=True)
    return f-(1-brightness_weight)*mean


def _merge_boxes(boxes,max_boxes):
    """Merge overlapping boxes; past max_boxes, their union."""
    boxes=[list(b) for b in boxes]
    merged=True
    while merged:
        merged=False
        for i in range(len(boxes)):
            for j in range(i+1,len(boxes)):
                a,b=boxes[i],boxes[j]
                if a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]:
                    boxes[i]=[min(a[0],b[0]),min(a[1],b[1]),max(a[2],b[2]),max(a[3],b[3])]
                    del boxes[j]
                    merged=True
                    break
            if merged:
                break
    if len(boxes)>max_boxes:
        boxes=[[min(b[0] for b in boxes),min(b[1] for b in boxes),max(b[2] for b in boxes),max(b[3] for b in boxes)]]
    return boxes


def blob_proposals(rgb,max_candidates=32,r_min=3.5,r_max=70.,min_contrast=10.,moving=None,moving_budget=24,
                   static_budget=8):
    """Colour-independent ball proposals on Lab (lightness and both chroma channels): at each scale of a pyramid,
    a centre box that is uniform and differs from the ring around it scores high (score = centre-ring contrast minus
    the centre's own spread, so edges and texture, whose centre box holds two colours, score low; a Laplacian ranked
    the Nov 1 ball 314th of 7211 peaks behind blinds and frame edges). The best peaks are refined by a local mask
    (ball against its ring, thin attachments such as the holding arm opened away) for an honest centre and radius.

    Replaces whole-image contours for the gesture: on the Nov 1 clip a 9 px-radius ball at 5.3 m, touched by an arm
    and a picture frame's edge, merged into non-round contours (found in 18-23 of 50 frames, radius 4-5 px)."""
    import cv2
    lab=colour_features(rgb)
    peaks=[]
    # The search starts at half resolution (radii from 4 px; the 160 mm bucket ball is ~7 px at 7 m), the refine
    # uses full resolution.
    img,scale=cv2.pyrDown(lab),2
    while min(img.shape[:2])>12 and 2*scale<=r_max:
        sq=img*img
        for rs in (2.0,2.8):                  # ball radius in this level's pixels
            r=rs*scale
            if r<r_min*.8 or r>r_max:
                continue
            a=max(1,int(round(.7*rs)))         # centre box half-width (inside the ball)
            b=max(a+2,int(round(1.9*rs)))      # ring box half-width (outside it)
            m_in=cv2.blur(img,(2*a+1,2*a+1))
            v_in=np.maximum(cv2.blur(sq,(2*a+1,2*a+1))-m_in*m_in,0).sum(axis=2)
            n_in,n_out=(2*a+1)**2,(2*b+1)**2
            m_out=(cv2.blur(img,(2*b+1,2*b+1))*n_out-m_in*n_in)/(n_out-n_in)
            score=np.linalg.norm(m_in-m_out,axis=2)-np.sqrt(v_in)
            local=cv2.dilate(score,np.ones((3,3),np.uint8))
            peak=(score>=local)&(score>min_contrast)
            m=int(math.ceil(1.2*rs))             # box filters reflect at the border: false blobs along the edges
            peak[:m]=peak[-m:]=False
            peak[:,:m]=peak[:,-m:]=False
            ys,xs=np.nonzero(peak)
            us,vs=(xs+.5)*scale-.5,(ys+.5)*scale-.5
            moved=(moving[np.minimum((vs/2).astype(int),moving.shape[0]-1),np.minimum((us/2).astype(int),moving.shape[1]-1)]>0
                   if moving is not None else np.zeros(len(xs),bool))
            peaks.append(np.column_stack((moved,score[ys,xs],us,vs,np.full(len(xs),r))))
        img=cv2.pyrDown(img)
        scale*=2
    peaks=np.concatenate(peaks) if peaks else np.zeros((0,5))
    peaks=peaks[np.lexsort((-peaks[:,1],-peaks[:,0]))]
    # Thin duplicates cheaply first: the best peak per 6 px cell (the same ball peaks at several scales and pixels).
    _,first=np.unique((peaks[:,2]//6).astype(np.int64)*4096+(peaks[:,3]//6).astype(np.int64),return_index=True)
    peaks=peaks[np.sort(first)]
    budget={True:moving_budget if moving is not None else 0,False:static_budget}
    out=[]
    tried=[]
    for moved,score,u,v,r in peaks:
        if len(out)>=max_candidates or not any(budget.values()):
            break
        if not budget[bool(moved)]:
            continue
        # One refine per spot: the same ball peaks at several scales and neighbouring pixels.
        if any((u-t[0])**2+(v-t[1])**2<max(9.,(.8*max(r,t[2]))**2) for t in tried):
            continue
        tried.append((u,v,r))
        budget[bool(moved)]-=1
        ref=_refine(lab,u,v,r,min_contrast)
        if ref is None:
            continue
        u2,v2,r2,area,fill=ref
        if not r_min<=r2<=r_max or any((u2-c[0])**2+(v2-c[1])**2<max(9.,(.7*max(r2,c[2]))**2) for c in out):
            continue
        out.append((u2,v2,r2,area,score*fill))
    out.sort(key=lambda c:-c[4])
    return [c[:4] for c in out]


def _refine(lab,u,v,r,min_contrast):
    """(u, v, r, area, fill) of the compact region at a LoG peak, or None when it isn't one."""
    import cv2
    h,w=lab.shape[:2]
    half=int(math.ceil(2.6*r))+2
    x0,x1,y0,y1=max(0,int(u)-half),min(w,int(u)+half+1),max(0,int(v)-half),min(h,int(v)+half+1)
    win=lab[y0:y1,x0:x1]
    yy,xx=np.ogrid[y0:y1,x0:x1]
    d2=(xx-u)**2+(yy-v)**2
    ring=(d2>(1.7*r)**2)&(d2<(2.5*r)**2)
    core=d2<=max(1.5,.5*r)**2
    if np.count_nonzero(ring)<12 or not core.any():
        return None
    bg=np.median(win[ring],axis=0)
    centre=np.median(win[core],axis=0)
    step=centre-bg
    contrast=float(math.sqrt(step@step))
    if contrast<min_contrast:
        return None
    # Ball pixels are those near the centre's colour, within 0.7 of the distance to the ring's nearest quarter: the ring
    # can be two backgrounds at once (Nov 1: a ball on the edge between a dark shirt and a bright wall; their median
    # was a colour neither had, and a ball-vs-median split failed).
    off=win-centre
    d2c=np.einsum('ijk,ijk->ij',off,off)
    near=float(np.sqrt(np.percentile(d2c[ring],25)))
    if near<min_contrast:
        return None
    mask=(d2c<(.7*near)**2).astype(np.uint8)        # 0.7 of the way: 0.5 trimmed shaded rims (Nov 1 sweep)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))     # specks only
    n,labels,stats,cents=cv2.connectedComponentsWithStats(mask,connectivity=8)
    if n<2:
        return None
    iu,iv=int(round(u))-x0,int(round(v))-y0
    pick=labels[min(max(iv,0),labels.shape[0]-1),min(max(iu,0),labels.shape[1]-1)]
    if pick==0:
        # The peak can sit just off the region (small balls): nearest component to the peak.
        dist=[(cents[i][0]-(u-x0))**2+(cents[i][1]-(v-y0))**2 for i in range(1,n)]
        pick=1+int(np.argmin(dist))
        if dist[pick-1]>(r*.8)**2:
            return None
    region=(labels==pick).astype(np.uint8)
    # The largest circle inside the region is the ball even with an arm or hand attached (narrower than the ball;
    # Nov 1: a backlit ball joined the dark arm into one non-round region and was rejected in 46 of 132 frames).
    # The window's own edge counts as outside only where the image continues past it.
    pad=cv2.copyMakeBorder(region,1,1,1,1,cv2.BORDER_CONSTANT,value=0)
    dt=cv2.distanceTransform(pad,cv2.DIST_L2,5)[1:-1,1:-1]
    R=float(dt.max())
    if R<2.5 or not .5*r<=R<=1.9*r:
        return None
    cy,cx=np.unravel_index(int(np.argmax(dt)),dt.shape)
    R+=.5                                       # the distance runs to the last pixel's centre, not its edge
    # Just outside the circle must be mostly not-region: an attachment may cover part of the rim, not most of it.
    a=np.linspace(0,2*math.pi,48,endpoint=False)
    px=np.round(cx+1.3*R*np.cos(a)).astype(int)
    py=np.round(cy+1.3*R*np.sin(a)).astype(int)
    inside=(px>=0)&(px<region.shape[1])&(py>=0)&(py<region.shape[0])
    if inside.sum()<24:
        return None
    clear=float(1-region[py[inside],px[inside]].mean())
    if clear<.55:
        return None
    return float(cx+x0),float(cy+y0),R,math.pi*R*R,clear


class CameraGuard:
    """Fixed reference image, robust background registration; no pose correction.

    If the mount shifts, the CAD gravity/rim pose is no longer trusted. Sparse or
    moving-foreground-only scenes explicitly report an unverified guard.
    """
    def __init__(self):
        self.reference=None
        self.points=None
        self.count=0
        self.status=dict(verified=False,moved=False,reason='Waiting for a fixed-scene reference')

    def update(self,rgb,target=None):
        import cv2
        self.count+=1
        if self.count%6:
            return self.status
        gray=cv2.cvtColor(np.ascontiguousarray(rgb[::4,::4]),cv2.COLOR_RGB2GRAY)
        if self.reference is None:
            mask=np.full(gray.shape,255,np.uint8)
            if target:
                u,v,r=target[:3]
                cv2.circle(mask,(round(u/4),round(v/4)),round(max(15,r)),0,-1)
            points=cv2.goodFeaturesToTrack(gray,100,.03,7,mask=mask)
            if points is None or len(points)<15:
                self.status=dict(verified=False,moved=False,reason='Too few background features to verify the fixed mount')
                return self.status
            self.reference,self.points=gray,points
            return self.status
        q,valid,_=cv2.calcOpticalFlowPyrLK(self.reference,gray,self.points,None,winSize=(15,15),maxLevel=2)
        good=valid.ravel().astype(bool)
        if good.sum()<12:
            self.status=dict(verified=False,moved=False,reason='Background registration lost: check the mount')
            return self.status
        a,b=self.points[good].reshape(-1,2),q[good].reshape(-1,2)
        matrix,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=.7)
        if matrix is None or inliers.sum()<12:
            self.status=dict(verified=False,moved=False,reason='No stable background consensus')
            return self.status
        keep=inliers.ravel().astype(bool)
        displacement=float(np.median(np.linalg.norm(b[keep]-a[keep],axis=1))*4)
        self.status=dict(verified=True,moved=displacement>3.,displacement_px=displacement,
                         reason='Mount moved: verify bucket pose and identify again' if displacement>3 else 'Background agrees with the fixed reference')
        return self.status
