"""Read-only audit of recorded D435 clips; keeps the frozen detector unchanged.

Run with the sim venv. Output is incremental JSON plus contact sheets. Photometry
is conditional on accepted offline colour picks, not independent detection truth.
Blur travel is centroid speed times exposure, NOT a measured optical streak.
loc3 is reserved for validation; this script does not tune any settings.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import rs_analyze as A
import rs_detect as D
import rs_predict as P
import session as S


def quant(x):
    a = np.asarray(x, float)
    a = a[np.isfinite(a)]
    return np.percentile(a, [10, 50, 90]).tolist() if len(a) else None


def finite_json(x):
    if isinstance(x, dict):
        return {k: finite_json(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite_json(v) for v in x]
    if isinstance(x, (float, np.floating)) and not np.isfinite(x):
        return None
    return x


def photo(img, u, v, r, gain):
    # Inner half-radius reduces threshold-edge/background contamination.
    rr = max(2., r / gain)
    x0, x1 = max(0, int(u-2*rr)), min(img.shape[1], int(u+2*rr+1))
    y0, y1 = max(0, int(v-2*rr)), min(img.shape[0], int(v+2*rr+1))
    a = img[y0:y1, x0:x1].astype(float) / 255
    yy, xx = np.mgrid[y0:y1, x0:x1]
    rad = np.hypot(xx-u, yy-v)
    core, ring = rad < .5*rr, (rad > 1.4*rr) & (rad < 2*rr)
    val = a.max(axis=-1)
    sat = (val-a.min(axis=-1)) / np.maximum(val, 1e-8)
    lum = a @ np.array([.2126, .7152, .0722])  # encoded RGB luma, not physical luminance
    med = lambda b, m: float(np.median(b[m])) if m.any() else float('nan')
    return dict(value=med(val, core), saturation=med(sat, core),
                luma=med(lum, core), local_luma_difference=med(lum, core)-med(lum, ring))


def sheets(th, picks, out):
    valid = [i for i, p in enumerate(picks) if p is not None]
    if valid:
        ids = sorted(set(np.linspace(valid[0], valid[-1], 8).round().astype(int).tolist()
                         + [min(len(picks)-1, valid[-1]+3), min(len(picks)-1, valid[-1]+8)]))
    else:
        ids = np.linspace(0, len(th.frames)-1, 10).round().astype(int).tolist()
    sheet = Image.new('RGB', (212*len(ids), 272), '#181818')
    dr = ImageDraw.Draw(sheet)
    for n, i in enumerate(ids):
        im = Image.fromarray(np.asarray(th.frames[i])).copy()
        di = ImageDraw.Draw(im)
        p = picks[i]
        if p is not None:
            c = p[1]
            di.ellipse((c.u-c.r-3, c.v-c.r-3, c.u+c.r+3, c.v+c.r+3), outline='white', width=2)
            x, y = round(c.u), round(c.v)
        else:
            x, y = 424, 240
        sheet.paste(im.resize((212, 120)), (212*n, 24))
        crop = Image.fromarray(np.asarray(th.frames[i])).crop((x-48,y-48,x+48,y+48))
        sheet.paste(crop, (212*n+58, 164))
        dr.text((212*n+3, 4), f'f{i} {th.t[i]:.3f}s '+('pick' if p else 'NO PICK'), fill='white')
    sheet.save(out)
    # Full clip too: a fitted track may only cover ball handling or a decoy.
    sheet = Image.new('RGB', (1272, 576), '#181818')
    dr = ImageDraw.Draw(sheet)
    for k, i in enumerate(np.linspace(0, len(th.frames)-1, 24).round().astype(int)):
        x, y = k % 6 * 212, k // 6 * 144
        sheet.paste(Image.fromarray(np.asarray(th.frames[i])).resize((212,120)), (x,y+24))
        dr.text((x+2,y+2), f'f{i} {th.t[i]:.3f}s', fill='white')
    sheet.save(out.with_name(out.stem+'_full.png'))


def scene_stability(root, out):
    """Optional OpenCV pass: robust setup/end background image displacement.

    Run with the realsense venv. This is image registration, not a calibrated
    camera translation, pitch measurement, or proof the rig was stable in between.
    """
    import cv2
    rows=[]
    for sd in sorted(root.iterdir()):
        p,q=sd/'setup_color.npy',sd/'end_color.npy'
        if not p.exists() or not q.exists():
            rows.append(dict(session=sd.name,available=False))
            continue
        a,b=np.load(p),np.load(q)
        orb=cv2.ORB_create(nfeatures=4000)
        ka,da=orb.detectAndCompute(cv2.cvtColor(a,cv2.COLOR_RGB2GRAY),None)
        kb,db=orb.detectAndCompute(cv2.cvtColor(b,cv2.COLOR_RGB2GRAY),None)
        if da is None or db is None:
            rows.append(dict(session=sd.name,available=True,error='no image descriptors'))
            continue
        pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(da,db,k=2)
        good=[pair[0] for pair in pairs if len(pair)==2 and pair[0].distance < .75*pair[1].distance]
        if len(good)<4:
            rows.append(dict(session=sd.name,available=True,error='too few matches'))
            continue
        x=np.float32([ka[m.queryIdx].pt for m in good])
        y=np.float32([kb[m.trainIdx].pt for m in good])
        h,mask=cv2.findHomography(x,y,cv2.RANSAC,3.)
        if h is None:
            rows.append(dict(session=sd.name,available=True,error='registration failed'))
            continue
        keep=mask.ravel().astype(bool)
        delta=y[keep]-x[keep]
        rows.append(dict(session=sd.name,available=True,opencv=cv2.__version__,
                         ratio_matches=len(good),inliers=int(keep.sum()),
                         median_displacement_px=float(np.median(np.linalg.norm(delta,axis=1))),
                         median_dxdy=np.median(delta,axis=0).tolist(),
                         inlier_span_xy=np.ptp(x[keep],axis=0).tolist(),homography=h.tolist()))
    (out/'rig_stability.json').write_text(json.dumps(rows,indent=2))


def ir_probes(root, out):
    """Conditional IR1 photometry at colour-picked rays, NOT independent IR truth.

    Map factory-calibrated rays using the colour source's range. Core radius is
    half the threshold-mask radius scaled by focal length. The background is the
    median of the first 0.25s of IR1. No exposure/gain enhancement is applied.
    """
    rows=[]
    for f in sorted(out.glob('*_t*.json')):
        row=json.loads(f.read_text())
        sd=root/row['session']
        sess=S.load_session(sd)
        th=S.load_throw(sd/row['clip'],sess)
        if 'ir1' not in th.names or not row['photo']:
            continue
        cal=A.calibration(sess)
        ic,idp=cal['intr']['color'],cal['intr']['ir1']
        R,tr=cal['ext'].get('depth_to_color',(np.eye(3),np.zeros(3)))
        st=th.stream('ir1')
        ts=th.t_of('ir1')
        bg=np.median(st.frames[:int(max(3,np.sum(ts<ts[0]+.25)))],axis=0)
        probes=[]
        for v in row['photo']:
            k=int(A.nearest(ts,np.array([v['t']]))[0])
            if abs(ts[k]-v['t']) > .008:
                continue
            pd=R.T@(ic.deproject(v['u'],v['v'],v['depth_or_size_z_m'])-tr)
            u,y=idp.project(pd)
            rad=max(2.,v['threshold_radius_px']*idp.fx/ic.fx)
            x0,x1=max(0,int(u-rad)),min(848,int(u+rad+1))
            y0,y1=max(0,int(y-rad)),min(480,int(y+rad+1))
            if x1<=x0 or y1<=y0:
                continue
            yy,xx=np.mgrid[y0:y1,x0:x1]
            core=np.hypot(xx-u,yy-y)<.5*rad
            if not core.any():
                continue
            values=np.asarray(st.frames[k,y0:y1,x0:x1],float)[core]
            delta=values-bg[y0:y1,x0:x1][core]
            probes.append(dict(color_frame=v['frame'],ir1_frame=k,
                               median_raw_0_255=float(np.median(values)),
                               median_difference_to_bg=float(np.median(delta)),
                               fraction_above_bg_plus35=float(np.mean(delta>35))))
        rows.append(dict(session=row['session'],clip=row['clip'],probes=probes))
    (out/'ir_probes.json').write_text(json.dumps(rows,indent=2))


def audit_clip(td, sess, out):
    th = S.load_throw(td, sess)
    cfg = A.with_session_ball(D.DetectCfg(), sess)
    bucket = P.Bucket.load(A.HERE / 'bucket.json')
    bucket.ball_radius = cfg.ball_radius
    start = time.perf_counter()
    res = A.analyze_throw(th, sess, cfg, session_dir=td.parent, up=bucket.up)
    A.add_predictions(res, bucket, cfg, up=bucket.up)
    row = dict(session=td.parent.name, clip=td.name, review=th.meta.get('review'),
               detector=sess.get('detector'), ball_metadata=sess.get('ball'),
               held_out=td.parent.name.endswith('_loc3'),
               sources=res['sources'], prediction=res.get('prediction'),
               analysis_seconds=time.perf_counter()-start,
               trigger_s=th.meta.get('t_trigger_s'), health={}, photo=[], picks=[])
    for name in th.names:
        st = th.stream(name)
        row['health'][name] = S.stream_stats(st.stamps, th.t0_ns)
        row['health'][name]['stamp_to_host_ms_p10_50_90'] = quant((st.stamps[:,1]-st.stamps[:,0])*1e-6)
        if st.info is not None:
            # Real colour recordings contain 20/80 here at configured 2000/8000 us.
            # Preserve raw values; do not silently treat this field as microseconds.
            row['health'][name]['stored_exposure_field_raw_p10_50_90'] = quant(st.info[:,0])
            row['health'][name]['gain_p10_50_90'] = quant(st.info[:,1])
    picks = res['_keep'].get('colour', (None, [None]*len(th.frames), None))[1]
    gain = float(sess.get('detector', {}).get('size_gain', 1))
    exposure_us = sess.get('camera', {}).get('exposure_us')
    row['configured_color_exposure_us'] = exposure_us
    ids = [i for i,p in enumerate(picks) if p is not None]
    for k, i in enumerate(ids):
        c = picks[i][1]
        q = photo(th.frames[i], c.u, c.v, c.r, gain)
        q.update(frame=i, t=float(th.t[i]), u=c.u, v=c.v, corrected_radius_px=c.r,
                 threshold_radius_px=c.r/gain, depth_or_size_z_m=c.z)
        if len(ids) > 1:
            ia, ib = ids[max(0,k-1)], ids[min(len(ids)-1,k+1)]
            ca, cb = picks[ia][1], picks[ib][1]
            speed = np.hypot(cb.u-ca.u, cb.v-ca.v)/(th.t[ib]-th.t[ia])
            q['image_speed_px_s'] = float(speed)
            q['exposure_travel_px'] = float(speed * exposure_us * 1e-6) if exposure_us else None
        row['photo'].append(q)
        row['picks'].append([i, float(th.t[i]), c.u,c.v,c.r,c.z])
    if ids:
        last = picks[ids[-1]][1]
        row['last_pick_edge_distance_px'] = dict(left=last.u-last.r, right=848-last.u-last.r,
                                                top=last.v-last.r, bottom=480-last.v-last.r)
        row['accepted_span_s'] = float(th.t[ids[-1]]-th.t[ids[0]])
        row['max_internal_missing_frames'] = int(max(np.diff(ids)-1, default=0))
    sheets(th, picks, out / f'{td.parent.name}_{td.name}.png')
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root', type=Path)
    ap.add_argument('--out', type=Path, default=A.RESULTS/'audit_20261031')
    ap.add_argument('--session', default='')
    ap.add_argument('--scene-only', action='store_true', help='OpenCV setup/end registration; realsense venv')
    ap.add_argument('--ir-probes-only', action='store_true', help='IR1 photometry at cached colour picks')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    if a.scene_only:
        scene_stability(a.root,a.out)
        return
    if a.ir_probes_only:
        ir_probes(a.root,a.out)
        return
    hashes = {n: hashlib.sha256((A.HERE/n).read_bytes()).hexdigest() for n in
              ('rs_detect.py','rs_analyze.py','rs_predict.py','arc2d.py','record_rs.py')}
    provenance = dict(detector_hashes=hashes,
        root=str(a.root.resolve()), held_out='loc3: no tuning or detector edits',
        limits=['conditional offline metrics; not frame-labelled recall',
                'exposure travel uses configured exposure; raw colour metadata field has different units',
                'clips selected by colour trigger; missed unsaved throws absent',
                'yellow ball size/mass inherited incorrectly; metric range/drag provisional',
                'camera stationary; no robot motion or optical streak measurement'])
    pf = a.out/'provenance.json'
    if pf.exists():
        previous = json.loads(pf.read_text())
        if previous['detector_hashes'] != hashes or previous['root'] != provenance['root']:
            raise RuntimeError('Inputs/code differ from this cached baseline; use a fresh --out directory for the comparison')
    pf.write_text(json.dumps(provenance, indent=2))
    for sd in sorted(a.root.iterdir()):
        if not (sd/'session.json').exists() or a.session not in sd.name:
            continue
        sess = S.load_session(sd)
        for td in S.throw_dirs(sd):
            f = a.out/f'{sd.name}_{td.name}.json'
            if f.exists():
                continue
            try:
                row = audit_clip(td, sess, a.out)
                f.write_text(json.dumps(finite_json(row), indent=2, default=float, allow_nan=False))
                c = row['sources'].get('colour',{})
                print(sd.name,td.name,'OK' if c.get('ok') else 'FAIL',
                      c.get('flight_frames'),c.get('rms_px'),c.get('reason',''),flush=True)
            except Exception as e:
                import traceback
                traceback.print_exc()
                print(sd.name,td.name,'ERROR',repr(e),flush=True)


if __name__ == '__main__':
    main()
