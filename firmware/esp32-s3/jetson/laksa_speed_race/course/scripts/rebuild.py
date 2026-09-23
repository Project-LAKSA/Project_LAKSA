#!/usr/bin/env python3
"""Deterministic exports from the canonical metric vector geometry.

No ROS, simulator, policy or optimization imports. Source rasters are optional
for regeneration of maps/missions; embedded rectified previews are retained for
overlay regeneration. Local metric GeoJSON uses an explicitly documented local
frame (not WGS84), as appropriate for ROS map coordinates.
"""
from __future__ import annotations
import base64
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
Image.MAX_IMAGE_PIXELS=200_000_000
STATUS='USER_VALIDATION_REQUIRED'

def load(p):return yaml.safe_load(p.read_text())
def native(v):
    if isinstance(v,dict):return {k:native(x) for k,x in v.items()}
    if isinstance(v,(list,tuple,np.ndarray)):return [native(x) for x in v]
    if isinstance(v,np.generic):return v.item()
    return v
def ysave(p,v):p.write_text(yaml.safe_dump(native(v),sort_keys=False,allow_unicode=True))
def jsave(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False,ensure_ascii=False)+'\n')
def feature(coords,typ,**props):return dict(type='Feature',properties=props,geometry=dict(type=typ,coordinates=coords))
def collection(features,course):return dict(type='FeatureCollection',coordinate_frame=f'{course}_map',coordinate_units='meters',geographic_crs=False,features=features)

def primitive_samples(p,step=.025):
    t=np.linspace(0,1,max(2,math.ceil(p['length_m']/step)+1))
    if p['type']=='LINE':
        xy=np.array(p['start_m'])+t[:,None]*(np.array(p['end_m'])-p['start_m']);yaw=np.full(len(t),p['start_yaw_rad']);k=np.zeros(len(t))
    else:
        a=p['start_angle_rad']+t*p['sweep_rad'];sgn=np.sign(p['sweep_rad'])
        xy=np.array(p['center_m'])+p['radius_m']*np.column_stack([np.cos(a),np.sin(a)])
        yaw=a+sgn*math.pi/2;k=np.full(len(t),sgn/p['radius_m'])
    return xy,yaw,k

def speed_vectors(g):
    points=[];yaws=[];curves=[];left=[];right=[];polys=[];gates=[];s=0.
    for p in g['primitives']:
        xy,yaw,k=primitive_samples(p);norm=np.column_stack([-np.sin(yaw),np.cos(yaw)])
        l=xy+norm*g['nominal_path_width_m']/2;r=xy-norm*g['nominal_path_width_m']/2
        polys.append(dict(id=p['id'],vertices_m=np.vstack([l,r[::-1]]).tolist(),layer='ground',provenance=p['provenance']))
        for i in [0]+([len(xy)//2] if p['length_m']>4 else []):
            gates.append(dict(id=f'gate_{len(gates):03}',s_m=s+i/(len(xy)-1)*p['length_m'],center_m=xy[i].tolist(),yaw_rad=float(yaw[i]),width_m=g['nominal_path_width_m'],endpoints_m=[l[i].tolist(),r[i].tolist()],primitive=p['id'],kind='VIRTUAL_TOPOLOGY_GATE',provenance='DERIVED_FROM_LABELS'))
        points.extend(xy[:-1]);yaws.extend(yaw[:-1]);curves.extend(k[:-1]);left.extend(l[:-1]);right.extend(r[:-1]);s+=p['length_m']
    points.append(points[0]);yaws.append(yaws[0]);curves.append(curves[0]);left.append(left[0]);right.append(right[0])
    points=np.array(points);yaws=np.unwrap(yaws);left=np.array(left);right=np.array(right)
    gs=gates[1:]+[dict(gates[0],id='finish',s_m=s)]
    for i,q in enumerate(gs):q['order']=i+1
    return polys,points,yaws,np.array(curves),left,right,gs,s

def densify(points,step=.025):
    out=[]
    for a,b in zip(points,points[1:]):
        a=np.array(a);b=np.array(b);n=max(1,math.ceil(np.linalg.norm(b-a)/step));out.extend(a+np.arange(n)[:,None]/n*(b-a))
    out.append(np.array(points[-1]));return np.array(out)

def obstacle_vectors(g):
    points=[];gates=[];s=0.;sections=[]
    for sec in g['sections']:
        pp=np.array(sec['route_reference_m']);start=len(points)
        if points and np.linalg.norm(np.array(points[-1])-pp[0])>1e-8:
            # Registered connecting reference only. Does not carve free space.
            join=densify([points[-1],pp[0]])
            points.extend(join[1:].tolist())
        ds=densify(pp);points.extend(ds[1:].tolist() if points else ds.tolist())
        sections.append(dict(id=sec['id'],first_sample=start,last_sample=len(points)-1,reference_only=not sec['route_mandatory']))
        # Gates at section entry and exit enforce semantic order. Variable-zone
        # internal reference points never become mandatory waypoints.
        for index,kind in [(0,'entry'),(-1,'exit')]:
            tangent=pp[1]-pp[0] if index==0 else pp[-1]-pp[-2]
            yaw=math.atan2(tangent[1],tangent[0]);pos=pp[index]
            gates.append(dict(id=f'{sec["id"]}_{kind}',order=len(gates)+1,center_m=pos.tolist(),yaw_rad=yaw,section=sec['id'],kind='SECTION_PORTAL',width_m=None,width_provenance='UNRESOLVED_LOCAL_PORTAL_WIDTH',provenance=sec['provenance'],layer='bridge_upper' if sec['id']=='bridge' else 'helical_ramp' if sec['id']=='helical_ramp_down' else 'ground'))
    pts=np.array(points);diff=np.gradient(pts,axis=0);yaw=np.unwrap(np.arctan2(diff[:,1],diff[:,0]));return g['free_regions']+g.get('occupied_regions',[]),pts,yaw,gates,sections

def raster(polys,w,h,res):
    nx=math.ceil(w/res);ny=math.ceil(h/res);out=Image.new('L',(nx,ny),0);d=ImageDraw.Draw(out)
    # Raster coordinates map metric grid-cell centers to pixels. Free polygons
    # are rendered from canonical vectors, never extracted from source pixels.
    for p in polys:d.polygon([(x/res-.5,ny-y/res-.5) for x,y in p['vertices_m']],fill=0 if p.get('occupancy')=='occupied' else 255)
    return out

def occupancy_export(out,polys,w,h,name,status=STATUS):
    info={}
    for suffix,res in [('nav2',.05),('hires',.02)]:
        im=raster(polys,w,h,res);fn=f'{name}_{suffix}.png';im.save(out/fn)
        spec=dict(image=fn,resolution=res,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary',width=im.width,height=im.height,coordinate_frame=f'{out.name}_map',reconstruction_status=STATUS)
        spec['reconstruction_status']=status
        ysave(out/f'{name}_{suffix}.yaml',spec);info[suffix]=spec
    return info

def rectify(out,meta):
    preview=out/'rectified_source.png'
    src=ROOT/'source'/meta['global_geometry_authority']
    if not preview.exists() and src.exists():
        im=Image.open(src);box=meta['registration']['envelope_original_px'];w=1200 if out.name=='speed_course' else 1000
        geom=load(out/'geometry.yaml');h=round(w*geom['overall_bounds']['max_y_m']/geom['overall_bounds']['max_x_m'])
        # Independent axis affine rectification from explicit full-plan envelope.
        im=im.transform((w,h),Image.Transform.EXTENT,box,resample=Image.Resampling.BICUBIC);im.save(preview)
    if not preview.exists():raise RuntimeError('Overlay requires retained rectified_source.png or original source image')
    return Image.open(preview)

def ink_residual(preview,polys,w,h):
    gray=np.asarray(preview.convert('L'),float)
    # Local contrast selects drawn ink under non-uniform scan illumination.
    # Reported as an ink-proximity diagnostic, not a falsely precise boundary
    # annotation. Printed labels/hatched barriers can also enter this mask.
    background=ndimage.maximum_filter(gray,size=15)
    mask=(background-gray)>35
    dt=ndimage.distance_transform_edt(~mask,sampling=[h/gray.shape[0],w/gray.shape[1]])
    coords=np.vstack([densify(p['vertices_m']+[p['vertices_m'][0]],.04) for p in polys])
    xx=np.clip((coords[:,0]/w*gray.shape[1]).astype(int),0,gray.shape[1]-1);yy=np.clip(((h-coords[:,1])/h*gray.shape[0]).astype(int),0,gray.shape[0]-1)
    dist=dt[yy,xx];px=dist/(w/gray.shape[1]);worst=np.argsort(dist)[-30:]
    return dict(method='VECTOR_TO_LOCAL_CONTRAST_INK_DISTANCE',scope='diagnostic nearest visible ink, NOT independently segmented physical boundary truth',includes_internal_region_seams=True,mean_residual_m=float(np.mean(dist)),p95_residual_m=float(np.percentile(dist,95)),max_residual_m=float(max(dist)),mean_residual_px=float(np.mean(px)),p95_residual_px=float(np.percentile(px,95)),max_residual_px=float(max(px)),pixel_frame='rectified_source.png',limitations=['text and barrier hatch may lower residual','blank terrain/variable boundaries may increase residual','hand verification required; not a complete bidirectional boundary residual'],worst_locations_m=coords[worst].tolist())

def landmark_residual(course,meta,polys,w,h):
    evidence=load(ROOT/'validation/boundary_landmarks.yaml');pixels=np.array(evidence[course]['points_px'],float)
    x0,y0,x1,y1=meta['registration']['envelope_reference_px']
    points=np.column_stack([(pixels[:,0]-x0)*w/(x1-x0),(y1-pixels[:,1])*h/(y1-y0)])
    # Sample canonical edges at <= 5mm. Nearest point discretization error <=2.5mm.
    boundary=np.vstack([densify(p['vertices_m']+[p['vertices_m'][0]],.005) for p in polys])
    distances,indices=cKDTree(boundary).query(points);matched=boundary[indices]
    px=np.sqrt(((points[:,0]-matched[:,0])*(x1-x0)/w)**2+((points[:,1]-matched[:,1])*(y1-y0)/h)**2)
    original_scale=meta['registration']['source_pixel_magnification'];opx=np.sqrt(((points[:,0]-matched[:,0])*(x1-x0)/w*original_scale[0])**2+((points[:,1]-matched[:,1])*(y1-y0)/h*original_scale[1])**2)
    return dict(method=evidence['method'],confidence='LOW',sample_count=len(points),independence='Held-out from radius fit' if course=='speed_course' else 'Includes polygon construction corners; NOT independent accuracy validation',reading_uncertainty_reference_px=evidence['reference_pixel_uncertainty'],mean_residual_m=float(distances.mean()),p95_residual_m=float(np.percentile(distances,95)),max_residual_m=float(distances.max()),mean_residual_px=float(opx.mean()),p95_residual_px=float(np.percentile(opx,95)),max_residual_px=float(opx.max()),pixel_frame='original full-authority source image',mean_reference_px=float(px.mean()),p95_reference_px=float(np.percentile(px,95)),limitations=evidence['notes'],samples=[dict(source_reference_px=p.tolist(),source_metric_m=q.tolist(),matched_vector_m=m.tolist(),residual_m=float(d)) for p,q,m,d in zip(pixels,points,matched,distances)])

def route_metrics(im,pts,res):
    a=np.asarray(im)>127;nx,ny=im.width,im.height;x=(pts[:,0]/res).astype(int);y=ny-1-(pts[:,1]/res).astype(int)
    inside=(x>=0)&(x<nx)&(y>=0)&(y<ny);free=np.zeros(len(pts),bool);free[inside]=a[y[inside],x[inside]]
    labels,n=ndimage.label(a);sizes=np.bincount(labels.ravel())[1:]
    return dict(free_component_count=int(n),free_component_sizes_cells=sizes.tolist(),route_sample_count=len(pts),route_samples_occupied=int(np.count_nonzero(~free)),route_occupied_locations_m=pts[~free][::max(1,np.count_nonzero(~free)//20)].tolist(),all_route_samples_free=bool(free.all()))

def dimensions(g,anns):
    report=[];byid={p['id']:p for p in g.get('primitives',[])};regions={p['id']:p for p in g.get('free_regions',[])}
    if g['course']=='obstacle_course':
        terrain={s['id']:s for s in load(ROOT/'canonical/obstacle_course/terrain.yaml')['surfaces']}
        variables=load(ROOT/'canonical/obstacle_course/variable_elements.yaml')['families']
    for a in anns:
        aid=a['id'];target=a['value_si'];value=None;note='Semantic value encoded; independent geometric verification unresolved.'
        if aid=='overall_width':value=g['overall_bounds']['max_x_m'];note='Documented envelope, not drivable-surface extent.'
        elif aid=='overall_height':value=g['overall_bounds']['max_y_m'];note='Documented envelope, not drivable-surface extent.'
        elif g['course']=='speed_course':
            if aid=='path_width':value=g['nominal_path_width_m'];note='Analytic normal offsets.'
            elif aid.startswith('radius_'):value=byid[a['feature_id']]['radius_m'];note='Exact labeled radius; its centerline interpretation requires review.'
            elif aid=='inner_straight_44ft':value=sum(p['length_m'] for p in byid.values() if p['id'].startswith('inner_straight'));note='Sum of two portions split by start/finish.'
            elif aid=='outer_straight_44ft':value=byid[aid]['length_m'];note='Endpoint distance.'
        else:
            region_key={'start_width':('start_lane',1),'car_wash_width':('car_wash',1),'bucket_exit_width':('bucket_exit',1),'bank_width':('bank',0),'bank_length':('bank',1),'gravel_width':('gravel',1),'gravel_length':('gravel',0),'narrow_straight_width':('narrow_straight',0)}
            if aid in region_key:
                key,axis=region_key[aid];v=np.array(regions[key]['vertices_m']);value=float(np.ptp(v[:,axis]));note='Measured from canonical polygon.'
            elif aid=='narrow_curve_radius':value=g['terrain_arc_primitives'][1]['radius_m'];note='Analytic arc; centerline radius reference inferred.'
            elif aid=='narrow_curve_width':value=g['terrain_arc_primitives'][1]['width_m'];note='Analytic normal offset.'
            elif aid=='helix_radius':value=g['terrain_arc_primitives'][0]['radius_m'];note='Approximate label, used as nominal.'
            elif aid=='helix_width':value=g['terrain_arc_primitives'][0]['width_m'];note='Analytic normal offset.'
            elif aid in ('wide_width','wide_length'):
                v=np.array(regions['wide_section']['vertices_m']);value=float(np.ptp(v[:,0 if aid=='wide_width' else 1]));note='Registered illustrated envelope differs from approximate variable-rule envelope; LOW_CONFIDENCE.'
            elif aid=='bank_divider_radius':value=g['occupied_regions'][0]['radius_m'];note='Approximate printed radius; registered divider end.'
            elif aid=='bank_approach_length':
                v=np.array(regions['bank_departure']['vertices_m']);value=float(np.ptp(v[:,0]));note='Registered horizontal interval; scan-vs-label disagreement retained.'
            elif aid.startswith('hoop_interval'):
                seg=variables['hoops']['placement_segments_m'];value=abs(seg[1][0][0]-seg[0][0][0]);note='Registered separation of dashed placement lines; both printed intervals nominal 96in.'
            elif aid=='hoop_transverse':
                seg=variables['hoops']['placement_segments_m'][0];value=float(np.linalg.norm(np.array(seg[0])-seg[1]));note='Registered dashed-line span.'
            else:
                keys={'bank_angle':('bank','angle_rad'),'gravel_depth':('gravel','depth_m'),'pothole_depth':('pothole','holes_depth_m'),'bump_height':('pothole','bumps_height_m'),'pothole_diameter':('pothole','diameter_m'),'tunnel_width':('tunnel','structure_width_m'),'tunnel_length':('tunnel','structure_length_m'),'tunnel_path_width':('tunnel','path_width_m'),'tunnel_path_length':('tunnel','path_length_m'),'bridge_height':('bridge','z_m'),'ramp_up_grade':('ramp_up','grade'),'ramp_up_width':('ramp_up','width_m'),'helix_grade':('helix','grade'),'helix_rails':('helix','side_rail_height_m')}
                if aid in keys:
                    key,field=keys[aid];value=abs(terrain[key][field]);note='Compared with canonical analytic terrain/structure parameter; no physical measurement claimed.'
                elif aid in ['bucket_min','bucket_max']:value=variables['buckets']['min_count' if aid=='bucket_min' else 'max_count'];note='Compared with legal count schema; no buckets generated.'
                elif aid.rsplit('_',1)[-1] in ['height','width','length']:
                    key,field=aid.rsplit('_',1)
                    if key in terrain:
                        value=terrain[key][{'height':'rise_m','width':'width_m','length':'run_m'}[field]];note='Compared with analytic ramp parameter.'
        error=None if value is None else abs(value-target)
        tolerance=max(.03,abs(target)*.01) if aid.startswith('radius_') else .02 if 'width' in aid else max(.0001,abs(target)*.005)
        status='UNRESOLVED_COMPARISON' if error is None else 'PASS' if error<=tolerance else 'LOW_CONFIDENCE'
        entry=dict(**a,reconstructed_value_si=value,absolute_error_si=error,tolerance_si=tolerance,validation_status=status,validation_note=note)
        if status=='LOW_CONFIDENCE':entry['confidence']='LOW'
        report.append(entry)
    return report

def analytic_boundaries(g):
    result=[]
    for p in g['primitives']:
        for side,offset in [('left',g['nominal_path_width_m']/2),('right',-g['nominal_path_width_m']/2)]:
            q=dict(p);q['id']=p['id']+'_'+side;q['source_centerline_primitive']=p['id'];q['provenance']='DERIVED_FROM_LABELS'
            if p['type']=='ARC':
                q['radius_m']=p['radius_m']-np.sign(p['sweep_rad'])*offset
                q['length_m']=abs(q['sweep_rad'])*q['radius_m']
                for key,a in [('start_m',q['start_angle_rad']),('end_m',q['start_angle_rad']+q['sweep_rad'])]:q[key]=(np.array(q['center_m'])+q['radius_m']*np.array([math.cos(a),math.sin(a)])).tolist()
            else:
                n=np.array([-math.sin(p['start_yaw_rad']),math.cos(p['start_yaw_rad'])])
                for key in ['start_m','end_m']:q[key]=(np.array(p[key])+offset*n).tolist()
            result.append(q)
    return result

def export_terrain(out,g,w,h):
    spec=load(out/'terrain.yaml');res=.02;ny=math.ceil(h/res);nx=math.ceil(w/res);height=np.zeros((ny,nx),np.float32);valid=np.zeros((ny,nx),np.uint8);sem=np.zeros((ny,nx),np.uint8)
    yy=(ny-np.arange(ny)-.5)*res;xx=(np.arange(nx)+.5)*res;X,Y=np.meshgrid(xx,yy);regions={p['id']:p for p in g['free_regions']}
    for idx,p in enumerate(g['free_regions'],1):
        mask=np.asarray(raster([p],w,h,res))>127;valid[mask]=1;sem[mask]=idx
    # Separate ground and upper-layer arrays preserve bridge topology. The
    # bank's datum is unknown: relative rise only; do not assert absolute z.
    upper=np.zeros_like(height);upper_valid=np.zeros_like(valid);bank_relative=np.zeros_like(height)
    for surf in spec['surfaces']:
        region=regions.get(surf['region'])
        if region is None:continue
        mask=np.asarray(raster([region],w,h,res))>127
        v=np.array(region['vertices_m']);model=surf['model']
        if model=='ELEVATED_PLANE':upper[mask]=surf['z_m'];upper_valid[mask]=1
        elif model=='HELICAL_RAMP':
            c=np.array(surf['center_m']);ang=(np.arctan2(Y-c[1],X-c[0])-surf['start_angle_rad'])%(2*math.pi)
            z=surf['z_start_m']-np.clip(ang/surf['sweep_rad'],0,1)*surf['z_start_m'];upper[mask]=z[mask];upper_valid[mask]=1
        elif model=='BANKED_PLANE':bank_relative[mask]=((X-v[:,0].min())*math.tan(surf['angle_rad']))[mask]
        elif surf['id']=='ramp_up':
            z=np.clip((v[:,0].max()-X)*surf['grade'],0,surf['z_end_m']);upper[mask]=z[mask];upper_valid[mask]=1
        elif model=='MATERIAL_BOX':height[mask]=surf['surface_elevation_m']
        elif model=='VARIABLE_RELIEF':height[mask]=surf['nominal_platform_z_m']
        elif model=='LINEAR_RAMP':
            direction=surf['travel_direction_xy'][0];t=(X-v[:,0].min())/surf['run_m'];t=t if direction>0 else 1-t
            z=np.clip(t if surf['grade']>0 else 1-t,0,1)*surf['rise_m'];height[mask]=z[mask]
    for name,array in [('heightfield',height),('upper_layer_heightfield',upper),('bank_relative_heightfield',bank_relative)]:np.save(out/f'{name}.npy',array,allow_pickle=False)
    for name,array in [('terrain_semantics',sem),('heightfield_valid',valid*255),('upper_layer_valid',upper_valid*255)]:Image.fromarray(array).save(out/f'{name}.png')
    jsave(out/'terrain_raster_metadata.json',dict(resolution=res,origin=[0,0,0],shape=[ny,nx],row_zero='highest Y',unknown_absolute_bank_datum=True,semantic_ids={str(i):p['id'] for i,p in enumerate(g['free_regions'],1)},upper_layer='Separate surface; never merge bridge into ground elevation'))

def overlay(out,meta,g,polys,pts,gates,anns,report):
    preview=out/'rectified_source.png';src='data:image/png;base64,'+base64.b64encode(preview.read_bytes()).decode()
    w=g['overall_bounds']['max_x_m'];h=g['overall_bounds']['max_y_m'];labels=[]
    rx0,ry0,rx1,ry1=meta['registration']['envelope_reference_px']
    for a in anns:
        if a.get('reference_position_px'):
            px,py=a['reference_position_px'];labels.append(dict(text=a['original_text'],x=(px-rx0)/(rx1-rx0)*w,y=(ry1-py)/(ry1-ry0)*h,status=a['validation_status'],note=a['validation_note']))
    payload=dict(course=out.name,w=w,h=h,source=src,polys=polys,route=pts[::2].tolist(),gates=gates,labels=labels,start=g['start_finish'],report=report)
    template=(ROOT/'scripts/overlay_template.html').read_text();(out/'validation_overlay.html').write_text(template.replace('__PAYLOAD__',json.dumps(payload,ensure_ascii=False,allow_nan=False).replace('</','<\\/')))

def build(course):
    out=ROOT/'canonical'/course;g=load(out/'geometry.yaml');meta=load(out/'source_metadata.yaml');anns=load(out/'geometry_annotations.yaml')['dimensions'];w=g['overall_bounds']['max_x_m'];h=g['overall_bounds']['max_y_m']
    status=meta['reconstruction_status'];approval={}
    if status=='CANONICAL_APPROVED':
        record=load(out/'approval.yaml')
        if hashlib.sha256((out/'geometry.yaml').read_bytes()).hexdigest()!=record['approved_geometry_sha256']:
            raise ValueError('Approved geometry changed: quantitative inconsistency review required before rebuilding.')
        approval={key:record[key] for key in ['approved_by_user','approval_status','approval_note']}
    if course=='speed_course':
        polys,pts,yaw,k,left,right,gates,length=speed_vectors(g)
        features=[feature(left.tolist(),'LineString',id='left_physical_boundary'),feature(right.tolist(),'LineString',id='right_physical_boundary')]
        ysave(out/'boundary_primitives.yaml',dict(primitives=analytic_boundaries(g),authority='derived from geometry.yaml exact normal offsets'))
        ss=np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]
        with (out/'centerline.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['s_m','x_m','y_m','yaw_rad','curvature_1pm','left_clearance_m','right_clearance_m'])
            for s,p,a,c in zip(ss,pts,yaw,k):writer.writerow([f'{x:.9f}' for x in [s,p[0],p[1],a,c,g['nominal_path_width_m']/2,g['nominal_path_width_m']/2]])
        jsave(out/'centerline.geojson',collection([feature(pts.tolist(),'LineString',id='closed_circuit_centerline',length_m=length)],course))
        name='speed_course';sections=None
    else:
        polys,pts,yaw,gates,sections=obstacle_vectors(g);length=float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum());name='static_occupancy'
        features=[feature([p['vertices_m']+[p['vertices_m'][0]]],'Polygon',id=p['id'],layer=p['layer'],provenance=p['provenance'],semantics='occupied barrier' if p.get('occupancy')=='occupied' else 'free region; overlapping polygon interiors are unioned; shared edges are NOT physical walls') for p in polys]
        jsave(out/'route_reference.geojson',collection([feature(pts.tolist(),'LineString',id='ordered_section_reference',mandatory_internal_trajectory=False),* [feature(s['route_reference_m'],'LineString',section=s['id'],order=s['order']) for s in g['sections']]],course))
        export_terrain(out,g,w,h)
    jsave(out/'boundaries.geojson',collection(features,course));maps=occupancy_export(out,polys,w,h,name,status)
    ysave(out/'checkpoints.yaml',dict(reconstruction_status=status,ordered=True,gates=gates))
    start=g['start_finish'];mission=dict(schema_version='laksa-canonical-mission-v1',reconstruction_status=STATUS,training_allowed=False,mission_type='CLOSED_CIRCUIT' if course=='speed_course' else 'START_TO_FINISH',direction='FROM_DRAWING_ARROWS',competition_window_s=1200,objective='MINIMIZE_VALID_COMPLETION_TIME',coordinate_frame=meta['coordinate_frame']['name'],nav2_concept='NavigateThroughPoses / Waypoint Follower',start_pose=dict(x=start['position_m'][0],y=start['position_m'][1],yaw=start['yaw_rad']),finish=dict(line_center_m=start['position_m'],crossing_yaw_rad=start['yaw_rad'],requires_all_prior_gates=True),closed_lap=course=='speed_course',gates=[dict(order=q['order'],gate_id=q['id'],pose=dict(x=q['center_m'][0],y=q['center_m'][1],yaw=q['yaw_rad']),**({'section':q['section'],'layer':q['layer']} if 'section' in q else {})) for q in gates],validity_rules=['gate crossings must occur in listed order and forward direction','leaving the legal corridor invalidates that attempt; reentry does not advance progress','finish counts only after every required gate','a gate crossing cannot be substituted by a nearby parallel lane'],unknown_rules=meta['unknown_competition_rules'])
    if sections:mission.update(sections=sections,variable_zones='Checkpoints are nominal portals; regenerate variable entrance portal from approved instance before use.',layer_rule='Bridge upper and tunnel ground crossings are separate states; no transition at their planar intersection.',planar_navigation_limit='A single static 2D occupancy cannot enforce grade-separated topology. This projection requires per-section/layer planning restrictions before simulation.')
    mission.update(reconstruction_status=status,**approval)
    ysave(out/'mission.yaml',mission)
    preview=rectify(out,meta);residual=ink_residual(preview,polys,w,h);dim=dimensions(g,anns);metrics=route_metrics(Image.open(out/f'{name}_hires.png'),pts,.02)
    points_bounds=np.vstack([p['vertices_m'] for p in polys]);excess=np.maximum(0,np.maximum(-points_bounds,points_bounds-[w,h])).max()
    report=dict(schema_version='laksa-reconstruction-validation-v1',reconstruction_status=STATUS,course=course,training_allowed=False,geometry_authority='geometry.yaml',documented_bounds_m=[w,h],drivable_bounds_m=[points_bounds.min(axis=0).tolist(),points_bounds.max(axis=0).tolist()],maximum_drivable_envelope_excess_m=float(excess),dimension_comparisons=dim,dimensions_pass=sum(d['validation_status']=='PASS' for d in dim),dimensions_low_confidence=[d['id'] for d in dim if d['validation_status']=='LOW_CONFIDENCE'],dimensions_not_independently_verified=[d['id'] for d in dim if d['validation_status']=='UNRESOLVED_COMPARISON'],radius_reference_uncertainty='Speed and narrow-turn labels read clearly; centerline vs boundary radius reference is not explicitly led.',route_reference_length_m=length,geometry_primitive_count=len(g.get('primitives',polys)),checkpoint_count=len(gates),maximum_sample_spacing_m=float(np.linalg.norm(np.diff(pts,axis=0),axis=1).max()),raster_to_vector_residual=residual,topology_checks=metrics,map_exports=maps,limitations=g['reconstruction_uncertainty'])
    if course=='speed_course':
        report.update(nominal_path_width_m=g['nominal_path_width_m'],measured_vector_width_min_m=float(np.linalg.norm(left-right,axis=1).min()),measured_vector_width_max_m=float(np.linalg.norm(left-right,axis=1).max()),closed_route_error_m=float(np.linalg.norm(pts[0]-pts[-1])),max_primitive_connection_gap_m=max(float(np.linalg.norm(np.array(a['end_m'])-b['start_m'])) for a,b in zip(g['primitives'],g['primitives'][1:])))
        free=np.array(Image.open(out/f'{name}_hires.png'))>127;holes=ndimage.binary_fill_holes(free)&~free;_,nholes=ndimage.label(holes);report['topology_checks']['enclosed_non_drivable_components']=int(nholes)
    else:report['grade_separated_crossing']=g['grade_separated_crossing']
    report['boundary_landmark_residual']=landmark_residual(course,meta,polys,w,h)
    report['validation_result']='USER_REVIEW_REQUIRED' if metrics['all_route_samples_free'] else 'GEOMETRY_CORRECTION_REQUIRED'
    report.update(reconstruction_status=status,**approval)
    if approval and metrics['all_route_samples_free']:report['validation_result']='CANONICAL_APPROVED'
    jsave(out/'validation_report.json',report);overlay(out,meta,g,polys,pts,gates,dim,report)
    print(course,json.dumps(dict(length_m=round(length,3),components=metrics['free_component_count'],occupied_samples=metrics['route_samples_occupied'],envelope_excess_m=round(float(excess),3),residual_p95_m=round(residual['p95_residual_m'],3))))

if __name__=='__main__':
    for course in ['speed_course','obstacle_course']:build(course)
