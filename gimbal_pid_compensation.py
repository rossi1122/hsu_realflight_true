"""Camera ray -> NED -> SAME reference camera used by virtual projection.
Default model follows this project's earth pitch/roll + body-relative yaw
assumption. This convention must be checked against the installed firmware.
"""
import copy
import math
from types import SimpleNamespace
import numpy as np
from dive_guidance import R_G2C, rot_y_np, rot_z_np
from realvision_pid_logger import RealVisionPIDLogger, REAL_PID_FIELDS

GEOMETRY_FIELDS = ['gcomp_valid', 'gcomp_reason', 'gcomp_pose_method',
    'gcomp_gimbal_method', 'gcomp_pose_gap_ms', 'gcomp_gimbal_gap_ms',
    'gcomp_capture_mono', 'gcomp_gimbal_yaw', 'gcomp_gimbal_pitch',
    'gcomp_gimbal_roll', 'gcomp_ac_pitch', 'gcomp_ac_roll', 'gcomp_ac_yaw',
    'gcomp_ac_course', 'gcomp_u', 'gcomp_v', 'gcomp_u_err_deg', 'gcomp_v_err_deg']
COMP_PID_FIELDS = [f.replace('real_', 'real_comp_', 1) for f in REAL_PID_FIELDS]

def rx(deg):
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
    return np.array([[1.,0.,0.],[0.,c,s],[0.,-s,c]])

def reference_matrix(pitch,course,tilt=0.):
    # Deliberately matches project_body_camera: roll=0 and course, not nose yaw.
    return R_G2C @ rot_y_np(math.radians(tilt)) @ rot_y_np(math.radians(pitch)) @ rot_z_np(math.radians(course))

def camera_matrix(cfg, gy,gp,gr, aircraft):
    roll,pitch,yaw,course=aircraft
    y=cfg.SIYI_YAW_SIGN*gy+cfg.GCOMP_YAW_OFFSET_DEG
    p=cfg.SIYI_PITCH_SIGN*gp+cfg.GIMBAL_PITCH_OFFSET_DEG
    r=cfg.GCOMP_ROLL_SIGN*gr+cfg.GCOMP_ROLL_OFFSET_DEG
    mode=cfg.GCOMP_ATTITUDE_FRAME
    if mode == 'earth_pitch_roll_body_yaw':
        # Project's existing convention: yaw encoder offset from aircraft yaw.
        yaw_world=yaw+y
        return R_G2C @ rot_y_np(math.radians(p)) @ rx(r) @ rot_z_np(math.radians(yaw_world))
    if mode == 'body_relative':
        return (R_G2C @ rot_y_np(math.radians(p)) @ rx(r) @ rot_z_np(math.radians(y))
                @ rx(roll) @ rot_y_np(math.radians(pitch)) @ rot_z_np(math.radians(yaw)))
    raise ValueError('Unknown GCOMP_ATTITUDE_FRAME: '+mode)

def transform_pixel(cfg, virtual_cfg, u,v, aircraft,gimbal):
    fx,fy,cx,cy=cfg.effective_intrinsics()
    ray=np.array([(u-cx)/fx,(v-cy)/fy,1.])
    actual=camera_matrix(cfg,*gimbal,aircraft)
    reference=reference_matrix(aircraft[1],aircraft[3],virtual_cfg.GIMBAL_DIVE_TILT_BODY_DEG)
    converted=reference @ actual.T @ ray
    if not np.isfinite(converted).all() or converted[2] <= 1e-6:
        return None
    return (virtual_cfg.CX+virtual_cfg.FX*converted[0]/converted[2],
            virtual_cfg.CY+virtual_cfg.FY*converted[1]/converted[2])

def capture_geometry(cfg, virtual_cfg, st, cap_mono, pose_history, gimbal):
    result=dict.fromkeys(GEOMETRY_FIELDS,'')
    result.update(gcomp_valid=0,gcomp_reason='missing_capture_time')
    if cap_mono is None:
        return result
    target=cap_mono-float(getattr(cfg,'CAPTURE_LATENCY_MS',0.))/1000.
    result['gcomp_capture_mono']=target
    pose=pose_history.at(target+cfg.GCOMP_POSE_RX_DELAY_S,cfg.GCOMP_MAX_SYNC_GAP_S)
    att=gimbal.get_attitude_at(target+cfg.GCOMP_GIMBAL_RX_DELAY_S,cfg.GCOMP_MAX_SYNC_GAP_S)
    if pose is None or att is None:
        result['gcomp_reason']='missing_or_stale_attitude'
        return result
    pa,ga=pose['angles'],att['angles']
    result.update(gcomp_pose_method=pose['method'],gcomp_gimbal_method=att['method'],
        gcomp_pose_gap_ms=pose['distance_s']*1000,gcomp_gimbal_gap_ms=att['distance_s']*1000,
        gcomp_ac_roll=pa[0],gcomp_ac_pitch=pa[1],gcomp_ac_yaw=pa[2],gcomp_ac_course=pa[3],
        gcomp_gimbal_yaw=ga[0],gcomp_gimbal_pitch=ga[1],gcomp_gimbal_roll=ga[2])
    if not getattr(st,'detected',False):
        result['gcomp_reason']='no_detection'
        return result
    pixel=transform_pixel(cfg,virtual_cfg,st.bbox_u,st.bbox_v,pa,ga)
    if pixel is None:
        result['gcomp_reason']='ray_behind_reference_camera'
        return result
    result.update(gcomp_valid=1,gcomp_reason='ok',gcomp_u=pixel[0],gcomp_v=pixel[1])
    return result

class GimbalCompPIDLogger:
    def __init__(self,cfg,gcfg):
        self.cfg=cfg
        self.raw=RealVisionPIDLogger(cfg,gcfg)
        # Reuse identical PID but feed corrected coordinates at virtual zoom.
        ccfg=copy.copy(cfg)
        ccfg.ZOOM_LEVEL=cfg.VIRTUAL_ZOOM_LEVEL
        self.comp=RealVisionPIDLogger(ccfg,gcfg)
        self.enabled=self.raw.enabled

    def tick(self,sm,out,inp,seq_before,st,frame_idx,raw_idx,cap,ready,now,wall,origin,geometry=None):
        self.raw.enabled=self.enabled
        self.comp.enabled=self.enabled and self.cfg.GCOMP_ENABLED
        raw=self.raw.tick(sm,out,inp,seq_before,st,frame_idx,raw_idx,cap,ready,now,wall,origin)
        geom=dict(geometry or {})
        cst=SimpleNamespace(detected=bool(geom.get('gcomp_valid',False)),
             bbox_u=geom.get('gcomp_u') if geom.get('gcomp_valid') else None,
             bbox_v=geom.get('gcomp_v') if geom.get('gcomp_valid') else None)
        corrected=self.comp.tick(sm,out,inp,seq_before,cst,frame_idx,raw_idx,cap,ready,now,wall,origin)
        raw.update({k.replace('real_','real_comp_',1):v for k,v in corrected.items()})
        raw.update({k:geom.get(k,'') for k in GEOMETRY_FIELDS})
        if self.comp.enabled and geom.get('gcomp_valid') and sm.pid.update_seq!=seq_before:
            raw['gcomp_u_err_deg']=math.degrees(math.atan2(geom['gcomp_u']-sm._sp_u,sm.cfg.FX))
            raw['gcomp_v_err_deg']=math.degrees(math.atan2(geom['gcomp_v']-sm._sp_v,sm.cfg.FY))
        return raw
