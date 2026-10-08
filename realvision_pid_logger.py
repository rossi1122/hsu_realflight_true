"""Independent real-YOLO PID observer. No ROS, bridge or actuator access."""
import copy
import math

from dive_guidance import DivePID

REAL_PID_FIELDS = [
    'real_pid_enabled', 'real_pid_active', 'real_pid_updated',
    'real_pid_detected', 'real_pid_frame_idx', 'real_pid_raw_frame_idx',
    'real_pid_new_frame', 'real_pid_cap_t', 'real_pid_ready_t',
    'real_pid_age_ms', 'real_pid_dt', 'real_pid_u', 'real_pid_v',
    'real_pid_u_sp', 'real_pid_v_sp', 'real_pid_fx', 'real_pid_fy',
    'real_pid_u_err_deg', 'real_pid_v_err_deg',
    'real_pid_lat_p', 'real_pid_lat_i', 'real_pid_lat_d',
    'real_pid_lon_p', 'real_pid_lon_i', 'real_pid_lon_d',
    'real_pid_dpsi_deg', 'real_pid_dtheta_deg',
    'real_pitch_cmd', 'real_heading_cmd', 'real_pid_compute_ms',
]


def finite(value):
    return value is not None and math.isfinite(float(value))


class RealVisionPIDLogger:
    def __init__(self, cfg, virtual_cfg):
        self.enabled = bool(getattr(cfg, 'REALVISION_PID_LOG_ENABLED', True))
        self.max_age = float(getattr(cfg, 'REALVISION_PID_MAX_AGE_S', 0.5))
        self.capture_latency = float(getattr(cfg, 'CAPTURE_LATENCY_MS', 0.0)) / 1000.0
        self.cfg = copy.deepcopy(virtual_cfg)
        self.cfg.FX, self.cfg.FY, self.cfg.CX, self.cfg.CY = cfg.effective_intrinsics()
        self.vfx, self.vfy = virtual_cfg.FX, virtual_cfg.FY
        self.vcx, self.vcy = virtual_cfg.CX, virtual_cfg.CY
        self.pid = DivePID(self.cfg)
        self.in_dive = False
        self.entry_t = None
        self.last_frame = None

    def tick(self, sm, out, inp, seq_before, st, frame_idx, raw_idx,
             cap_mono, ready_mono, now_mono, now_wall, origin_wall):
        row = dict.fromkeys(REAL_PID_FIELDS, '')
        row.update(real_pid_enabled=int(self.enabled), real_pid_active=0,
                   real_pid_updated=0, real_pid_detected=0, real_pid_new_frame=0)
        in_dive = out['mode'] == 'DIVE'
        if not self.enabled:
            return row
        if in_dive and not self.in_dive:
            # State machine resets its PID on entry. Copy these exact seeds.
            self.pid.reset(sm.pid.pitch_cmd, sm.pid.heading_cmd)
            self.entry_t = inp['t']
            self.last_frame = None
        self.in_dive = in_dive
        if not in_dive:
            return row

        updated = sm.pid.update_seq != seq_before
        u_sp = self.cfg.CX + (sm._sp_u - self.vcx) * self.cfg.FX / self.vfx
        v_sp = self.cfg.CY + (sm._sp_v - self.vcy) * self.cfg.FY / self.vfy
        age = now_mono - cap_mono + self.capture_latency if cap_mono is not None else None
        cap_t = now_wall - origin_wall - (now_mono - cap_mono) if cap_mono is not None else None
        ready_t = now_wall - origin_wall - (now_mono - ready_mono) if ready_mono is not None else None
        u = getattr(st, 'bbox_u', None)
        v = getattr(st, 'bbox_v', None)
        detected = bool(getattr(st, 'detected', False) and finite(u) and finite(v)
                        and age is not None and 0 <= age <= self.max_age
                        and cap_t - self.capture_latency >= self.entry_t)
        if updated:
            pitch = inp.get('pitch_deg')
            if pitch is None:
                pitch = out['pitch_cmd_deg']
            if self.cfg.BENCH_DIVE_PITCH_BASE_DEG is not None:
                pitch = float(self.cfg.BENCH_DIVE_PITCH_BASE_DEG)
            vn, ve = inp.get('vn'), inp.get('ve')
            course = (math.degrees(math.atan2(ve, vn))
                      if vn is not None and ve is not None and vn * vn + ve * ve > 4
                      else (inp.get('yaw_true_deg') or 0.0))
            self.pid.update(u, v, u_sp, v_sp, detected, pitch, course, sm.pid.last_dt)
            if self.cfg.DIVE_HOLD_HEADING:
                self.pid.heading_cmd = out['heading_cmd_deg']
        # Keep PID reset seeds untouched during settle, as the virtual PID does.

        row.update(real_pid_active=int(updated), real_pid_updated=int(updated),
                   real_pid_detected=int(detected), real_pid_frame_idx=frame_idx,
                   real_pid_raw_frame_idx=raw_idx,
                   real_pid_new_frame=int(raw_idx >= 0 and raw_idx != self.last_frame),
                   real_pid_cap_t=cap_t if cap_t is not None else '',
                   real_pid_ready_t=ready_t if ready_t is not None else '',
                   real_pid_age_ms=age * 1000 if age is not None else '',
                   real_pid_dt=sm.pid.last_dt if updated else '',
                   real_pid_u=u if finite(u) else '', real_pid_v=v if finite(v) else '',
                   real_pid_u_sp=u_sp, real_pid_v_sp=v_sp,
                   real_pid_fx=self.cfg.FX, real_pid_fy=self.cfg.FY,
                   real_pitch_cmd=self.pid.pitch_cmd if updated else out['pitch_cmd_deg'],
                   real_heading_cmd=self.pid.heading_cmd if updated else out['heading_cmd_deg'],
                   real_pid_compute_ms=self.pid.last_compute_ms if updated else '')
        self.last_frame = raw_idx
        if updated:
            for axis, pid in [('lat', self.pid.lat), ('lon', self.pid.lon)]:
                for term in ('p', 'i', 'd'):
                    row['real_pid_' + axis + '_' + term] = getattr(pid, term)
            # A lost frame retains internal info; do not label it a fresh error.
            if detected:
                row.update(real_pid_u_err_deg=self.pid.info['u_err_deg'],
                           real_pid_v_err_deg=self.pid.info['v_err_deg'],
                           real_pid_dpsi_deg=self.pid.info['dpsi'],
                           real_pid_dtheta_deg=self.pid.info['dtheta'])
        return row
