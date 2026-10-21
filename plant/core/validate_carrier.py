#!/usr/bin/env python3
"""Offline validation of prescribed carrier kinematics and release continuity.
No FC, arming stubs or runtime recovery are involved. Not a flight-quality verdict.
"""
import argparse
import ctypes as C
import json
from pathlib import Path
import numpy as np
import validate_plant as vp

D = C.c_double
class Config(C.Structure):
    _fields_ = [(k,D) for k in ['wait_s','climb_s','cruise_s','height_m','speed_m_s']]
class Sample(C.Structure):
    _fields_ = [('rigid',D*13),('accel',D*3),('carried',C.c_uint32)]

def validate(path):
    lib=vp.Lib(path); dll=lib.lib
    dll.plant_carrier_sample.argtypes=[C.POINTER(Config),D,C.POINTER(Sample)]
    dll.plant_carrier_step.argtypes=[C.c_void_p,C.POINTER(vp.Input),C.POINTER(Config),C.POINTER(vp.Truth)]
    cfg=Config(10,40,10,100,30*1852/3600)
    def sample(t):
        s=Sample();assert dll.plant_carrier_sample(C.byref(cfg),t,C.byref(s))==0
        return np.array(s.rigid),np.array(s.accel),s.carried
    # Analytic derivatives checked against finite differences, away from boundaries.
    for t in [2,10.5,20,30,49.5,55]:
        x,a,_=sample(t);xp,_,_=sample(t+1e-3);xm,_,_=sample(t-1e-3)
        assert np.max(abs((xp[:3]-xm[:3])/.002-x[3:6]))<1e-6
        assert np.max(abs((xp[3:6]-xm[3:6])/.002-a))<1e-6
    for t in [10,50,60]:
        xp,ap,_=sample(t+1e-6);xm,am,_=sample(t-1e-6)
        assert np.max(abs(xp-xm))<4e-5
        assert np.max(abs(ap-am))<1e-5
    for k in range(5):
        bad=Config(*[getattr(cfg,n) for n,_ in Config._fields_]);setattr(bad,Config._fields_[k][0],float('nan'))
        assert dll.plant_carrier_sample(C.byref(bad),0,C.byref(Sample()))!=0
    ic=vp.Initial();ic.latitude_deg=22.910275834988674;ic.longitude_deg=120.31096929726908
    p=lib.plant();assert dll.plant_init(p,C.byref(ic))==0
    u=vp.Input();truth=vp.Truth();dt=.004;previous=None
    for k in range(15001):
        # Vary commands before release so continuity is not just a zero-actuator special case.
        u.command[:]=[.03,-.02,.15,0,.12]
        assert dll.plant_carrier_step(p,C.byref(u),C.byref(cfg),C.byref(truth))==0
        assert abs(truth.time_s-k*dt)<1e-10
        v=np.r_[truth.position_ned_m,truth.velocity_body_m_s,truth.quaternion_nb,truth.rates_body_rad_s]
        prescribed,acc,carried=sample(k*dt)
        assert np.max(abs(v-prescribed))<1e-9
        if carried:
            assert np.max(abs(np.array(truth.specific_force_body_m_s2)-(acc-[0,0,9.80665])))<1e-9
        if k==15000:
            assert abs(truth.height_agl_m-100)<1e-9
            assert abs(truth.airspeed_true_m_s-cfg.speed_m_s)<1e-9
            assert np.max(abs(np.array(truth.surface_deflection_rad)-previous))<.01
        previous=np.array(truth.surface_deflection_rad)
    # Free step has its own acceleration; it must not remain pinned to the carrier.
    assert dll.plant_carrier_step(p,C.byref(u),C.byref(cfg),C.byref(truth))==0
    assert abs(truth.rates_body_rad_s[1])>1e-6
    return {'status':'PASS','cases':['finite_difference_kinematics','segment_continuity','invalid_config',
             '15000_carried_steps','release_position_velocity_attitude','nonzero_actuator_continuity',
             'free_dynamics_after_release'],'release_height_m':100,'release_speed_m_s':cfg.speed_m_s,
             'scope':'carrier physics boundary only; no FC/EKF/arming or flight-stability verdict'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--plant-lib',type=Path,required=True);p.add_argument('--out',type=Path);a=p.parse_args()
    result=json.dumps(validate(a.plant_lib.resolve()),indent=2)
    if a.out:a.out.write_text(result+'\n')
    print(result)
