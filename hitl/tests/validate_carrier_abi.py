"""Check the real-arming carrier gate and continuous sensor clock, no hardware."""
import argparse,ctypes as C,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from host.abi import Model,Initial,Input,Output,header

def validate(path):
 m=Model(path);ic=header(Initial);ic.latitude_deg=22.91;ic.longitude_deg=120.31
 cfg=dict(wait_s=1.,climb_s=4.,cruise_s=1.,height_m=100.,speed_m_s=30*1852/3600)
 m.check(m.lib.hil_model_reset(C.byref(ic)));m.configure_carrier(cfg)
 i=header(Input);o=header(Output);last_t=-1;gps_last=-1;released=None
 for k in range(2301):
  i.step_index=k;i.actuator_valid=i.actuator_new=1;i.controls[4]=.12
  i.fc_armed=2 if k<250 else 0 if k<500 else 1
  m.check(m.lib.hil_model_step(C.byref(i),C.byref(o)))
  assert o.model_time_ns==k*4000000 and o.model_time_ns>last_t;last_t=o.model_time_ns
  assert o.gps_sample_time_ns>=gps_last;gps_last=o.gps_sample_time_ns
  if k<500:
   assert o.reserved==1 and abs(o.raw_source[19])<1e-9 and abs(o.raw_source[27])<1e-9
  if o.reserved==3 and released is None:
   released=k;assert k==2000
   assert abs(o.raw_source[19]-100)<1e-9 and abs(o.raw_source[27]-cfg['speed_m_s'])<1e-9
   assert np.max(abs(np.array(o.raw_source[28:31])))<1e-9
 # Disarm never authorizes continued carriage/free flight.
 i.step_index=2301;i.fc_armed=0
 assert m.lib.hil_model_step(C.byref(i),C.byref(o))!=0
 m.check(m.lib.hil_model_reset(C.byref(ic)));m.configure_carrier(cfg)
 assert m.lib.hil_model_configure_carrier is not None
 # Regression: a prescribed ten-second cruise must not look like a frozen
 # pressure sensor after MAVLink float hPa encoding. Exercise real sensor code,
 # not the analytic carrier curve. The same reset/seed must reproduce every bit.
 pressure_runs=[]
 cruise_cfg=dict(cfg,cruise_s=10.)
 for repeat in range(2):
  m.check(m.lib.hil_model_reset(C.byref(ic)));m.configure_carrier(cruise_cfg)
  pressure=[];i.fc_armed=1
  for k in range(3750):
   i.step_index=k
   m.check(m.lib.hil_model_step(C.byref(i),C.byref(o)))
   if k>=1250:pressure.append(o.differential_pressure_pa)
  pressure_runs.append(np.array(pressure))
 assert np.array_equal(*pressure_runs),'carrier noise is not reproducible'
 pressure=pressure_runs[0]
 wire=np.float32(pressure/100.)
 assert np.std(pressure)>.05 and np.std(pressure)<.15
 assert np.max(abs(pressure-np.mean(pressure)))<1.
 assert np.mean(wire[1:]!=wire[:-1])>.99,'pressure freezes after wire quantization'
 # PX4 averages pressure over approximately 50 ms before its IAS stuck test.
 averaged=wire[:len(wire)//13*13].reshape(-1,13).mean(axis=1)
 # An occasional equal adjacent float is harmless; PX4 rejects a continuous
 # two-second interval. Use its FLT_EPSILON IAS comparison and 2 s timeout.
 ias=np.float32(np.sqrt(2.*averaged*100./1.225))
 run=longest=0
 for previous,current in zip(ias[:-1],ias[1:]):
  run=run+1 if abs(float(current)-float(previous))<=np.finfo(np.float32).eps else 0
  longest=max(longest,run)
 assert longest*13*.004<2.,'averaged IAS triggers PX4 two-second stuck check'
 pressure_sigma=float(np.std(pressure))
 # Air-start and nonzero attitude cannot select this carrier path.
 for field in ['height_m','euler_deg']:
  bad=header(Initial);bad.latitude_deg=22.91;bad.longitude_deg=120.31
  if field=='height_m':bad.height_m=100
  else:bad.euler_deg[1]=1
  m.check(m.lib.hil_model_reset(C.byref(bad)))
  try:m.configure_carrier(cfg)
  except RuntimeError:pass
  else:raise AssertionError('non-ground carrier accepted')
 m.lib.hil_model_terminate()
 return {'status':'PASS','release_at_model_s':released*.004,'armed_at_model_s':2.,
         'cruise_pressure_noise_std_pa':pressure_sigma,
         'longest_averaged_ias_equal_interval_s':longest*13*.004,
         'checks':['unknown_not_armed','unarmed_ground_hold','release_after_confirmed_arm',
                   'model_and_GPS_clock_continuity','release_state','disarm_fault','air_start_refused',
                   'ten_second_cruise_pressure_not_stuck','wire_float_pressure_not_stuck',
                   'averaged_pressure_not_stuck','fixed_seed_reproducibility']}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--library',type=Path,required=True);p.add_argument('--out',type=Path);a=p.parse_args()
 result=json.dumps(validate(a.library.resolve()),indent=2)
 if a.out:a.out.write_text(result+'\n')
 print(result)
