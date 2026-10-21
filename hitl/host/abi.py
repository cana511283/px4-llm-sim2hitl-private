"""Independent ctypes validation; no MATLAB engine, sockets, or serial ports."""
import argparse,ctypes as C,json,time,platform,subprocess,threading
from pathlib import Path
import numpy as np
ABI=0x10001
class Initial(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32),('height_m',C.c_double),('body_velocity_m_s',C.c_double*3),('euler_deg',C.c_double*3),('latitude_deg',C.c_double),('longitude_deg',C.c_double),('geoid_separation_m',C.c_double),('seed_policy',C.c_uint32),('reserved',C.c_uint32*7)]
class Input(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32),('controls',C.c_double*16),('step_index',C.c_uint64),('actuator_age_ns',C.c_uint64),('input_sequence',C.c_uint64),('actuator_valid',C.c_uint32),('actuator_new',C.c_uint32),('fc_armed',C.c_uint32),('reserved',C.c_uint32)]
class Output(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32),('model_time_ns',C.c_uint64),('imu_sample_time_ns',C.c_uint64),('gps_sample_time_ns',C.c_uint64),('accel_m_s2',C.c_double*3),('gyro_rad_s',C.c_double*3),('magnetic_uT',C.c_double*3),('absolute_pressure_pa',C.c_double),('differential_pressure_pa',C.c_double),('gps_lla_msl',C.c_double*3),('velocity_ned_m_s',C.c_double*3),('ground_speed_m_s',C.c_double),('course_deg',C.c_double),('raw_source',C.c_double*58),('imu_valid',C.c_uint32),('imu_updated',C.c_uint32),('gps_valid',C.c_uint32),('gps_updated',C.c_uint32),('pressure_height_valid',C.c_uint32),('temperature_valid',C.c_uint32),('fields_updated',C.c_uint32),('reserved',C.c_uint32)]
class Info(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32),('initial_size',C.c_uint32),('input_size',C.c_uint32),('output_size',C.c_uint32),('output_phase',C.c_uint32),('fixed_step_ns',C.c_uint64),('imu_period_ns',C.c_uint64),('gps_period_ns',C.c_uint64),('aircraft_id',C.c_char*32),('build_id',C.c_char*64),('source_sha256',C.c_char*65),('platform',C.c_char*32),('raw_count',C.c_uint32),('seed_policy',C.c_uint32),('instance_count',C.c_uint32),('reserved',C.c_uint32),('capabilities',C.c_uint64),('initial_supported',C.c_uint64)]
class Wind(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32),('steady_ned_m_s',C.c_double*3),('start_mode',C.c_uint32),('reserved0',C.c_uint32),('start_height_m',C.c_double),('start_time_s',C.c_double),('ramp_s',C.c_double),('gust_start_s',C.c_double),('gust_length_m',C.c_double*3),('gust_magnitude_m_s',C.c_double*3),('shear_speed_6m_m_s',C.c_double),('shear_from_deg',C.c_double),('turbulence_speed_6m_m_s',C.c_double),('turbulence_from_deg',C.c_double),('turbulence_seed',C.c_uint32*4)]
class Carrier(C.Structure):
 _fields_=[('abi_version',C.c_uint32),('struct_size',C.c_uint32)]+[(k,C.c_double) for k in ['wait_s','climb_s','cruise_s','height_m','speed_m_s']]
CAP_CARRIER=512
CAP_WIND=256
def header(cls):
 x=cls();x.abi_version=ABI;x.struct_size=C.sizeof(cls);return x
def initial(air=False):
 x=header(Initial);x.height_m=100 if air else 0;x.body_velocity_m_s[0]=20 if air else 0
 x.euler_deg[1]=.8633;x.euler_deg[2]=25 if air else 0;x.latitude_deg=22.910275834988674;x.longitude_deg=120.31096929726908;return x
class Model:
 def __init__(self,path):
  self.lib=C.CDLL(str(path))
  self.lib.hil_model_step.argtypes=[C.POINTER(Input),C.POINTER(Output)]
  self.lib.hil_model_initialize.argtypes=[C.POINTER(Initial)];self.lib.hil_model_reset.argtypes=[C.POINTER(Initial)]
  self.lib.hil_model_get_info.argtypes=[C.POINTER(Info)];self.lib.hil_model_get_last_error.argtypes=[C.c_void_p,C.c_uint32]
  self.info=header(Info);self.check(self.lib.hil_model_get_info(C.byref(self.info)))
  assert [self.info.initial_size,self.info.input_size,self.info.output_size]==[C.sizeof(Initial),C.sizeof(Input),C.sizeof(Output)]
 def check(self,code):
  if code:
   b=C.create_string_buffer(256);self.lib.hil_model_get_last_error(b,256);raise RuntimeError(f'{code}: {b.value.decode()}')
 def configure_wind(self,config):
  # config: plant/wind.py parse() (HilWindConfig field names). After reset, before the first step.
  if not self.info.capabilities&CAP_WIND or not hasattr(self.lib,'hil_model_configure_wind'):raise ValueError('Release has no wind support')
  w=header(Wind)
  for name,value in config.items():
   if isinstance(value,list):getattr(w,name)[:]=value
   else:setattr(w,name,value)
  function=self.lib.hil_model_configure_wind;function.argtypes=[C.POINTER(Wind)];function.restype=C.c_int32
  self.check(function(C.byref(w)))
 def configure_carrier(self,config):
  if not self.info.capabilities&CAP_CARRIER or not hasattr(self.lib,'hil_model_configure_carrier'):
   raise ValueError('Release has no carrier support')
  c=header(Carrier)
  for name,value in config.items():setattr(c,name,value)
  fn=self.lib.hil_model_configure_carrier;fn.argtypes=[C.POINTER(Carrier)];fn.restype=C.c_int32
  self.check(fn(C.byref(c)))
 def reset(self,air=False): self.check(self.lib.hil_model_reset(C.byref(initial(air))))
 def run(self,controls,air=False,flags=None,wind=None):
  self.reset(air)
  if wind is not None:self.configure_wind(wind)
  i=header(Input);o=header(Output);rows=[];canonical=[]
  for k,u in enumerate(controls):
   i.step_index=k;i.input_sequence=k;i.actuator_valid=i.actuator_new=1;i.fc_armed=k%3
   if flags is not None: i.actuator_valid,i.actuator_new=flags[k]
   i.controls[:]=u;self.check(self.lib.hil_model_step(C.byref(i),C.byref(o)))
   rows.append([o.model_time_ns/1e9,*o.raw_source]);canonical.append(bytes(o))
  return np.array(rows),canonical

class Trace(C.Structure):
 _fields_=[('index',C.c_uint64),('duration_ns',C.c_uint64),('lateness_ns',C.c_uint64),('lag_ns',C.c_int64)]
class Snapshot(C.Structure):
 _fields_=[('output',Output),('produced',C.c_uint64),('misses',C.c_uint64),('max_lateness_ns',C.c_uint64),('max_step_ns',C.c_uint64),('start_ns',C.c_uint64),('now_ns',C.c_uint64),('running',C.c_int32),('fault',C.c_int32)]
class Worker:
 def __init__(self,path,model):
  self.lib=C.CDLL(str(path));self.model=model;self.stop_lock=threading.RLock()
  if not hasattr(self.lib,'hil_io_clear_commands'):raise ValueError('Native worker lacks the command-stop barrier; update the release before starting HIL')
  self.lib.hil_io_clear_commands.argtypes=[];self.lib.hil_io_clear_commands.restype=None
  self.lib.hil_worker_now_ns.restype=C.c_uint64
  self.lib.hil_worker_start.argtypes=[C.c_void_p,C.POINTER(Input),C.c_uint64]
  self.lib.hil_worker_input.argtypes=[C.POINTER(Input)]
  self.lib.hil_worker_snapshot.argtypes=[C.POINTER(Snapshot)]
  self.lib.hil_worker_drain.argtypes=[C.POINTER(Trace),C.c_uint32]
  self.lib.hil_io_command.argtypes=[C.c_void_p,C.c_uint32]
  self.lib.hil_io_config.argtypes=[C.c_uint64,C.c_uint32]
  if hasattr(self.lib,"hil_io_flat_rangefinder"):
   self.lib.hil_io_flat_rangefinder.argtypes=[C.c_uint32]
   self.lib.hil_io_range_count.restype=C.c_uint64
  self.lib.hil_io_encode.argtypes=[C.POINTER(Output),C.c_uint64,C.c_void_p,C.c_uint32,C.c_uint32]
  self.lib.hil_io_drain.argtypes=[C.POINTER(Packet),C.c_uint32]
  self.lib.hil_io_stats.argtypes=[C.POINTER(IoStats)]
  for name in ['hil_io_drain_events','hil_io_diagnostics','hil_worker_diagnostics']:
   if not hasattr(self.lib,name):raise ValueError('Native worker lacks stability diagnostics; update the complete release')
  self.lib.hil_io_drain_events.argtypes=[C.POINTER(IoEvent),C.c_uint32]
  self.lib.hil_io_diagnostics.argtypes=[C.POINTER(IoEvent),C.POINTER(C.c_uint64)]
  self.lib.hil_worker_diagnostics.argtypes=[C.POINTER(WorkerDiagnostic)]
 def now(self):return self.lib.hil_worker_now_ns()
 def start(self):
  code=self.lib.hil_worker_start(C.cast(self.model.lib.hil_model_step,C.c_void_p),C.byref(header(Input)),20000000)
  if code:raise RuntimeError('native worker start failed '+str(code))
 def snapshot(self):
  value=Snapshot();self.lib.hil_worker_snapshot(C.byref(value));return value
 def stop(self):
  with self.stop_lock:self.lib.hil_worker_stop();self.lib.hil_io_clear_sensor()
 def drain(self):
  values=(Trace*1024)();n=self.lib.hil_worker_drain(values,1024)
  return [(v.index,v.duration_ns,v.lateness_ns,v.lag_ns) for v in values[:n]]

class Packet(C.Structure):
 _fields_=[('stamp',C.c_uint64),('len',C.c_uint32),('kind',C.c_uint32),('bytes',C.c_uint8*280)]
class IoStats(C.Structure):
 _fields_=[(name,C.c_uint64) for name in ['sensor','gps','heartbeat','commands','replaced','errors','log_drops','max_write_ns']]

def io_stats(worker):
 value=IoStats();worker.lib.hil_io_stats(C.byref(value));return {n:getattr(value,n) for n,_ in IoStats._fields_}
def io_drain(worker):
 values=(Packet*512)();n=worker.lib.hil_io_drain(values,512);return [(v.stamp,v.kind,bytes(v.bytes[:v.len])) for v in values[:n]]

class IoEvent(C.Structure):
 _fields_=[(n,C.c_uint64) for n in ['begin_ns','end_ns','packet_stamp_ns']]+[(n,C.c_uint32) for n in ['len','offset','kind','reason']]+[(n,C.c_int32) for n in ['write_errno','poll_result','poll_revents']]+[(n,C.c_uint32) for n in ['retries','short_writes','reserved']]
class WorkerDiagnostic(C.Structure):
 _fields_=[(n,C.c_uint64) for n in ['previous_begin_ns','previous_end_ns','publish_begin_ns','publish_end_ns','deadline_ns','wake_ns','fault_index']]
def struct_dict(value):return {name:getattr(value,name) for name,_ in value._fields_}
def io_events(worker):
 values=(IoEvent*256)();n=worker.lib.hil_io_drain_events(values,256);return [struct_dict(v) for v in values[:n]]
def diagnostics(worker):
 event=IoEvent();counts=(C.c_uint64*2)();worker.lib.hil_io_diagnostics(C.byref(event),counts)
 value=WorkerDiagnostic();worker.lib.hil_worker_diagnostics(C.byref(value))
 return {'io_last_event':struct_dict(event),'io_events':counts[0],'io_event_drops':counts[1],'worker':struct_dict(value)}
def clock_calibration(worker):
 pairs=[]
 for _ in range(20):
  before=time.monotonic_ns();native=worker.now();after=time.monotonic_ns();pairs.append((after-before,native-(before+after)//2,before,after,native))
 width,offset,before,after,native=min(pairs)
 return {'python_to_native_offset_ns':offset,'uncertainty_ns':(width+1)//2,'python_before_ns':before,'python_after_ns':after,'native_ns':native}
