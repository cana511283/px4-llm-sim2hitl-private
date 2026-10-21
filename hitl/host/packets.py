import math,struct
from pymavlink.dialects.v20 import common as mav
def finite_range(v,lo,hi,name):
 if not math.isfinite(v) or not lo<=v<=hi:raise ValueError(name+' outside wire range')
 return v
def integer(v,scale,lo,hi,name):return int(round(finite_range(v*scale,lo,hi,name)))
def baro_metadata(pressure_pa,fields):
 # Derived ISA transport metadata; raw model channels and ABI validity stay unchanged.
 if not fields & 0x200:return 0.,0.,fields
 if not math.isfinite(pressure_pa) or pressure_pa<=0:raise ValueError('Barometric pressure must be finite and positive')
 temperature_k=288.15*(pressure_pa/101325.)**0.1902632365084836
 return (288.15-temperature_k)/.0065,temperature_k-273.15,fields|0x1800
class Encoder:
 def __init__(self,quality,flat_rangefinder=False):
  self.flat_rangefinder=flat_rangefinder
  self.writer=mav.MAVLink(None,srcSystem=2,srcComponent=51);self.quality=quality;self.gps_time=None;self.heartbeat_time=None
 def encode(self,o,epoch_us):
  # Validate all physical and wire fields before packing any message.
  pressure_alt,temperature,fields=baro_metadata(o.absolute_pressure_pa,o.fields_updated)
  values=[*o.accel_m_s2,*o.gyro_rad_s,*[v*.01 for v in o.magnetic_uT],o.absolute_pressure_pa*.01,o.differential_pressure_pa*.01,pressure_alt,temperature]
  for v in values:finite_range(v,-3.4028234e38,3.4028234e38,'HIL_SENSOR float32')
  stamp=epoch_us+o.model_time_ns//1000
  msgs=[mav.MAVLink_hil_sensor_message(stamp,*values,fields)]
  if o.gps_sample_time_ns!=self.gps_time:
   lat,lon,alt=o.gps_lla_msl;q=self.quality
   msgs.append(mav.MAVLink_hil_gps_message(epoch_us+o.gps_sample_time_ns//1000,q['fix_type'],integer(lat,1e7,-900000000,900000000,'latitude'),integer(lon,1e7,-1800000000,1800000000,'longitude'),integer(alt,1000,-2147483648,2147483647,'altitude'),q['eph_cm'],q['epv_cm'],integer(o.ground_speed_m_s,100,0,65534,'ground speed'),*[integer(v,100,-32768,32767,'NED velocity') for v in o.velocity_ned_m_s],round(o.course_deg*100)%36000,q['satellites']))
  if self.heartbeat_time is None or o.model_time_ns-self.heartbeat_time>=1000000000:msgs.append(mav.MAVLink_heartbeat_message(1,12,0,0,0,3))
  if self.flat_rangefinder and o.model_time_ns%40000000==0:
   pitch,roll,agl=o.raw_source[11],o.raw_source[12],o.raw_source[19]
   down=math.cos(pitch)*math.cos(roll)
   distance=agl/down if down>0 else float('nan')
   valid=all(math.isfinite(v) for v in [pitch,roll,agl,distance]) and agl>=0 and .05<=distance<=150
   msgs.append(mav.MAVLink_distance_sensor_message((stamp//1000)&0xffffffff,5,15000,round(distance*100) if valid else 15001,0,0,25,255,0,0,[0,0,0,0],100 if valid else 1))
  frames=[]
  for msg in msgs:
   frames.append((msg.get_type(),msg.pack(self.writer)));self.writer.seq=(self.writer.seq+1)%256
  self.gps_time=o.gps_sample_time_ns
  if any(k=='HEARTBEAT' for k,_ in frames):self.heartbeat_time=o.model_time_ns
  return frames
