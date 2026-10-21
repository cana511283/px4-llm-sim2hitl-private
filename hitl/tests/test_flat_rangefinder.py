"""Decode real native wire frames; verify tilted plane range and invalid readings."""
import ctypes as C, math, os, sys, time, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from host.abi import Output,header
from host.packets import Encoder
from host.scenario import load_scene
from pymavlink.dialects.v20 import common as mav

class FlatRangeTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  import tempfile,update,subprocess
  cls.temp=tempfile.TemporaryDirectory();folder=Path(cls.temp.name)
  wrapper=folder/'probe.c';wrapper.write_text('#include "hil_model_api.h"\nextern int hil_io_publish(const HilModelOutput*);\n__attribute__((visibility("default"))) int probe_publish(const HilModelOutput*o){return hil_io_publish(o);}')
  cmd=update.worker_command(folder);cmd.insert(-3,str(wrapper));subprocess.run(cmd,check=True);cls.lib=C.CDLL(cmd[-1])
  cls.lib.hil_io_encode.argtypes=[C.POINTER(Output),C.c_uint64,C.c_void_p,C.c_uint32,C.c_uint32]
  cls.lib.hil_io_range_count.restype=C.c_uint64
 def output(self,h=100,roll=0,pitch=0):
  o=header(Output);o.absolute_pressure_pa=101325;o.raw_source[19]=h;o.raw_source[11]=pitch;o.raw_source[12]=roll
  return o
 def decode(self,data):return mav.MAVLink(None).parse_buffer(data) or []
 def native(self,o,flags=4):
  b=C.create_string_buffer(280);n=self.lib.hil_io_encode(C.byref(o),1000000,b,280,flags);self.assertGreater(n,0);return self.decode(b.raw[:n])
 def test_level_tilt_limits_and_wire_reference(self):
  for h,roll,pitch,expected in [(100,0,0,10000),(100,math.acos(.8),0,12500),(100,0,math.acos(.8),12500),(151,0,0,15001),(0,0,0,15001),(-.001,0,0,15001),(10,math.pi,0,15001),(.05,0,0,5),(150,0,0,15000)]:
   with self.subTest(h=h,roll=roll,pitch=pitch):
    o=self.output(h,roll,pitch);m=self.native(o)[-1];self.assertEqual(m.get_type(),'DISTANCE_SENSOR');self.assertEqual(m.current_distance,expected);self.assertEqual(m.orientation,25);self.assertEqual(m.signal_quality,1 if expected==15001 else 100)
    frames=Encoder({'fix_type':3,'eph_cm':30,'epv_cm':50,'satellites':10},True).encode(o,1000000)
    ref=[self.decode(v)[0] for k,v in frames if k=='DISTANCE_SENSOR'][0]
    self.assertEqual(m.to_dict(),ref.to_dict())
 def test_opt_in_and_cadence(self):
  self.assertEqual([m.get_type() for m in self.native(self.output(),0)],['HIL_SENSOR'])
  rd,wr=os.pipe();os.set_blocking(rd,False)
  try:
   self.assertEqual(self.lib.hil_io_start(wr),0);self.lib.hil_io_config(1000000,2563);self.lib.hil_io_flat_rangefinder(1)
   for k in range(21):
    o=self.output();o.model_time_ns=k*4000000;self.assertEqual(self.lib.probe_publish(C.byref(o)),0);time.sleep(.002)
   time.sleep(.02);msgs=self.decode(os.read(rd,10000));ds=[m for m in msgs if m.get_type()=='DISTANCE_SENSOR'];self.assertEqual(len(ds),3);self.assertEqual([m.time_boot_ms for m in ds],[1000,1040,1080]);self.assertEqual(self.lib.hil_io_range_count(),3)
   self.lib.hil_io_clear_sensor();o=self.output();self.lib.probe_publish(C.byref(o));time.sleep(.01)
   with self.assertRaises(BlockingIOError):os.read(rd,10000)
  finally:self.lib.hil_io_stop();os.close(rd);os.close(wr)
 def test_scene_rejects_nonboolean(self):
  scene={'schema_version':1,'aircraft_id':'skysurfer','flat_rangefinder':True};self.assertTrue(load_scene(scene)[0]['flat_rangefinder'])
  for value in [1,'yes',None,{}]:
   with self.assertRaises(ValueError):load_scene(dict(scene,flat_rangefinder=value))

if __name__=='__main__':unittest.main(verbosity=2)
