"""Offline native mission protocol/lifecycle checks; no sockets or hardware."""
import sys,unittest,copy,tempfile,json
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_protocol import MissionExchange
from autotakeoff.autotakeoff import mav,SOURCE
from native_circuit import NativeCircuit,items,PARAMETERS

def fc(msg):
 msg._header.srcSystem=1;msg._header.srcComponent=1;return msg
class Link:
 def __init__(self,stored=()):self.stored=copy.deepcopy(list(stored));self.queue=[];self.sent=[];self.pending=[]
 def send(self,m):
  self.sent.append(m.get_type());kind=m.get_type()
  if kind=='MISSION_REQUEST_LIST':self.queue.append(fc(mav.MAVLink_mission_count_message(*SOURCE,len(self.stored),0)))
  elif kind=='MISSION_REQUEST_INT':
   v=self.stored[m.seq];p=[float('nan') if x is None else x for x in v['params']];xy=[round(x*1e7) for x in p[4:6]] if v['frame']!=2 else list(map(round,p[4:6]))
   self.queue.append(fc(mav.MAVLink_mission_item_int_message(*SOURCE,m.seq,v['frame'],v['command'],int(v['current']),int(v['autocontinue']),*p[:4],*xy,p[6],0)))
  elif kind=='MISSION_COUNT':
   self.count=m.count;self.pending=[]
   if m.count:self.queue.append(fc(mav.MAVLink_mission_request_int_message(*SOURCE,0,0)))
   else:self.stored=[];self.queue.append(fc(mav.MAVLink_mission_ack_message(*SOURCE,0,0)))
  elif kind=='MISSION_ITEM_INT':
   p=[m.param1,m.param2,m.param3,m.param4,m.x/1e7,m.y/1e7,m.z]
   # PX4 mavlink_mission.cpp parses/serializes this command's radius in
   # param2 and leaves unused param3 zero. An echo-only fixture missed this.
   if m.command==mav.MAV_CMD_NAV_LOITER_TO_ALT:p[2]=0
   if m.frame==2:p[4:6]=[m.x,m.y]
   import math
   self.pending.append(dict(seq=m.seq,command=m.command,frame=m.frame,current=bool(m.current),autocontinue=bool(m.autocontinue),params=[None if not math.isfinite(x) else x for x in p]))
   if len(self.pending)==self.count:self.stored=self.pending;self.queue.append(fc(mav.MAVLink_mission_ack_message(*SOURCE,0,0)))
   else:self.queue.append(fc(mav.MAVLink_mission_request_int_message(*SOURCE,len(self.pending),0)))
 def receive(self):q=self.queue;self.queue=[];return q
class Tests(unittest.TestCase):
 def test_recipe_keeps_firmware_required_by_deploy_validation(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);src=root/'base';fw=src/'firmware';fw.mkdir(parents=True)
   (fw/'image.px4').write_bytes(b'firmware fixture')
   (fw/'firmware.json').write_text('{"outputs":{"firmware":{"file":"image.px4"}}}')
   (fw/'parameters.json.xz').write_bytes(b'parameter definitions fixture')
   (src/'params.json').write_text('{"set":{}}');out=root/'result';out.mkdir()
   n=NativeCircuit.__new__(NativeCircuit);n.args=SimpleNamespace(out=out,release=src)
   for name in ['trial','restore']:
    recipe=n.recipe(name,PARAMETERS)
    self.assertEqual((recipe/'firmware/image.px4').read_bytes(),b'firmware fixture')
    self.assertTrue((recipe/'firmware/parameters.json.xz').is_file())
    self.assertEqual(json.loads((recipe/'params.json').read_text())['set']['FW_THR_TRIM']['value'],.121)
 def test_upload_download_restore_including_empty(self):
  l=Link();x=MissionExchange(l,lambda:None)
  self.assertEqual(x.download(),[]);x.upload(items());self.assertTrue(x.equal(items(),x.download()))
  x.upload([]);self.assertEqual(x.download(),[])
 def test_armed_gate_prevents_mutation(self):
  l=Link()
  def reject():raise RuntimeError('armed')
  with self.assertRaisesRegex(RuntimeError,'armed'):MissionExchange(l,reject).upload(items())
  self.assertEqual(l.sent,[])
 def test_early_ack_does_not_pass(self):
  class Bad(Link):
   def send(self,m):
    if m.get_type()=='MISSION_COUNT':self.queue=[fc(mav.MAVLink_mission_ack_message(*SOURCE,0,0))]
  with self.assertRaisesRegex(RuntimeError,'Early mission ACK'):MissionExchange(Bad(),lambda:None).upload(items())
 def test_rejected_mission_is_not_retried(self):
  class Bad(Link):
   def send(self,m):
    if m.get_type()=='MISSION_COUNT':self.sent.append('COUNT');self.queue=[fc(mav.MAVLink_mission_ack_message(*SOURCE,mav.MAV_MISSION_DENIED,0))]
  l=Bad()
  with self.assertRaisesRegex(RuntimeError,'rejected'):MissionExchange(l,lambda:None).upload(items())
  self.assertEqual(l.sent,['COUNT'])
 def test_mismatch_detected(self):
  b=copy.deepcopy(items());b[3]['params'][6]=99
  self.assertFalse(MissionExchange.equal(items(),b))
  b=copy.deepcopy(items());b[3]['params'][5]+=1e-5
  self.assertFalse(MissionExchange.equal(items(),b))
 def test_landing_requires_final_item_and_stopped(self):
  n=NativeCircuit.__new__(NativeCircuit);n.link=Link();n.mode_sent=False;n.result={};n.landed=True;n.last_seq=9;n.finished=False
  n.sequence_at=n.landed_at=42
  v=SimpleNamespace(mode=7,sub_mode=0,height=.1)
  self.assertFalse(n.flight_step(v,41,0,{}));self.assertTrue(n.mode_sent)
  v.mode=v.sub_mode=4
  self.assertFalse(n.flight_step(v,42,0,{'telemetry':{'VFR_HUD':{'values':{'groundspeed_m_s':3}}}}))
  self.assertTrue(n.flight_step(v,43,0,{'telemetry':{'VFR_HUD':{'values':{'groundspeed_m_s':.2}}}}))
 def test_restore_parameters_even_if_mission_restore_fails(self):
  n=NativeCircuit.__new__(NativeCircuit);n.mission_attempted=n.params_attempted=True;n.backup=[];n.result={};called=[]
  class Bad:
   def upload(self,v):raise RuntimeError('mission failure')
  n.exchange=Bad();n.restore_recipe=Path('/tmp/recipe');n.args=SimpleNamespace(out=Path('/tmp/results'))
  n.call=lambda action,args:(called.append(action) or {'verdict':'PASS'})
  with self.assertRaisesRegex(RuntimeError,'mission failure'):n.restore()
  self.assertEqual(called,['param-deploy']);self.assertEqual(n.result['parameter_restore'],'PASS')
 def test_native_geometry_and_mavlink_frames(self):
  a=items();self.assertEqual([x['name'] for x in a[2:7]],['upwind','crosswind','downwind','base','final_circuit'])
  self.assertTrue(all(x['frame']==mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT for x in a if x['command'] in [16,21,31]))
  self.assertEqual(a[-2]['command'],mav.MAV_CMD_NAV_LOITER_TO_ALT);self.assertEqual(a[-1]['command'],mav.MAV_CMD_NAV_LAND)
 def test_loiter_to_alt_radius_in_param2(self):
  a=items();orbit=a[-2]
  self.assertEqual(orbit['params'][1],-100)
  self.assertEqual(orbit['params'][2],0)
  x=MissionExchange(Link(),lambda:None)
  actual=x.upload(a)
  self.assertEqual(actual[-2]['params'][1],-100)
if __name__=='__main__':unittest.main()
