"""Bounded MAVLink mission exchange via the existing owner agent link only.
No direct serial access, mode changes or arming. Every mutation requires caller's
verified unarmed/HITL owner gate. No automatic retry after a timeout or rejection.
"""
import math,time,json
from pathlib import Path
from autotakeoff.autotakeoff import mav,TARGET,SOURCE

class MissionExchange:
 def __init__(self,link,gate,timeout=8):
  self.link=link;self.gate=gate;self.timeout=timeout;self.last_heartbeat=-math.inf
 def send(self,msg):self.link.send(msg)
 def wait(self,kinds):
  end=time.monotonic()+self.timeout
  while time.monotonic()<end:
   now=time.monotonic()
   if now-self.last_heartbeat>=1:
    self.send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS,mav.MAV_AUTOPILOT_INVALID,0,0,0,3));self.last_heartbeat=now
   for msg in self.link.receive():
    if (msg.get_srcSystem(),msg.get_srcComponent())!=TARGET:continue
    if msg.get_type() in kinds:
     if hasattr(msg,'mission_type') and msg.mission_type!=0:continue
     if hasattr(msg,'target_system') and msg.target_system!=SOURCE[0]:continue
     if hasattr(msg,'target_component') and msg.target_component!=SOURCE[1]:continue
     return msg
   time.sleep(.01)
  raise RuntimeError('Mission protocol timeout waiting for '+str(kinds))
 @staticmethod
 def normalize(item):
  return {k:item[k] for k in ['seq','command','frame','current','autocontinue','params']}
 @staticmethod
 def equal(left,right):
  if len(left)!=len(right):return False
  for a,b in zip(left,right):
   if any(a[k]!=b[k] for k in ['seq','command','frame','autocontinue']):return False
   for n,(x,y) in enumerate(zip(a['params'],b['params'])):
    if x is None or y is None:
     if x is not None or y is not None:return False
    elif n in (4,5) and a['frame']!=2:
     # Global coordinates are signed 1e-7-degree integers on the wire.
     # A relative float tolerance at longitude 120 deg would allow metres
     # of wrong position. Permit only one integer unit for round/truncate.
     if abs(round(x*1e7)-round(y*1e7))>1:return False
    elif not math.isclose(x,y,rel_tol=2e-6,abs_tol=2e-6):return False
  return True
 def download(self):
  self.send(mav.MAVLink_mission_request_list_message(*TARGET,0));count=self.wait({'MISSION_COUNT'}).count
  if count>1000:raise RuntimeError('Mission count exceeds bounded backup limit')
  result=[]
  for seq in range(count):
   self.send(mav.MAVLink_mission_request_int_message(*TARGET,seq,0));msg=self.wait({'MISSION_ITEM_INT'})
   if msg.seq!=seq:raise RuntimeError('Mission download sequence mismatch')
   p=[msg.param1,msg.param2,msg.param3,msg.param4,msg.x/1e7,msg.y/1e7,msg.z]
   if msg.frame==2:p[4:6]=[msg.x,msg.y]
   result.append(dict(seq=seq,command=msg.command,frame=msg.frame,current=bool(msg.current),autocontinue=bool(msg.autocontinue),params=[None if not math.isfinite(x) else x for x in p]))
  self.send(mav.MAVLink_mission_ack_message(*TARGET,mav.MAV_MISSION_ACCEPTED,0))
  return result
 def upload(self,items):
  self.gate()
  self.send(mav.MAVLink_mission_count_message(*TARGET,len(items),0))
  seen=set()
  deadline=time.monotonic()+min(180,20+len(items)*self.timeout)
  while True:
   if time.monotonic()>deadline:raise RuntimeError('Mission upload overall timeout')
   msg=self.wait({'MISSION_REQUEST_INT','MISSION_REQUEST','MISSION_ACK'})
   if msg.get_type()=='MISSION_ACK':
    if msg.type!=mav.MAV_MISSION_ACCEPTED:raise RuntimeError('PX4 mission rejected: '+str(msg.type))
    if seen!=set(range(len(items))):raise RuntimeError('Early mission ACK before all items requested')
    break
   seq=msg.seq
   if not 0<=seq<len(items):raise RuntimeError('Mission request index out of range')
   self.gate();i=items[seq];p=[float('nan') if x is None else x for x in i['params']]
   if msg.get_type()=='MISSION_REQUEST_INT':
    xy=[round(p[4]*1e7),round(p[5]*1e7)] if i['frame']!=2 else [round(p[4]),round(p[5])]
    out=mav.MAVLink_mission_item_int_message(*TARGET,seq,i['frame'],i['command'],int(i['current']),int(i['autocontinue']),*p[:4],*xy,p[6],0)
   else:
    frame={5:0,6:3,7:3,11:10}.get(i['frame'],i['frame'])
    out=mav.MAVLink_mission_item_message(*TARGET,seq,frame,i['command'],int(i['current']),int(i['autocontinue']),*p,0)
   self.send(out);seen.add(seq)
  actual=self.download()
  if not self.equal(items,actual):raise RuntimeError('Mission readback differs from upload')
  return actual

def save(path,items):Path(path).write_text(json.dumps(items,indent=2,allow_nan=False)+'\n')
