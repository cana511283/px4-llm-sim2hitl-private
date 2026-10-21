"""Native AUTO mission lifecycle used by carrier_test --circuit.
All mission writes are gated unarmed through the sole owner; parameter changes
use the existing transactional param-deploy, with a separate exact-value restore
recipe derived from the pre-test readback. No firmware writes.
"""
import json,math,shutil,sys,time
from pathlib import Path
from autotakeoff.autotakeoff import mav,TARGET
from mission_protocol import MissionExchange,save
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'missions'))
from carrier_circuit import items,PARAMETERS,export

class NativeCircuit:
 def __init__(self,args,call,link,before):
  self.args=args;self.call=call;self.link=link;self.before=before
  self.exchange=MissionExchange(link,self.gate)
  self.backup=None;self.params_attempted=False;self.mission_attempted=False
  self.mode_sent=False;self.mode_at=None;self.last_seq=None;self.landed=False;self.finished=False
  self.sequence_at=-math.inf;self.landed_at=-math.inf
  self.result={'scope':'native five-leg circuit and orbit landing; performance not graded'}
 def gate(self):
  s=self.call('status')
  if (s['fc']['identity']!=list(TARGET) or s['fc']['armed'] is not False or s['fc']['SYS_HITL']!=1
      or s['model']=='running' or s['recovery_required'] or s['param_deploy_pending']
      or s['fc'].get('heartbeat_age_s') is None or s['fc']['heartbeat_age_s']>3):
   raise RuntimeError('Mission write requires stopped, verified unarmed HITL owner after recovery')
 def recipe(self,name,values):
  dest=self.args.out/name;dest.mkdir()
  src=self.args.release
  # Keep the same firmware/parameter definitions, not a new firmware build.
  # param-deploy validates firmware.json against the actual .px4 even though
  # this operation never flashes it. Both trial and restore recipes need it.
  shutil.copytree(src/'firmware',dest/'firmware')
  p=json.loads((src/'params.json').read_text())
  for k,v in values.items():p['set'][k]={'value':v,'why':'Temporary carrier AUTO test; restore from exact pre-test readback'}
  (dest/'params.json').write_text(json.dumps(p,indent=2)+'\n')
  return dest
 def prepare(self):
  self.gate();self.backup=self.exchange.download();save(self.args.out/'mission-before.json',self.backup)
  before=json.loads(Path(self.before['path']).read_text());values={v['name']:v['value'] for v in before['parameters']}
  self.restore_recipe=self.recipe('restore-recipe',{k:values[k] for k in PARAMETERS})
  trial=self.recipe('trial-recipe',PARAMETERS);export(self.args.out/'mission-plan')
  self.params_attempted=True
  r=self.call('param-deploy',{'release':str(trial.resolve()),'out':str((self.args.out/'trial-parameters').resolve())})
  if r.get('verdict')!='PASS':raise RuntimeError('Trial parameter deployment did not pass')
  self.mission_attempted=True
  self.exchange.upload(items());self.result['mission_upload']='PASS';self.result['parameters_trial']=r
 def update(self,msg):
  if (msg.get_srcSystem(),msg.get_srcComponent())!=TARGET:return
  kind=msg.get_type()
  if kind=='MISSION_CURRENT':
   self.last_seq=msg.seq
   self.sequence_at=time.monotonic()
   if getattr(msg,'mission_state',None)==mav.MISSION_STATE_COMPLETE:self.finished=True
  if kind=='EXTENDED_SYS_STATE':
   self.landed=msg.landed_state==mav.MAV_LANDED_STATE_ON_GROUND
   self.landed_at=time.monotonic()
 def flight_step(self,v,now,released_at,status):
  if now-released_at<40:return False
  if not self.mode_sent:
   self.link.send(mav.MAVLink_command_long_message(*TARGET,mav.MAV_CMD_DO_SET_MODE,0,1,4,4,0,0,0,0))
   self.mode_sent=True;self.mode_at=now;print('Requesting native AUTO_MISSION after 40 s stabilization',flush=True)
  if v.mode!=4 or v.sub_mode!=4:
   if now-self.mode_at>4:raise RuntimeError('Native AUTO_MISSION not confirmed, or unexpectedly exited')
   return False
  if now-self.sequence_at>3 or now-self.landed_at>3:
   raise RuntimeError('Mission or landed-state telemetry stale')
  if self.landed and math.isfinite(v.height) and v.height<2 and status.get('telemetry',{}).get('VFR_HUD',{}).get('values',{}).get('groundspeed_m_s',math.inf)<1:
   if self.last_seq!=len(items())-1:raise RuntimeError('Landed before final mission item')
   self.result.update(landing_observed=True,final_seq=self.last_seq,mission_finished_telemetry=self.finished,
    landing_status=status);return True
  if now-self.mode_at>900:raise RuntimeError('Native mission bounded flight timeout')
  return False
 def restore(self):
  errors=[]
  if self.mission_attempted:
   try:
    self.exchange.upload(self.backup);save(self.args.out/'mission-restored.json',self.exchange.download())
    self.result['mission_restore']='PASS'
   except Exception as e:errors.append('mission: '+str(e));self.result['mission_restore']='FAILED'
  if self.params_attempted:
   try:
    r=self.call('param-deploy',{'release':str(self.restore_recipe.resolve()),'out':str((self.args.out/'restored-parameters').resolve())})
    if r.get('verdict')!='PASS':raise RuntimeError('Parameter restore did not pass')
    self.result['parameter_restore']='PASS'
   except Exception as e:errors.append('parameters: '+str(e));self.result['parameter_restore']='FAILED'
  if errors:raise RuntimeError('; '.join(errors))
