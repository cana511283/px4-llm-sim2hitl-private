#!/usr/bin/env python3
"""Bounded carrier release through the sole owner (no firmware writes).
Requires explicit approval for the first real run. Ground start, genuine normal arming,
continuous carrier sensors, release at configured time, then fixed Stabilized inputs.
This records observations, not a flight-stability PASS. --circuit adds native AUTO
mission/landing with backed-up mission and transactional temporary parameters.
"""
import argparse
import json
import math
import hashlib
import sys
import signal
import time
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parent/'plant/core'))
from host.ipc import request
from autotakeoff.autotakeoff import Link,Vehicle,mav,TARGET,STABILIZED,TAKEOVER_STICK
from host.scenario import load_scene
from host import runtime_identity


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--scenario',type=Path,required=True);p.add_argument('--release',type=Path,required=True)
 p.add_argument('--mission-profile',choices=['carrier_circuit','carrier_orbit_landing'],default='carrier_circuit');p.add_argument('--out',type=Path,required=True);p.add_argument('--circuit',action='store_true');a=p.parse_args()
 if a.out.exists():raise ValueError('Use a new output directory; previous evidence is preserved')
 scene,_=load_scene(a.scenario)
 if 'carrier' not in scene:raise ValueError('Expected an explicit carrier scenario')
 if a.mission_profile=='carrier_orbit_landing' and (not a.circuit or scene.get('flat_rangefinder',False)):raise ValueError('Orbit no-range profile requires --circuit and disabled rangefinder')
 c=scene['carrier'];release_s=c['wait_s']+c['climb_s']+c['cruise_s']
 if not 1<=c['wait_s']<=30 or not 10<=c['climb_s']<=120 or not 5<=c['cruise_s']<=30:
  raise ValueError('First carrier test requires wait 1..30 s, climb 10..120 s, cruise 5..30 s')
 a.out.mkdir(parents=True)
 result={'scope':'carrier release, no mission or landing; performance not graded','process':'RUNNING','recovery':'NOT_RUN'}
 def sampled_status(s):
  if not a.circuit:return s
  # Keep all safety/telemetry evidence. Full parameters are saved separately by
  # readback; repeating 1979 values at 50 Hz would add gigabytes of disk I/O.
  v=dict(s);v['parameters']={'SYS_HITL':s.get('parameters',{}).get('SYS_HITL'),
                            'count':len(s.get('parameters',{}))}
  return v
 def save(): (a.out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
 def call(action,args=None):
  r=request(HERE,action,args,timeout_s=1800 if action=='fetch-log' else 600 if action=='param-readback' else 90 if action=='recover' else 45)
  recorded=r
  if a.circuit and action=='status' and r['status']=='complete':
   recorded={'status':'complete','result':sampled_status(r.get('result',r.get('operation',{}).get('result')))}
  with (a.out/'operations.jsonl').open('a') as f:f.write(json.dumps({'at':time.time(),'action':action,'response':recorded})+'\n')
  if r['status']!='complete':raise RuntimeError(str(r))
  return r.get('result',r.get('operation',{}).get('result'))
 def readback(name):
  b=call('param-readback',{'release':str(a.release.resolve()),'out':str((a.out/name).resolve())})
  from importlib.util import spec_from_file_location,module_from_spec
  spec=spec_from_file_location('check_params',HERE.parent/'deploy/check_params.py');mod=module_from_spec(spec);spec.loader.exec_module(mod)
  report=mod.check(a.release,b['path']);(a.out/(name+'-check.json')).write_text(json.dumps(report,indent=2))
  if report['verdict']!='PASS':raise RuntimeError(name+' parameter/identity check failed')
  return b
 link=None;started=False;saved_error=None;native=None
 try:
  start=call('status');result['before']=start
  if start['fake_fc'] or start['unarmed_only'] or not start['ready_to_start_model'] or start['model']=='running' or start['pilot']['running']:
   raise RuntimeError('Requires idle real verified, unarmed, SYS_HITL=1 owner, no pending recovery/deploy or pilot')
  inventory=runtime_identity.source_id(runtime_identity.inventory(HERE.parent))
  if inventory!=start['runtime_identity']['source_id']:raise RuntimeError('Owner source changed; restart the owner')
  if start['fc']['armed'] is not False:raise RuntimeError('FC must be unarmed')
  if start['fc']['identity']!=list(TARGET):raise RuntimeError('Carrier pilot target does not match verified FC')
  b=readback('params-before');result['params_before']=b
  vals={v['name']:v['value'] for v in json.loads(Path(b['path']).read_text())['parameters']}
  if vals.get('SYS_HITL')!=1:raise RuntimeError('HITL must be on')
  # Derive release commands from this machine's plant and verified FC scale.
  import sys as _sys
  _sys.path.insert(0,str(HERE.parent/'sitl'));import trim
  tr=trim.trim(trim.Core(trim.default_plant_lib()),c['speed_m_s'],c['height_m'])
  scale=vals['FW_MAN_P_MAX'];offset=vals['FW_PSP_OFF']
  if scale<=0:raise RuntimeError('Invalid pitch scale')
  pitch=round(-(tr['pitch_deg']-offset)/scale*1000);throttle=round(tr['throttle']*1000)
  if not -1000<=pitch<=1000 or not 0<=throttle<=1000:raise RuntimeError('Trim commands outside bounds')
  result['trim_commands']={'pitch_x':pitch,'throttle_z':throttle}
  listing=call('fetch-log',{'list':True});result['logs_before']=listing
  if not listing.get('complete'):raise RuntimeError('Incomplete initial log list')
  link=Link(14560,14576,open(a.out/'agent-wire.bin','wb'))
  if a.circuit:
   from native_circuit import NativeCircuit
   native=NativeCircuit(a,call,link,b);native.prepare()
   result['scope']=native.result['scope']
  # From here onward a failure always stops the model and runs standard recovery.
  started=True;call('model-start',{'scene':scene})
  v=Vehicle();phase='telemetry';phase_at=time.monotonic();begin=phase_at;hb_at=0;sent_mode=False;arm_sent=False;released_at=None
  with (a.out/'samples.jsonl').open('w') as samples:
   while True:
    now=time.monotonic()
    for m in link.receive():
     v.update(m,now)
     if native:native.update(m)
     if m.get_type()=='COMMAND_ACK' and m.get_srcSystem()==TARGET[0] and m.command in [mav.MAV_CMD_COMPONENT_ARM_DISARM,mav.MAV_CMD_DO_SET_MODE] and m.result not in [0,5]:
      raise RuntimeError(f'PX4 rejected command {m.command}: {m.result}')
    if now>=hb_at:
     link.send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS,mav.MAV_AUTOPILOT_INVALID,0,0,0,3));hb_at=now+1
    x,z=(pitch,throttle) if phase=='released' else (0,0)
    if native and native.mode_sent and v.mode==4 and v.sub_mode==4:
     x=0  # neutral pitch/roll/yaw keepalive; AUTO owns attitude and throttle
    link.send(mav.MAVLink_manual_control_message(TARGET[0],x,0,z,0,0))
    if v.pilot and now-v.seen.get('pilot',-math.inf)<.5 and max(abs(n) for n in v.pilot)>TAKEOVER_STICK:
     raise RuntimeError('QGC pilot input: stop the automatic test')
    s=call('status');samples.write(json.dumps({'elapsed_s':now-begin,'status':sampled_status(s)})+'\n');samples.flush()
    if s['last_error'] or s['model']!='running' or s['recovery']=='failed':raise RuntimeError('Owner model error: '+str(s['last_error']))
    if s['scheduler']['time_constraint_result']!=0 and now-begin>2:raise RuntimeError('Real-time scheduling not active')
    if s['misses'] or s['metrics'].get('serial_errors',0):raise RuntimeError('Timing miss or serial error; preserve evidence')
    if s['fc']['identity']!=start['fc']['identity'] or s['fc']['SYS_HITL']!=1:raise RuntimeError('FC identity/HITL changed')
    if s['fc'].get('heartbeat_age_s') is None or s['fc']['heartbeat_age_s']>3:raise RuntimeError('FC heartbeat stale')
    if v.fresh(now):
     if phase=='telemetry' and s['scheduler']['time_constraint_result']==0:phase='mode';phase_at=now
    elif phase!='telemetry':raise RuntimeError('Telemetry stale/nonfinite')
    if phase=='mode':
     if not sent_mode:
      link.send(mav.MAVLink_command_long_message(*TARGET,mav.MAV_CMD_DO_SET_MODE,0,1,STABILIZED,0,0,0,0,0));sent_mode=True
     if v.mode==STABILIZED and now-phase_at>=1:
      if v.vehicle_type!=mav.MAV_TYPE_FIXED_WING:raise RuntimeError('Expected fixed-wing FC')
      link.send(mav.MAVLink_command_long_message(*TARGET,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0));arm_sent=True;phase='arming';phase_at=now
    elif phase=='arming' and v.armed:
     phase='carried';phase_at=now
    elif phase in ['carried','released']:
     if not v.armed:raise RuntimeError('Unexpected disarming')
     if not (native and native.mode_sent) and v.mode!=STABILIZED:raise RuntimeError('Unexpected mode transition')
     cp=(s.get('model_truth') or {}).get('carrier_phase')
     if cp==3 and released_at is None:
      phase='released';released_at=now;result['release_observed']=s
      print(f'Carrier released; AUTO delay {native.auto_delay} s' if native else 'Carrier released; observing free-flight stabilization for 40 s',flush=True)
     if native and released_at is not None:
      if native.flight_step(v,now,released_at,s):break
     elif released_at is not None and now-released_at>=40:break
     if released_at is None and now-phase_at>release_s+5:raise RuntimeError('Release timeout')
    if phase in ['telemetry','mode','arming'] and now-phase_at>20:raise RuntimeError(phase+' timeout; no retry')
    if now-begin>release_s+(1000 if native else 90):raise RuntimeError('Bounded test timeout')
    time.sleep(.02)
  result['process']='COMPLETE';result['performance']='NOT_GRADED'
 except BaseException as exc:
  saved_error=exc;result['process']='FAILED';result['error']=str(exc)
 finally:
  if started:
   try:
    result['after_model_stop']=call('model-stop')
    call('recover',{'hil_force_disarm':True})
    end=call('status');result['after_recovery']=end
    if end['fc']['armed'] is not False or end['fc']['SYS_HITL']!=1 or end['recovery_required'] or end['param_deploy_pending']:
     raise RuntimeError('Recovery final state not verified')
    result['recovery']='PASS'
   except Exception as exc:result['recovery']='FAILED';result['recovery_error']=str(exc)
  if native:
   try:
    if started and result['recovery']!='PASS':raise RuntimeError('Recovery incomplete; no mission/parameter writes attempted')
    native.restore()
    final=call('status');result['after_restoration']=final
    if (final['fc']['armed'] is not False or final['fc']['SYS_HITL']!=1
        or final['recovery_required'] or final['param_deploy_pending']):
     raise RuntimeError('Final recovery/restore state not verified')
   except Exception as exc:result['restore_error']=str(exc);saved_error=saved_error or exc
   result['native_circuit']=native.result
  if link:link.close()
  save()
 if started and result['recovery']=='PASS':
  try:
   end=result['after_recovery'];begin=result['before']
   if end.get('instance_id')!=begin.get('instance_id') or end.get('build_id')!=begin.get('build_id'):
    raise RuntimeError('Owner/model identity changed across test')
   if runtime_identity.source_id(runtime_identity.inventory(HERE.parent))!=begin['runtime_identity']['source_id']:
    raise RuntimeError('Runtime source changed during test')
   result['params_after']=readback('params-after')
   before=json.loads(Path(result['params_before']['path']).read_text())
   after=json.loads(Path(result['params_after']['path']).read_text())
   if before['autopilot_version']!=after['autopilot_version']:raise RuntimeError('FC identity changed across test')
   old={v['name']:v['bits'] for v in before['parameters']};new={v['name']:v['bits'] for v in after['parameters']}
   owned=set(json.loads((a.release/'params.json').read_text())['fc_owned']['resolved_names'])
   changed=[n for n in old if old[n]!=new[n]]
   result['changed_fc_owned']=[n for n in changed if n in owned]
   result['changed_other']=[n for n in changed if n not in owned]
   if result['changed_other']:raise RuntimeError('Unexpected parameter changes: '+str(result['changed_other']))
   result['logs_after']=call('fetch-log',{'list':True})
   result['fc_log']=call('fetch-log',{'out':str((a.out/'fc-log').resolve())})
   log=result['fc_log']
   if log['id']<=result['logs_before']['newest_id']:raise RuntimeError('Downloaded log is not new for this run')
   again=call('fetch-log',{'out':str((a.out/'fc-log-again').resolve()),'log_id':log['id']})
   result['fc_log_again']=again
   if hashlib.sha256(Path(log['path']).read_bytes()).hexdigest()!=log['sha256'] or again['sha256']!=log['sha256']:
    raise RuntimeError('FC log downloads differ')
   sys.path.insert(0,str(HERE.parent/'analysis'));import ulog_strict
   result['log_parse']=ulog_strict.parse(log['path'])
   if not result['log_parse']['ok']:raise RuntimeError('Strict ULog parse failed')
   result['log_attribution']='PASS'
  except Exception as exc:result['evidence_error']=str(exc)
  save()
 print(json.dumps(result,indent=2))
 return 1 if saved_error or result['recovery']!='PASS' or result.get('evidence_error') else 0

def interrupted(signum, _frame):
 raise KeyboardInterrupt('Signal '+str(signum)+'; stop and recover')

if __name__=='__main__':
 signal.signal(signal.SIGTERM,interrupted)
 if hasattr(signal,'SIGHUP'):signal.signal(signal.SIGHUP,interrupted)
 raise SystemExit(main())
