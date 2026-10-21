"""Bounded offline load without a window: native model, writer, log, ~20 Hz IPC and a 10 Hz status reader on a
fake PTY FC; never opens hardware. The windowless form of the former GUI load harness (run_gui_load.py); the
pass conditions are the same minus the window's own (Qt timer gaps, GUI threads).

  <HIL venv python> -B hitl/tests/run_load.py <new empty folder> [--duration 300]   (600 when 300 looks doubtful)
"""
import argparse,collections,ctypes,json,socket,sys,threading,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from host.core import Core
from host.transport import FakePTY
from host.ipc import Endpoint,request

def main():
 parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter);parser.add_argument('output',type=Path);parser.add_argument('--scene',type=Path);parser.add_argument('--duration',type=float,default=300);parser.add_argument('--peer-mode',choices=['thread','process'],default='thread');parser.add_argument('--gil-pause-at',type=float);parser.add_argument('--gil-pause-ms',type=int,default=350);args=parser.parse_args()
 if not 1<=args.duration<=1800:parser.error('duration must be 1..1800 seconds')
 out=args.output;out.mkdir(parents=True,exist_ok=False)
 sockets=[];ports=[]
 for _ in range(4):
  sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sockets.append(sock);ports.append(sock.getsockname()[1])
 for sock in sockets:sock.close()
 fc=FakePTY(ROOT.parent,args.peer_mode,out/'peer.json');core=Core(ROOT,fc,out/'logs',fake_state_dir=out/'state',route_settings={'qgc':ports[:2],'agent':ports[2:]});endpoint=Endpoint(core)
 gaps=collections.deque(maxlen=512);events=collections.deque(maxlen=512);start=None;queries=0;samples=[];errors=[];before=None;elapsed=0;injected=False;observer_stop=threading.Event();startup=time.monotonic()
 def poll():
  nonlocal queries
  while not observer_stop.is_set():
   try:request(ROOT,'status',fake_state_dir=out/'state');queries+=1
   except Exception as exc:
    if not observer_stop.is_set():errors.append(str(exc))
   observer_stop.wait(.05)
 def observer():
  previous=core.worker.now()
  while not observer_stop.wait(.005):
   now=core.worker.now()
   if now-previous>=20000000:gaps.append({'begin_ns':previous,'end_ns':now,'gap_ns':now-previous})
   previous=now
 threads=[threading.Thread(target=poll,daemon=True),threading.Thread(target=observer,daemon=True)]
 for thread in threads:thread.start()
 try:
  while True:
   time.sleep(.1)
   if start is None:
    if core.status()['ready_to_start_model']:core.start(args.scene);start=time.monotonic()
    elif time.monotonic()-startup>10:errors.append('FC startup timeout');break
    continue
   elapsed=time.monotonic()-start;s=core.status()
   samples.append({'seconds':elapsed,'native_ns':core.worker.now(),'produced_steps':s['produced_steps'],'misses':s['misses'],'metrics':s['metrics'],'peer':{k:v for k,v in fc.diagnostics().items() if k not in ['events','counts']}})
   if args.gil_pause_at is not None and elapsed>=args.gil_pause_at and not injected:
    injected=True;begin=core.worker.now()
    ctypes.PyDLL(None).usleep(args.gil_pause_ms*1000)
    events.append({'begin_ns':begin,'end_ns':core.worker.now(),'action':'injected GIL hold'})
   if int(elapsed)//30!=getattr(main,'printed',None):main.printed=int(elapsed)//30;print('load seconds',round(elapsed,2),'steps',s['produced_steps'],'misses',s['misses'],flush=True)
   if fc.diagnostics().get('error'):errors.append('Fake peer: '+fc.diagnostics()['error']);break
   if core.state=='fault' or elapsed>=args.duration:break
 except Exception as exc:errors.append(repr(exc))
 finally:
  if start is not None:elapsed=time.monotonic()-start
  before=core.status();observer_stop.set();core.close(force=True);endpoint.close()
  for thread in threads:thread.join(timeout=2)
 rows=np.loadtxt(out/'logs/timing.csv',delimiter=',',ndmin=2)
 post_close=core.status();metrics=post_close.get('metrics',{});timing={};windows=[]
 if rows.size:
  for j in range(2500,len(rows)+1,2500):
   lo=rows[j-2500];hi=rows[j-1];delta=(hi[0]-lo[0])*.004;windows.append(delta/(delta+(hi[2]-lo[2])/1e9))
  timing={'samples':len(rows),'step_us':{str(q):float(np.percentile(rows[:,1],q)/1000) for q in [50,95,99,100]},'lateness_us':{str(q):float(np.percentile(rows[:,2],q)/1000) for q in [50,95,99,100]},'misses':int((rows[:,2]>=4000000).sum()),'max_lag_ms':float(max(rows[:,3])/1e6),'end_lag_ms':float(rows[-1,3]/1e6),'rtf_10s':windows}
 peer=fc.final_diagnostics or fc.diagnostics()
 passed=bool(before and before['model']=='running' and elapsed>=args.duration and not errors and rows.size and timing['step_us']['99']<2000 and timing['step_us']['100']<4000 and timing['misses']==0 and all(.99<=v<=1.01 for v in windows) and not any(metrics.get(k,0) for k in ['serial_errors','native_log_drops','log_drops']) and not before['diagnostics']['io_event_drops'] and not peer.get('error') and not peer.get('parser_errors') and not core.io.is_alive() and not core.logger.is_alive())
 summary={'status':'PASS' if passed else 'FAIL','full_workload_qualified':passed and args.duration>=600 and args.gil_pause_at is None,'seconds':elapsed,'target_seconds':args.duration,'peer_mode':args.peer_mode,'injected_gil_hold_ms':args.gil_pause_ms if injected else None,'management_queries':queries,'errors':errors,'final':before,'post_close':post_close,'peer':peer,'timing':timing,'events':list(events),'python_thread_gaps':list(gaps)}
 (out/'load_summary.json').write_text(json.dumps(summary,indent=2));(out/'timing_summary.json').write_text(json.dumps(timing,indent=2));(out/'load_samples.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in samples));print(json.dumps({k:summary[k] for k in ['status','full_workload_qualified','seconds','errors']},indent=2),flush=True)
 return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
