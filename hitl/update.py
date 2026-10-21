"""Model release: generate (stage 1, MATLAB) and build/validate/publish (stage 2, no MATLAB session needed).

  python -B hitl/update.py --generate [--cache DIR]   stage 1: model -> model/generated (kept in git)
  python -B hitl/update.py [--force]                   stage 2: model/generated -> release + active pointer
  python -B hitl/update.py --rollback | --fc-only

Stage 1 runs plant/core/build_core.m (through the MATLAB MCP server, or pass --cache with the folder that
build_core() returned when it was run in MATLAB directly) and stores the generated C/h, its makefile, the
Simulink reference runs and the identity of the model sources in model/generated. Stage 2 refuses when the
model sources changed after stage 1. Compiler, MATLAB headers and library names come from
plant/core/toolchain.py (found on this machine, overridable by environment variables).
"""
import argparse,datetime,hashlib,json,os,re,shutil,subprocess,sys,uuid
from pathlib import Path
from host.release import platform_id
RUNTIME=Path(__file__).resolve().parent
ROOT=RUNTIME.parent
PLANT=ROOT/'plant'
MODEL=ROOT/'model'
GENERATED=MODEL/'generated'
sys.path.insert(0,str(PLANT/'core'))
import toolchain
FLAGS=['-std=c11','-O2','-fno-fast-math','-ffp-contract=off','-fvisibility=hidden']+toolchain.shared_flags()
# Plant gate reports sealed into every release next to the library validation (validation.json).
GATES=['reuse_policy.json','core_validation.json','plant_validation.json','carrier_validation.json','carrier_abi_validation.json','validation.json']
EVIDENCE=['build.json','core_build.json','core_compile.json']
CODEGEN='codegen/SkySurfer_Core_ert_rtw'
RESULTS=['reuse_policy.json','core_build.json','wind_wind.csv']+[f'{k}_{c}.csv' for k in ['source','core','controls'] for c in ['ground','air','wind']]
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def tree(folder,prefix):
 return {prefix+str(p.relative_to(folder)):digest(p) for p in sorted(Path(folder).rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
def model_sources():
 # What stage 1 generates from: the model folder (without its generated output), the reuse-library policy
 # and the MATLAB builder itself.
 records={'model/'+str(p.relative_to(MODEL)):digest(p) for p in sorted(MODEL.rglob('*')) if p.is_file() and not p.relative_to(MODEL).parts[0].startswith(('generated','.generated'))}
 for f in ['.satk/reuse-libraries.json','plant/core/build_core.m','plant/core/reuse_policy_gate.m']:records[f]=digest(ROOT/f)
 return records
def generated_manifest():
 path=GENERATED/'manifest.json'
 if not path.is_file():raise RuntimeError('model/generated is missing: run update.py --generate first')
 manifest=json.loads(path.read_text())
 for rel,sha in manifest['files'].items():
  if digest(GENERATED/rel)!=sha:raise RuntimeError('model/generated/'+rel+' differs from its manifest: run update.py --generate')
 if manifest['model_sources']!=model_sources():raise RuntimeError('The model sources changed after the code was generated: run update.py --generate')
 return manifest
def export_generated(cache):
 # Stage 1 output, only from a cache built from the current model (build_core copies the source slx unchanged).
 cache=Path(cache);gen=cache/CODEGEN;sources=model_sources()
 if digest(cache/'models/SkySurfer_CoreSource.slx')!=digest(MODEL/'FW_SkySurfer_HIL.slx'):raise RuntimeError('This cache was not generated from the current model/FW_SkySurfer_HIL.slx')
 gate=json.loads((cache/'results/reuse_policy.json').read_text())
 if gate.get('status')!='PASS':raise RuntimeError('Reuse-library gate did not pass: '+json.dumps(gate))
 # One generated C for every host: no instruction-set intrinsics (build_core.m sets InstructionSetExtensions 'None').
 isa=sorted({h for p in gen.iterdir() if p.suffix in ['.c','.h'] for h in re.findall(r'#\s*include\s*<((?:[a-z]*mmintrin|arm_neon|arm_sve)\.h)>',p.read_text(errors='replace'))})
 if isa:raise RuntimeError('Generated code includes instruction-set headers ('+', '.join(isa)+'): set InstructionSetExtensions to None in plant/core/build_core.m, the same C must compile on x86-64 and arm64')
 staging=MODEL/('.generated-'+str(uuid.uuid4()));(staging/CODEGEN).mkdir(parents=True);(staging/'results').mkdir()
 for p in sorted(gen.iterdir()):
  if p.suffix in ['.c','.h','.mk']:shutil.copy2(p,staging/CODEGEN/p.name)
 for name in RESULTS:shutil.copy2(cache/'results'/name,staging/'results'/name)
 build=json.loads((cache/'results/core_build.json').read_text())
 manifest={'schema_version':1,'created_utc':datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%SZ'),'matlab':build['matlab'],
           'model_sources':sources,'files':tree(staging,'')}
 (staging/'manifest.json').write_text(json.dumps(manifest,indent=2))
 old=MODEL/('.generated-old-'+str(uuid.uuid4()))
 if GENERATED.exists():GENERATED.rename(old)
 staging.rename(GENERATED)
 if old.exists():shutil.rmtree(old)
 print(json.dumps({'status':'PASS','generated':str(GENERATED),'files':len(manifest['files']),'matlab':manifest['matlab']}))
def build_cache():
 # A fresh build cache laid out from model/generated: the compilers and validators read <cache>/codegen and <cache>/results.
 cache=toolchain.cache_root()/('build_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
 shutil.copytree(GENERATED/'codegen',cache/'codegen');shutil.copytree(GENERATED/'results',cache/'results');(cache/'native').mkdir()
 return cache
def fingerprint():
 records=model_sources();records.update(tree(GENERATED,'model/generated/'))
 # Only the library validator belongs to the build; other tests (host, fake FC) do not change the model.
 for folder in ['interface','native','aircraft/skysurfer']:records.update(tree(RUNTIME/folder,'hitl/'+folder+'/'))
 for f in ['poc/source_columns.json','tests/validate_library.py','tests/validate_carrier_abi.py','update.py','host/abi.py','host/scenario.py','host/packets.py']:records['hitl/'+f]=digest(RUNTIME/f)
 # Plant core wrapper, C Truth, C sensors and their validators (plant/reference holds recorded data, not build inputs).
 for folder in ['include','core','truth','sensors']:records.update(tree(PLANT/folder,'plant/'+folder+'/'))
 records['compiler']=toolchain.compiler_version()
 records['architecture']=platform_id();records['compile_flags']=FLAGS
 return records
def reference_identity():
 data=PLANT/'reference/data';return {p.name:digest(p) for p in sorted(data.iterdir()) if p.is_file()}
def external_headers(cmd,gen,matlab):
 # Every header the library includes from outside the project (MATLAB rtw/simulink headers), found by the
 # compiler itself; keys are relative so the build-id does not depend on where MATLAB is installed.
 deps=subprocess.check_output(cmd+['-MM'],text=True).replace('\\\n',' ').replace('\\ ','\0').split()
 headers={}
 for token in deps:
  if token.endswith(':') or not token.endswith('.h'):continue
  p=Path(token.replace('\0',' ')).resolve()
  if matlab and matlab.resolve() in p.parents:headers['MATLAB/'+str(p.relative_to(matlab.resolve()))]=digest(p)
  elif gen.resolve() not in p.parents and ROOT.resolve() not in p.parents:headers[str(p)]=digest(p)
 return dict(sorted(headers.items()))
def compile_library(cache,fp):
 gen=cache/CODEGEN;out=cache/'native';rtw,matlab=toolchain.rtw_include_dirs()
 make=(gen/'SkySurfer_Core.mk').read_text();defines=[]
 for key in ['DEFINES_BUILD_ARGS','DEFINES_OPTS','DEFINES_STANDARD']:
  match=re.search(r'^'+key+r'\s*=\s*(.*)$',make,re.M);defines+=match.group(1).split() if match else []
 if '-DMULTI_INSTANCE_CODE=1' not in defines:raise RuntimeError('Generated core is not reusable')
 sources=[gen/x for x in ['SkySurfer_Core.c','SkySurfer_Core_data.c','rtGetInf.c','rtGetNaN.c','rt_nonfinite.c']]
 sources+=[PLANT/'core/plant_core.c',PLANT/'truth/plant_truth.c',PLANT/'sensors/plant_sensors.c',PLANT/'sensors/plant_wmm.c',PLANT/'sensors/plant_wmm_coeffs.c',RUNTIME/'aircraft/skysurfer/adapter.c']
 generated={str(p.relative_to(gen)):digest(p) for p in gen.glob('*') if p.suffix in ['.c','.h']}
 includes=[]
 for inc in [RUNTIME/'interface',PLANT/'include',PLANT/'sensors',gen]+rtw:includes+=['-I',str(inc)]
 base=toolchain.cc()+FLAGS+includes+defines+toolchain.model_defines(cache)+['-DHIL_SOURCE_HASH="'+fp['model/FW_SkySurfer_HIL.slx']+'"','-DHIL_PLATFORM="'+platform_id()+'"']
 # Without MATLAB the two run-time headers are the project's own (plant/core/rtw, already in the fingerprint).
 toolchain_id={'makefile_sha256':digest(gen/'SkySurfer_Core.mk'),'defines':defines,'model_defines':toolchain.model_defines(cache),'external_headers':external_headers([x for x in base if x not in toolchain.shared_flags()]+[str(p) for p in sources],gen,matlab),'rtw_headers':'MATLAB' if matlab else 'plant/core/rtw'}
 identity={'fingerprint':fp,'generated':generated,'toolchain':toolchain_id}
 build_id='skysurfer-'+hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:24]
 cmd=base+['-DHIL_BUILD_ID="'+build_id+'"']+[str(p) for p in sources]+['-o',str(out/toolchain.lib('skysurfer'))];subprocess.run(cmd,check=True)
 compile_worker(out)
 (cache/'results/build.json').write_text(json.dumps({'build_id':build_id,'fingerprint':fp,'generated':generated,'toolchain':toolchain_id,'command':cmd},indent=2));return build_id
WORKER_SOURCES=['native/posix/worker.c','native/posix/io.c']  # POSIX; clock and scheduling per platform inside
def worker_command(out):
 sources=[str(RUNTIME/x) for x in WORKER_SOURCES];inc=['-I',str(RUNTIME/'interface')]
 return toolchain.cc()+['-std=c11','-O2','-D_GNU_SOURCE','-fvisibility=hidden','-pthread']+toolchain.shared_flags()+inc+sources+['-lm','-o',str(out/toolchain.lib('hil_worker'))]
def compile_worker(out):
 cmd=worker_command(out);subprocess.run(cmd,check=True);return Path(cmd[-1]),cmd
def validate_worker(path):
 # FC-only gate: the exports the host needs, a monotonic clock, and the native serial writer on a pipe
 # (bytes delivered in order; a closed descriptor is recorded as EBADF). No model, no MATLAB.
 import errno,time
 sys.path.insert(0,str(RUNTIME));from host.abi import Worker,diagnostics,io_stats
 worker=Worker(path,None);lib=worker.lib;checks={'exports':'PASS'}
 t=[worker.now() for _ in range(1000)];checks['clock_monotonic']='PASS' if all(b>=a for a,b in zip(t,t[1:])) and t[-1]>t[0] else 'FAIL'
 rd,wr=os.pipe();os.set_blocking(rd,False);data=bytes(range(256))*4;got=b''
 try:
  lib.hil_io_start(wr);codes=[lib.hil_io_command(data[k:k+256],256) for k in range(0,len(data),256)];end=time.monotonic()+2
  while len(got)<len(data) and time.monotonic()<end:
   try:got+=os.read(rd,4096)
   except BlockingIOError:time.sleep(.001)
 finally:lib.hil_io_stop();os.close(rd);os.close(wr)
 checks['io_command_order']='PASS' if codes==[0]*4 and got==data else 'FAIL'
 rd,wr=os.pipe();os.close(wr)
 try:
  lib.hil_io_start(wr);lib.hil_io_command(b'bad',3);end=time.monotonic()+1
  while not io_stats(worker)['errors'] and time.monotonic()<end:time.sleep(.001)
  errors=io_stats(worker)['errors'];event=diagnostics(worker)['io_last_event']
 finally:lib.hil_io_stop();os.close(rd)
 checks['io_fatal_errno']='PASS' if errors==1 and event['write_errno']==errno.EBADF else 'FAIL'
 return {'status':'PASS' if all(v=='PASS' for v in checks.values()) else 'FAIL','checks':checks}
def fc_only(cache=None):
 # A release without a model, for a machine that cannot compile the model: the owner can talk to the FC
 # (status, readback, logs, SD card); actions that need the model refuse.
 pid=platform_id();cache=Path(cache or toolchain.cache_root()/'fc_only_build');(cache/'native').mkdir(parents=True,exist_ok=True)
 path,cmd=compile_worker(cache/'native')
 compiler=toolchain.compiler_version()
 identity={'platform':pid,'sources':{x:digest(RUNTIME/x) for x in WORKER_SOURCES+['interface/hil_model_api.h','host/abi.py','update.py']},'command':cmd[:-1],'compiler':compiler}
 build_id='fc-only-'+hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()[:24]
 result=validate_worker(path);(cache/'worker_validation.json').write_text(json.dumps(dict(result,build_id=build_id),indent=2))
 if result['status']!='PASS':raise RuntimeError('Worker validation failed '+json.dumps(result['checks'])+'; active unchanged')
 (cache/'build.json').write_text(json.dumps(dict(identity,build_id=build_id),indent=2))
 parent=RUNTIME/'releases/skysurfer'/pid;parent.mkdir(parents=True,exist_ok=True);staging=parent/('.staging-'+str(uuid.uuid4()));staging.mkdir()
 shutil.copy2(path,staging/path.name);shutil.copy2(RUNTIME/'aircraft/skysurfer/profile.json',staging/'profile.json')
 for name in ['worker_validation.json','build.json']:shutil.copy2(cache/name,staging/name)
 manifest={'schema_version':1,'platform':pid,'build_id':build_id,'abi_version':65537,'library':None,'worker':path.name,'fc_only':True,'gates':{'worker_validation.json':'PASS'},'required_status':'PASS','files':{p.name:digest(p) for p in staging.iterdir()}}
 (staging/'manifest.json').write_text(json.dumps(manifest,indent=2));return install(staging,parent,build_id,pid)
def install(staging,parent,build_id,pid,report=None):
 final=parent/build_id
 if final.exists():raise RuntimeError('Immutable release already exists; active unchanged')
 staging.rename(final);pointer={'schema_version':1,'build_id':build_id,'release':str(final.relative_to(RUNTIME)),'manifest_sha256':digest(final/'manifest.json')}
 active=RUNTIME/'active'/f'{pid}.json';previous=json.loads(active.read_text()) if active.exists() else None
 temporary=active.with_suffix('.tmp');temporary.write_text(json.dumps(pointer,indent=2));os.replace(temporary,active)
 if previous:(RUNTIME/'active'/f'{pid}.previous.json').write_text(json.dumps(previous,indent=2))
 print(json.dumps(dict({'status':'PASS','release':str(final),'build_id':build_id},**(report or {}))));return final
def validate(cache):
 # Plant gates first (core against the reference data, Truth/sensors), then the HIL library itself.
 subprocess.run([sys.executable,'-B',str(PLANT/'core/compile_core.py'),'--cache',str(cache)],check=True)
 for script in ['validate_core.py','validate_plant.py']:subprocess.run([sys.executable,'-B',str(PLANT/'core'/script),'--cache',str(cache)],check=True)
 subprocess.run([sys.executable,'-B',str(PLANT/'core/validate_carrier.py'),'--plant-lib',str(cache/'native'/toolchain.lib('skysurfer_plant')),'--out',str(cache/'results/carrier_validation.json')],check=True)
 subprocess.run([sys.executable,'-B',str(RUNTIME/'tests/validate_carrier_abi.py'),'--library',str(cache/'native'/toolchain.lib('skysurfer')),'--out',str(cache/'results/carrier_abi_validation.json')],check=True)
 subprocess.run([sys.executable,'-B',str(RUNTIME/'tests/validate_library.py'),'--cache',str(cache)],check=True)
def publish(cache,build_id,fp,reference,generated):
 # Every gate must have passed, not only the library validation; their reports are sealed in the release.
 gates={}
 for name in GATES:
  path=cache/'results'/name;gates[name]=json.loads(path.read_text()).get('status') if path.is_file() else 'MISSING'
 if any(v!='PASS' for v in gates.values()):raise RuntimeError('Required validation failed or missing '+json.dumps(gates)+'; active unchanged')
 pid=platform_id();parent=RUNTIME/'releases/skysurfer'/pid;parent.mkdir(parents=True,exist_ok=True);staging=parent/('.staging-'+str(uuid.uuid4()));staging.mkdir()
 names={'library':toolchain.lib('skysurfer'),'worker':toolchain.lib('hil_worker'),'plant_library':toolchain.lib('skysurfer_plant')}
 for f in names.values():shutil.copy2(cache/'native'/f,staging/f)
 shutil.copy2(RUNTIME/'aircraft/skysurfer/profile.json',staging/'profile.json')
 for name in GATES+EVIDENCE:shutil.copy2(cache/'results'/name,staging/name)
 manifest=dict({'schema_version':1,'platform':pid,'build_id':build_id,'abi_version':65537},**names,fingerprint=fp,gates=gates,reference=reference,
               generated={'manifest_sha256':digest(GENERATED/'manifest.json'),'matlab':generated['matlab']},required_status='PASS',files={p.name:digest(p) for p in staging.iterdir()})
 (staging/'manifest.json').write_text(json.dumps(manifest,indent=2));return install(staging,parent,build_id,pid,{'cache':str(cache)})
def main():
 p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
 p.add_argument('--generate',action='store_true',help='stage 1: run build_core.m (or take --cache) and store the result in model/generated')
 p.add_argument('--cache',type=Path,help='with --generate: the folder build_core() returned (MATLAB run without the MCP server)')
 p.add_argument('--rollback',action='store_true');p.add_argument('--force',action='store_true')
 p.add_argument('--fc-only',action='store_true',help='publish a release without a model (worker only)');args=p.parse_args()
 (RUNTIME/'active').mkdir(exist_ok=True);lock=RUNTIME/'active/update.lock'
 try:fd=os.open(str(lock),os.O_CREAT|os.O_EXCL|os.O_WRONLY)
 except FileExistsError:raise RuntimeError('Another update owns the publish lock; inspect before removing stale lock')
 try:
  os.write(fd,str(os.getpid()).encode());os.close(fd)
  pid=platform_id()
  if args.rollback:
   from host.release import read_release
   previous=RUNTIME/f'active/{pid}.previous.json';read_release(RUNTIME,previous);temp=RUNTIME/'active/rollback.tmp';temp.write_bytes(previous.read_bytes());os.replace(temp,RUNTIME/f'active/{pid}.json');print('rollback PASS');return
  if args.fc_only:fc_only();return
  if args.generate:
   if args.cache:cache=args.cache
   else:
    sys.path.insert(0,str(RUNTIME/'matlab'));from mcp_build import generate
    cache=generate(RUNTIME)
   export_generated(cache);return
  generated=generated_manifest();fp=fingerprint();active=RUNTIME/f'active/{pid}.json'
  if active.exists() and not args.force:
   from host.release import read_release
   release,manifest=read_release(RUNTIME)
   if manifest.get('fingerprint')==fp:print(json.dumps({'status':'PASS','unchanged':True,'build_id':manifest['build_id']}));return
  cache=build_cache();reference=reference_identity();build_id=compile_library(cache,fp);validate(cache)
  if fingerprint()!=fp:raise RuntimeError('Build dependencies changed during validation; active unchanged')
  if reference_identity()!=reference:raise RuntimeError('Plant reference data changed during validation; active unchanged')
  publish(cache,build_id,fp,reference,generated)
 finally:lock.unlink(missing_ok=True)
if __name__=='__main__':
 try:main()
 except Exception as exc:print(str(exc),file=sys.stderr);raise SystemExit(4)
