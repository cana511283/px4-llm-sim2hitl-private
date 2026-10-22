"""Shared native PX4 mission definition; local N/E metres from the simulated ground origin.
No vehicle connection or upload occurs here. The first full circuit is followed by a
separate orbit-to-alt landing approach, so the final leg of the circuit is not a landing.
"""
import json,math
from pathlib import Path
LAT=22.910275834988674
LON=120.31096929726908
SPEED=30*1852/3600
# name, command, north, east, relative altitude, MAVLink params 1..4
ROWS=[('airspeed',178,0,0,0,[0,SPEED,-1,0]),
 ('entry',16,1400,0,100,[0,35,0,None]),
 ('upwind',16,2000,0,100,[0,35,0,None]),
 ('crosswind',16,2000,-400,100,[0,35,0,None]),
 ('downwind',16,800,-400,100,[0,35,0,None]),
 ('base',16,800,0,100,[0,35,0,None]),
 ('final_circuit',16,1300,0,100,[0,35,0,None]),
 ('landing_start',189,0,0,0,[0,0,0,0]),
 ('descent_orbit',31,800,-100,40,[1,-100,0,1]),
 ('touchdown',21,1400,0,0,[0,0,0,None])]
# Proposed aircraft-specific trim, not an instruction to modify the physical FC.
PARAMETERS={'FW_THR_TRIM':.121,'FW_AIRSPD_TRIM':SPEED}

def items(rows=ROWS):
 result=[]
 for seq,(name,cmd,n,e,h,params) in enumerate(rows):
  lat=LAT+math.degrees(n/6371000);lon=LON+math.degrees(e/(6371000*math.cos(math.radians(LAT))))
  result.append(dict(seq=seq,name=name,command=cmd,frame=6 if cmd in [16,31,21] else 2,
   current=seq==0,autocontinue=True,params=params+[lat if cmd in [16,31,21] else 0,lon if cmd in [16,31,21] else 0,h],
   north_m=n,east_m=e,height_m=h))
 return result

def sitl_lines():
 lines=['auto_after 100']
 for item in items():
  p=[float('nan') if x is None else x for x in item['params']]
  lines.append('mission '+' '.join(str(x) for x in [item['command'],*p]))
 return lines

def export(out, data=None, parameters=PARAMETERS, auto_after_release_s=40, plan_name="carrier_circuit.plan"):
 out=Path(out);out.mkdir(parents=True,exist_ok=True)
 data=items() if data is None else data;(out/'mission.json').write_text(json.dumps({'version':1,'origin':[LAT,LON,0],
  'auto_after_release_s':auto_after_release_s,'parameters_proposed':parameters,'items':data},indent=2)+'\n')
 plan={'fileType':'Plan','version':1,'groundStation':'QGroundControl',
 'geoFence':{'circles':[],'polygons':[],'version':2},'rallyPoints':{'points':[],'version':2},
 'mission':{'cruiseSpeed':SPEED,'hoverSpeed':5,'firmwareType':12,'vehicleType':1,'version':2,
 'plannedHomePosition':[LAT,LON,0],'items':[{'type':'SimpleItem','autoContinue':True,'command':v['command'],
  'doJumpId':v['seq']+1,'frame':3 if v['frame']==6 else v['frame'],'params':v['params']} for v in data]}}
 (out/plan_name).write_text(json.dumps(plan,indent=2)+'\n')
 if math.degrees(math.atan2(40,600))>5:raise ValueError('Approach exceeds default FW_LND_ANG')
 return data
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();export(a.out)
