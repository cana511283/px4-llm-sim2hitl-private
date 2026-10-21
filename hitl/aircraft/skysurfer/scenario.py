import json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'plant'))
import wind as plant_wind
from host.abi import header,Initial
ALLOWED={'schema_version','aircraft_id','name','height_agl_m','velocity_mode','tas_m_s','body_velocity_m_s','alpha_deg','beta_deg','euler_deg','latitude_deg','longitude_deg','site_msl_m','geoid_separation_m','wind_ned_m_s','wind','angular_rates_rad_s','seed_policy','carrier','flat_rangefinder'}
def load_scene(value):
 scene=json.loads(Path(value).read_text()) if isinstance(value,(str,Path)) else dict(value)
 unknown=set(scene)-ALLOWED
 if unknown:raise ValueError('Unsupported scene fields: '+','.join(sorted(unknown)))
 if scene.get('schema_version')!=1 or scene.get('aircraft_id')!='skysurfer':raise ValueError('Scene schema/aircraft mismatch')
 for name in ['site_msl_m','geoid_separation_m','seed_policy']:
  if scene.get(name,0)!=0:raise ValueError(name+' is unsupported by this source recipe; supported value is 0')
 if scene.get('angular_rates_rad_s',[0,0,0])!=[0,0,0]:raise ValueError('angular_rates_rad_s is unsupported by this source recipe')
 if "flat_rangefinder" in scene and type(scene["flat_rangefinder"]) is not bool:raise ValueError("flat_rangefinder must be boolean")
 load_wind(scene)  # malformed wind is rejected here, before anything starts
 def number(name,default,lo,hi):
  v=scene.get(name,default)
  if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError(name+' outside supported range')
  return v
 mode=scene.get('velocity_mode','tas')
 if mode=='tas':
  if 'body_velocity_m_s' in scene:raise ValueError('TAS and body velocity modes are mutually exclusive')
  speed=number('tas_m_s',0,0,200);a=math.radians(number('alpha_deg',0,-45,45));b=math.radians(number('beta_deg',0,-45,45))
  velocity=[speed*math.cos(a)*math.cos(b),speed*math.sin(b),speed*math.sin(a)*math.cos(b)]
 elif mode=='body':
  if any(n in scene for n in ['tas_m_s','alpha_deg','beta_deg']):raise ValueError('Body velocity and TAS fields are mutually exclusive')
  velocity=scene.get('body_velocity_m_s')
 else:raise ValueError('Unknown velocity mode')
 euler=scene.get('euler_deg',[0,.8633,0])
 for name,values,limits in [('body_velocity',velocity,[200]*3),('Euler',euler,[180,80,360])]:
  if not isinstance(values,list) or len(values)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>lim for v,lim in zip(values,limits)):raise ValueError(name+' invalid')
 initial=header(Initial);initial.height_m=number('height_agl_m',0,0,10000);initial.body_velocity_m_s[:]=velocity;initial.euler_deg[:]=euler
 initial.latitude_deg=number('latitude_deg',22.910275834988674,-90,90);initial.longitude_deg=number('longitude_deg',120.31096929726908,-180,180)
 if 'carrier' in scene:
  carrier=scene['carrier'];keys={'wait_s','climb_s','cruise_s','height_m','speed_m_s'}
  if not isinstance(carrier,dict) or set(carrier)!=keys:raise ValueError('Invalid carrier fields')
  if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in carrier.values()):raise ValueError('Carrier values must be finite numbers')
  if carrier['wait_s']<0 or carrier['climb_s']<1 or carrier['cruise_s']<0 or not 0<carrier['height_m']<=10000 or not 0<carrier['speed_m_s']<=200:raise ValueError('Carrier outside supported range')
  if initial.height_m!=0 or any(initial.body_velocity_m_s) or any(initial.euler_deg) or 'wind' in scene or any(scene.get('wind_ned_m_s',[0,0,0])):raise ValueError('Carrier requires zero-state ground start and no wind')
 return scene,initial

def load_wind(scene):
 """(config, reminders) of plant/wind.py. The scene's wind_ned_m_s is the steady wind (same as wind.steady_ned_m_s).
 HITL starts on the ground, so the reminders assume a ground start."""
 wind=dict(scene.get('wind') or {})
 steady=scene.get('wind_ned_m_s',[0,0,0])
 if steady!=[0,0,0]:
  if 'steady_ned_m_s' in wind and wind['steady_ned_m_s']!=steady:raise ValueError('wind_ned_m_s and wind.steady_ned_m_s differ')
  wind['steady_ned_m_s']=steady
 return plant_wind.parse(wind or None,ground_start=scene.get('height_agl_m',0)==0)
