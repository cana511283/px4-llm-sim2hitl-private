#include "hil_model_api.h"
#include "plant_flat_ground.h"
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <poll.h>
#define EXPORT __attribute__((visibility("default")))
#define CAP 8192
typedef struct {uint64_t stamp;uint32_t len,kind;uint8_t bytes[280];} Packet;
typedef struct {uint64_t sensor,gps,heartbeat,commands,replaced,errors,log_drops,max_write_ns;} IoStats;
extern uint64_t hil_worker_now_ns(void);
static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition=PTHREAD_COND_INITIALIZER;
static pthread_t sender;
static int fd=-1,started,stopping,enabled,inflight,command_inflight;
static Packet latest[4],commands[128],records[CAP];
static int pending[4];
static int flat_rangefinder;
static uint64_t range_sent;
EXPORT uint64_t hil_io_range_count(void){pthread_mutex_lock(&mutex);uint64_t n=range_sent;pthread_mutex_unlock(&mutex);return n;}
/* Called only by management before worker start; no running configuration changes. */
EXPORT void hil_io_flat_rangefinder(uint32_t enabled_value){pthread_mutex_lock(&mutex);flat_rangefinder=enabled_value==1;range_sent=0;pthread_mutex_unlock(&mutex);}
static uint64_t read_cmd,write_cmd,read_log,write_log,epoch_us;
static uint8_t sequence;
static uint16_t eph=30,epv=50;
static uint8_t fix=3,satellites=10;
static IoStats stats;
/* Additive diagnostic API; the model ABI and existing IoStats stay unchanged. */
typedef struct {
 uint64_t begin_ns,end_ns,packet_stamp_ns;
 uint32_t len,offset,kind,reason;
 int32_t write_errno,poll_result,poll_revents;
 uint32_t retries,short_writes,reserved;
} IoEvent;
#define DIAG_CAP 256
static IoEvent events[DIAG_CAP],last_event;
static uint64_t event_read,event_write,event_drops;
EXPORT uint32_t hil_io_drain_events(IoEvent*out,uint32_t capacity){
 pthread_mutex_lock(&mutex);uint32_t n=0;
 while(event_read<event_write&&n<capacity)out[n++]=events[event_read++%DIAG_CAP];
 pthread_mutex_unlock(&mutex);return n;
}
EXPORT void hil_io_diagnostics(IoEvent*out,uint64_t*counts){
 pthread_mutex_lock(&mutex);*out=last_event;counts[0]=event_write;counts[1]=event_drops;pthread_mutex_unlock(&mutex);
}
static uint16_t crc_byte(uint16_t crc,uint8_t byte){uint8_t t=byte^(uint8_t)crc;t^=t<<4;return (crc>>8)^((uint16_t)t<<8)^((uint16_t)t<<3)^(t>>4);}
static void u16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void u32(uint8_t*p,uint32_t v){for(int k=0;k<4;k++)p[k]=v>>(8*k);}
static void u64(uint8_t*p,uint64_t v){for(int k=0;k<8;k++)p[k]=v>>(8*k);}
static void f32(uint8_t*p,double value){float v=(float)value;uint32_t bits;memcpy(&bits,&v,4);u32(p,bits);}
static Packet frame(uint32_t id,uint8_t extra,uint8_t *payload,int size,int kind){
 while(size>1&&!payload[size-1])size--;
 Packet p={.stamp=hil_worker_now_ns(),.kind=kind,.len=(uint32_t)(size+12)};
 uint8_t*b=p.bytes;b[0]=253;b[1]=size;b[4]=sequence++;b[5]=2;b[6]=51;b[7]=id;b[8]=id>>8;b[9]=id>>16;memcpy(b+10,payload,size);
 uint16_t crc=0xffff;for(int k=1;k<size+10;k++)crc=crc_byte(crc,b[k]);crc=crc_byte(crc,extra);u16(b+size+10,crc);return p;
}
EXPORT int hil_io_encode(const HilModelOutput *o,uint64_t epoch,uint8_t *out,uint32_t capacity,uint32_t flags) {
 uint8_t p[65]={0};uint32_t length=0;Packet packet;
 double v[]={o->accel_m_s2[0],o->accel_m_s2[1],o->accel_m_s2[2],o->gyro_rad_s[0],o->gyro_rad_s[1],o->gyro_rad_s[2],o->magnetic_uT[0]*.01,o->magnetic_uT[1]*.01,o->magnetic_uT[2]*.01,o->absolute_pressure_pa*.01,o->differential_pressure_pa*.01,0,0};
 /* PX4 requires ABS_PRESSURE, PRESSURE_ALT and TEMPERATURE together for BARO.
    The latter two are explicitly ISA-derived transport metadata, not source channels. */
 uint32_t fields=o->fields_updated;
 if(fields&0x200) {
  if(!(o->absolute_pressure_pa>0)||!isfinite(o->absolute_pressure_pa))return -4;
  double temperature_k=288.15*pow(o->absolute_pressure_pa/101325.,0.1902632365084836);
  v[11]=(288.15-temperature_k)/.0065;v[12]=temperature_k-273.15;fields|=0x1800;
 }
 for(int k=0;k<13;k++)if(!isfinite(v[k])||fabs(v[k])>3.4028234e38)return -4;
 u64(p,epoch+o->model_time_ns/1000);for(int k=0;k<13;k++)f32(p+8+k*4,v[k]);u32(p+60,fields);packet=frame(107,108,p,65,1);
 if(packet.len>capacity)return -4;memcpy(out,packet.bytes,packet.len);length=packet.len;
 if(flags&1) {
  double lat=o->gps_lla_msl[0]*1e7,lon=o->gps_lla_msl[1]*1e7,alt=o->gps_lla_msl[2]*1000,speed=o->ground_speed_m_s*100;
  if(!isfinite(lat)||fabs(lat)>900000000||!isfinite(lon)||fabs(lon)>1800000000||!isfinite(alt)||alt<-2147483648.||alt>2147483647.||!isfinite(speed)||speed<0||speed>65534||!isfinite(o->course_deg))return -4;
  for(int k=0;k<3;k++)if(!isfinite(o->velocity_ned_m_s[k])||o->velocity_ned_m_s[k]*100<-32768||o->velocity_ned_m_s[k]*100>32767)return -4;
  memset(p,0,sizeof(p));u64(p,epoch+o->gps_sample_time_ns/1000);u32(p+8,(uint32_t)(int32_t)llrint(lat));u32(p+12,(uint32_t)(int32_t)llrint(lon));u32(p+16,(uint32_t)(int32_t)llrint(alt));u16(p+20,eph);u16(p+22,epv);u16(p+24,llrint(speed));for(int k=0;k<3;k++)u16(p+26+k*2,(uint16_t)(int16_t)llrint(o->velocity_ned_m_s[k]*100));
  int64_t cog=llrint(o->course_deg*100)%36000;if(cog<0)cog+=36000;u16(p+32,cog);p[34]=fix;p[35]=satellites;packet=frame(113,124,p,39,2);
  if(length+packet.len>capacity)return -4;memcpy(out+length,packet.bytes,packet.len);length+=packet.len;
 }
 /* Explicit SkySurfer truth layout: r[19] AGL, r[11/12] pitch/roll.
    FRD body-down rotated into NED; yaw cannot change horizontal-plane range. */
 if(flags&4){
  double pitch=o->raw_source[11],roll=o->raw_source[12],agl=o->raw_source[19],range;
  double ray[]={sin(pitch)*cos(roll),-sin(roll),cos(pitch)*cos(roll)};
  int valid=plant_flat_ground_range_m(-agl,ray,.05,150.,&range);
  memset(p,0,sizeof(p));u32(p,(uint32_t)((epoch+o->model_time_ns/1000)/1000));
  u16(p+4,5);u16(p+6,15000);u16(p+8,valid?(uint16_t)llrint(range*100):15001);
  p[10]=0; /* laser */ p[11]=0;p[12]=25; /* body downward */ p[13]=255; /* variance unknown */
  p[38]=valid?100:1; /* MAVLink 1 means quality zero, 0 means unknown */
  packet=frame(132,85,p,39,4);
  if(length+packet.len>capacity)return -4;memcpy(out+length,packet.bytes,packet.len);length+=packet.len;
 }
 if(flags&2){memset(p,0,9);p[4]=1;p[5]=12;p[8]=3;packet=frame(0,50,p,9,3);if(length+packet.len>capacity)return -4;memcpy(out+length,packet.bytes,packet.len);length+=packet.len;}
 return length;
}
static void *send_loop(void *unused){
 (void)unused;
#ifdef __APPLE__
 pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE,0);
#endif
 for(;;){
  pthread_mutex_lock(&mutex);
  while(!stopping&&read_cmd==write_cmd&&!pending[0]&&!pending[1]&&!pending[2]&&!pending[3])pthread_cond_wait(&condition,&mutex);
  if(stopping){pthread_mutex_unlock(&mutex);break;}
  Packet p={0};
  if(read_cmd<write_cmd)p=commands[read_cmd++%128];
  else for(int k=0;k<4;k++)if(pending[k]){p=latest[k];pending[k]=0;break;}
  inflight=p.kind>0;command_inflight=p.kind==0;pthread_mutex_unlock(&mutex);
  IoEvent event={.begin_ns=hil_worker_now_ns(),.packet_stamp_ns=p.stamp,.len=p.len,.kind=p.kind};
  uint32_t offset=0;
  while(offset<p.len){
   if(event.retries && hil_worker_now_ns()-event.begin_ns>=100000000){event.reason=1;break;}
   ssize_t n=write(fd,p.bytes+offset,p.len-offset);
   event.retries++;
   if(n>0){if((uint32_t)n<p.len-offset)event.short_writes++;offset+=n;continue;}
   event.write_errno=n<0?errno:0;
   if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR){event.reason=2;break;}
   if(hil_worker_now_ns()-event.begin_ns>=100000000){event.reason=1;break;}
   struct pollfd waiter={fd,POLLOUT,0};event.poll_result=poll(&waiter,1,1);event.poll_revents=waiter.revents;
  }
  event.end_ns=hil_worker_now_ns();event.offset=offset;
  uint64_t duration=event.end_ns-event.begin_ns;
  pthread_mutex_lock(&mutex);
  if(duration>stats.max_write_ns)stats.max_write_ns=duration;
  if(offset!=p.len||duration>=1000000){
   last_event=event;
   if(event_write-event_read<DIAG_CAP)events[event_write++%DIAG_CAP]=event;else event_drops++;
  }
  inflight=command_inflight=0;pthread_cond_broadcast(&condition);
  if(offset!=p.len){stats.errors++;enabled=0;read_cmd=write_cmd;memset(pending,0,sizeof(pending));}
  else {
   if(p.kind==1)stats.sensor++;else if(p.kind==2)stats.gps++;else if(p.kind==3)stats.heartbeat++;else if(p.kind==4)range_sent++;else stats.commands++;
   p.stamp=hil_worker_now_ns();
   if(write_log-read_log<CAP)records[write_log++%CAP]=p;else stats.log_drops++;
  }
  pthread_mutex_unlock(&mutex);
 }
 return NULL;
}
EXPORT int hil_io_start(int serial_fd){
 if(started||serial_fd<0)return 2;
 fd=serial_fd;stopping=enabled=flat_rangefinder=0;range_sent=0;read_cmd=write_cmd=read_log=write_log=0;memset(pending,0,sizeof(pending));memset(&stats,0,sizeof(stats));event_read=event_write=event_drops=0;memset(&last_event,0,sizeof(last_event));started=1;
 if(pthread_create(&sender,NULL,send_loop,NULL)){started=0;return 3;}return 0;
}
EXPORT void hil_io_config(uint64_t epoch,uint32_t quality){pthread_mutex_lock(&mutex);epoch_us=epoch;sequence=0;fix=quality&255;satellites=(quality>>8)&255;eph=30;epv=50;enabled=1;pthread_mutex_unlock(&mutex);}
EXPORT int hil_io_command(const uint8_t *bytes,uint32_t length){
 if(!started||length>280)return 4;pthread_mutex_lock(&mutex);if(write_cmd-read_cmd>=128){pthread_mutex_unlock(&mutex);return 3;}
 Packet*p=&commands[write_cmd++%128];memset(p,0,sizeof(*p));p->len=length;memcpy(p->bytes,bytes,length);pthread_cond_signal(&condition);pthread_mutex_unlock(&mutex);return 0;
}
int hil_io_publish(const HilModelOutput *o){
 pthread_mutex_lock(&mutex);if(!started){pthread_mutex_unlock(&mutex);return 0;}
 int fault=stats.errors?9:stats.log_drops?6:0;
 if(fault||!enabled){pthread_mutex_unlock(&mutex);return fault;}
 uint8_t buffer[280];int length=hil_io_encode(o,epoch_us,buffer,sizeof(buffer),(o->gps_updated?1:0)|(o->model_time_ns%1000000000==0?2:0)|(flat_rangefinder&&o->model_time_ns%40000000==0?4:0));
 if(length<0){enabled=0;pthread_mutex_unlock(&mutex);return 4;}
 for(int index=0;index<length;){int len=buffer[index+1]+12;uint32_t id=buffer[index+7];int kind=id==107?0:id==113?1:id==132?3:2;
  if(pending[kind])stats.replaced++;Packet*p=&latest[kind];memset(p,0,sizeof(*p));p->stamp=hil_worker_now_ns();p->kind=kind+1;p->len=len;memcpy(p->bytes,buffer+index,len);pending[kind]=1;index+=len;}
 pthread_cond_signal(&condition);pthread_mutex_unlock(&mutex);return 0;
}
EXPORT void hil_io_clear_sensor(void){pthread_mutex_lock(&mutex);enabled=0;memset(pending,0,sizeof(pending));while(inflight)pthread_cond_wait(&condition,&mutex);pthread_mutex_unlock(&mutex);}
/* Management owns the producer lock before this barrier. No queued command
 * survives a stop/fault/recovery boundary; an in-flight write has a 100 ms
 * deadline and is recorded before this call returns. Sensor pacing is separate. */
EXPORT void hil_io_clear_commands(void){pthread_mutex_lock(&mutex);read_cmd=write_cmd;while(command_inflight)pthread_cond_wait(&condition,&mutex);pthread_mutex_unlock(&mutex);}
EXPORT void hil_io_stats(IoStats*out){pthread_mutex_lock(&mutex);*out=stats;pthread_mutex_unlock(&mutex);}
EXPORT uint32_t hil_io_drain(Packet*out,uint32_t capacity){pthread_mutex_lock(&mutex);uint32_t n=0;while(read_log<write_log&&n<capacity)out[n++]=records[read_log++%CAP];pthread_mutex_unlock(&mutex);return n;}
EXPORT void hil_io_stop(void){if(!started)return;pthread_mutex_lock(&mutex);stopping=1;enabled=0;memset(pending,0,sizeof(pending));read_cmd=write_cmd;pthread_cond_signal(&condition);pthread_mutex_unlock(&mutex);pthread_join(sender,NULL);started=0;fd=-1;}
