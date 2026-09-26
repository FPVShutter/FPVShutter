// Host simulation of src/dji_rsdk_camera.cpp + the shared transport: a fake
// camera drives the 0019 handshake, 1D02 pushes, 1D03 record, keep-alive,
// watchdog recovery and the rejection path. Uses the stub/ headers.
#include "dji_rsdk_camera.h"
#include "dji_rsdk_protocol.h"
#include "dji_duml_transport.h"
#include "settings.h"
#include <NimBLEDevice.h>
#include <vector>
unsigned long g_millis=1000; std::vector<std::vector<uint8_t>> g_writes; notify_callback g_notify=nullptr; int g_disconnects=0;
// ---- fakes for modules not under test
static ShutterSettings S; ShutterSettings& settingsGet(){return S;}
bool camRegistryActiveMac(uint8_t,char*,size_t){return false;}
void camRegistryRemember(uint8_t,const char*,const char*){} bool camRegistryMayAutoConnect(uint8_t,const char*){return false;} void camRegistryClearDiscovered(){}
void scanResultsAdd(uint8_t,const char*,const char*,int8_t,bool){} bool scanResultsIsShowAll(){return false;} void scanResultsStart(){} void scanResultsMarkComplete(){} void scanResultsUpdateSavedStatus(){}
bool serialConfigFcUartActive(){return false;}
// ----
static int fails=0;
#define CHECK(c) do{ if(!(c)){printf("  FAIL line %d: %s\n",__LINE__,#c);fails++;} else printf("  ok: %s\n",#c);}while(0)
static bool lastWrite(RsdkFrame&f,size_t back=0){ if(g_writes.size()<=back)return false; auto&w=g_writes[g_writes.size()-1-back]; static std::vector<uint8_t> keep; keep=w; return rsdkParseFrame(keep.data(),keep.size(),f);}
static void camSend(uint8_t type,uint16_t seq,uint16_t key,const uint8_t*p,size_t n,size_t split=0){
  uint8_t b[128]; size_t fl=rsdkBuildFrame(b,sizeof b,type,seq,key>>8,key&0xFF,p,n);
  if(split && split<fl){ g_notify(nullptr,b,split,true); g_notify(nullptr,b+split,fl-split,true);} else g_notify(nullptr,b,fl,true);}
static void tick(unsigned ms){ g_millis+=ms; djiRsdkUpdate(); }
int main(){
  printf("== happy path (Action 4 approves)\n");
  djiRsdkInit(); djiRsdkTargetMac("60:60:1f:00:00:01"); tick(10);
  CHECK(djiRsdkGetState()==BLE_AUTHENTICATING); CHECK(g_writes.empty());
  tick(100); CHECK(g_writes.empty());              // before the 300 ms connect delay
  tick(300);
  RsdkFrame f; CHECK(lastWrite(f) && f.key()==0x0019 && f.cmdType==0x02 && f.payloadLen==33);
  CHECK(f.payload[4]==6 && f.payload[5]==0xaa && f.payload[26]==DJI_RSDK_VERIFY_MODE && (f.payload[27]|(f.payload[28]<<8))==4);
  uint16_t ourSeq=f.seq;
  uint8_t resp[9]={0x33,0xFF,0,0, 0x00, 0,0,0,0}; camSend(0x20,ourSeq,0x0019,resp,9);
  CHECK(djiRsdkGetState()==BLE_AUTHENTICATING);
  uint8_t creq[33]={0}; creq[0]=0x00;creq[1]=0x00;creq[2]=0x33;creq[3]=0xFF; creq[26]=2; creq[27]=0; // allowed
  camSend(0x02,0x0042,0x0019,creq,33,20);          // split across two notifications
  size_t hsStart=g_writes.size(); size_t before=g_writes.size(); tick(5);
  CHECK(djiRsdkIsReady());
  CHECK(lastWrite(f,g_writes.size()-before-1) && f.key()==0x0019 && f.isResponse() && f.seq==0x0042 && f.payloadLen==9 && f.payload[4]==0);
  CHECK(!strcmp(djiRsdkGetTelemetry().model,"DJI Osmo Action 4"));
  before=hsStart; tick(5);
  bool sawSub=false,sawVer=false; for(size_t i=before;i<g_writes.size();i++){RsdkFrame g; auto w=g_writes[i]; if(rsdkParseFrame(w.data(),w.size(),g)){ if(g.key()==0x1D05&&g.payload[0]==3&&g.payload[1]==20)sawSub=true; if(g.key()==0x0000)sawVer=true;}}
  CHECK(sawSub); CHECK(sawVer);
  uint8_t st[38]={0}; st[0]=1; st[1]=1; st[2]=16; st[3]=6; st[23]=0xE4; st[24]=0x02; st[37]=77; st[15]=0x10; st[16]=0x27;
  camSend(0x00,7,0x1D02,st,38);
  auto&t=djiRsdkGetTelemetry(); CHECK(t.dataValid && t.state==CAM_STATE_STANDBY && t.recTimeSeconds==740 && t.batteryPercent==77 && t.storageRaw==10000);
  CHECK(djiRsdkSendStartRecord()); CHECK(lastWrite(f) && f.key()==0x1D03 && f.payload[4]==0 && f.payload[2]==0xFF && f.payload[3]==0x33);
  st[1]=3; st[5]=12; camSend(0x00,8,0x1D02,st,38); CHECK(t.state==CAM_STATE_RECORDING && t.recTimeSeconds==12);
  CHECK(djiRsdkSendStopRecord()); CHECK(lastWrite(f) && f.payload[4]==1);
  uint8_t rr[5]={0}; camSend(0x20,f.seq,0x1D03,rr,5);
  st[1]=1; camSend(0x00,9,0x1D02,st,38); CHECK(t.state==CAM_STATE_STANDBY && t.recTimeSeconds==740);
  printf("== keep-alive + watchdog soft recover\n");
  auto count=[&](size_t from,uint16_t key){int c=0; for(size_t i=from;i<g_writes.size();i++){RsdkFrame g; auto w=g_writes[i]; if(rsdkParseFrame(w.data(),w.size(),g)&&g.key()==key)c++;} return c;};
  before=g_writes.size(); for(int i=0;i<34;i++){ camSend(0x00,100+i,0x1D02,st,38); tick(500);}   // 17 s of live 2 Hz pushes
  CHECK(count(before,0x0000)==1); CHECK(count(before,0x1D05)==0); CHECK(djiRsdkIsReady());
  before=g_writes.size(); for(int i=0;i<30;i++) tick(500);   // camera goes quiet for 15 s
  { bool sub=false; for(size_t i=before;i<g_writes.size();i++){RsdkFrame g; auto w=g_writes[i]; if(rsdkParseFrame(w.data(),w.size(),g)&&g.key()==0x1D05)sub=true;} CHECK(sub); }  // 15 s quiet -> resubscribe
  CHECK(djiRsdkIsReady()); int d0=g_disconnects; tick(5001); CHECK(g_disconnects==d0+1);  // no answer -> full reconnect
  printf("== rejection path\n");
  g_writes.clear(); djiRsdkInit(); djiRsdkTargetMac("60:60:1f:00:00:01"); tick(10); tick(400);
  CHECK(lastWrite(f) && f.key()==0x0019);
  creq[27]=1; camSend(0x02,0x50,0x0019,creq,33); d0=g_disconnects; tick(5);
  CHECK(g_disconnects==d0+1 && !djiRsdkIsReady() && !strcmp(djiRsdkGetLastError(),"camera rejected pairing"));
  printf("== garbage + DUML frame on the link does not break parsing\n");
  djiRsdkInit(); djiRsdkTargetMac("x"); tick(10); tick(400); creq[27]=0;
  uint8_t duml[13]={0x55,0x0D,0x04,0x33,2,1,0,1,0x40,0,0,0,0}; g_notify(nullptr,duml,13,true);
  camSend(0x02,0x60,0x0019,creq,33); tick(5); CHECK(djiRsdkIsReady());
  printf(fails?"\nFAILURES: %d\n":"\nALL PASS\n",fails); return fails;
}
