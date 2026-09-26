// Host unit test for src/dji_rsdk_protocol.cpp. Vectors are DJI's own:
// the mode-switch example in docs/protocol_data_segment.md and the ten
// frames in test/connect_cmd_frame_builder/connect_cmd_frame.txt of
// github.com/dji-sdk/Osmo-GPS-Controller-Demo (MIT).
#include "../../src/dji_rsdk_protocol.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
static int fails=0;
#define CHECK(c) do{ if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);fails++;} }while(0)
static std::vector<uint8_t> hex(const char*s){std::vector<uint8_t> v; for(;s[0]&&s[1];s+=2){unsigned x; sscanf(s,"%2x",&x); v.push_back(x);} return v;}
static std::vector<RsdkFrame> got; static std::vector<std::vector<uint8_t>> gotPayload;
static void h(const RsdkFrame&f,void*){got.push_back(f); gotPayload.emplace_back(f.payload,f.payload+f.payloadLen);}
int main(){
  // 1) DJI worked example: mode switch to hyperlapse
  auto ex=hex("AA1B00010000000005 0057EE1D04000033FF0A01473936F4FAE1D0");
  std::vector<uint8_t> e2; for(auto b:hex("AA1B000100000000050057EE1D04000033FF0A01473936F4FAE1D0")) e2.push_back(b);
  uint8_t pl[]={0x00,0x00,0x33,0xFF,0x0A,0x01,0x47,0x39,0x36}; uint8_t out[64];
  size_t n=rsdkBuildFrame(out,sizeof out,0x01,5,0x1D,0x04,pl,9);
  CHECK(n==27 && memcmp(out,e2.data(),27)==0);
  RsdkFrame f; CHECK(rsdkParseFrame(e2.data(),e2.size(),f)); CHECK(f.key()==0x1D04 && f.seq==5 && f.payloadLen==9 && !f.isResponse());
  // 2) all 10 connect-request vectors from DJI's test dir
  static const char *VEC[]={
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000FE1F00000000796C667C",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000191A0000000033C8C7BF",
    "AA330002000000000100982F00197856341206383456789ABC000000000000000000000000000000008F1200000000AC768844",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000050B00000000E1B45396",
    "AA330002000000000100982F00197856341206383456789ABC000000000000000000000000000000004F0E000000009B1270F0",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000B71F00000000DD373154",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000501000000000368B20DD",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000B82100000000EF8376BB",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000100C00000000D9FB6877",
    "AA330002000000000100982F00197856341206383456789ABC00000000000000000000000000000000E10200000000048F9616",
  }; int k=0;
  uint8_t mac[6]={0x38,0x34,0x56,0x78,0x9A,0xBC};
  for(const char*line:VEC){ auto v=hex(line);
    RsdkFrame g; CHECK(rsdkParseFrame(v.data(),v.size(),g)); CHECK(g.key()==0x0019);
    uint16_t vd=g.payload[27]|(g.payload[28]<<8);
    uint8_t p[40]; size_t pn=rsdkBuildConnectRequest(p,0x12345678,mac,0,vd);
    size_t fn=rsdkBuildFrame(out,sizeof out,0x02,g.seq,0x00,0x19,p,pn);
    CHECK(fn==v.size() && memcmp(out,v.data(),fn)==0); k++; }
  printf("connect vectors checked: %d\n",k);
  // 3) reassembler: garbage + frame split across chunks + two frames in one chunk + corrupted frame
  RsdkReassembler r; std::vector<uint8_t> stream={0x55,0x0E,0x04,0xAA,0x01}; // DUML-ish junk + fake SOF
  stream.insert(stream.end(),e2.begin(),e2.end());
  auto bad=e2; bad[20]^=1; stream.insert(stream.end(),bad.begin(),bad.end());
  stream.insert(stream.end(),e2.begin(),e2.end());
  for(size_t i=0;i<stream.size();i+=7) r.feed(&stream[i],std::min<size_t>(7,stream.size()-i),h,nullptr);
  CHECK(got.size()==2); printf("reassembled %zu frames, crcErr=%u dropped=%u\n",got.size(),r.crcErrors(),r.droppedBytes());
  for(auto&p:gotPayload) CHECK(p.size()==9 && p[4]==0x0A);
  // 4) status push parse: synthesize from doc offsets
  uint8_t st[38]={0}; st[0]=0x01; st[1]=0x03; st[2]=16; st[3]=6; st[5]=0x2C; st[6]=0x01; // 300s
  st[15]=0x10; st[16]=0x27; st[23]=0xE4; st[24]=0x02; st[30]=1; st[37]=87;
  RsdkCameraStatus cs; CHECK(rsdkParseCameraStatus(st,38,cs));
  CHECK(cs.recordTimeS==300 && cs.remainCapacityMb==10000 && cs.remainTimeS==740 && cs.batteryPercent==87 && cs.tempOver==1 && rsdkFpsFromIdx(cs.fpsIdx)==60);
  CHECK(!rsdkParseCameraStatus(st,37,cs));
  // 5) record control polarity + model names
  uint8_t rc[9]; rsdkBuildRecordControl(rc,0x33FF0000,true); CHECK(rc[4]==0 && rc[2]==0xFF && rc[3]==0x33);
  rsdkBuildRecordControl(rc,0x33FF0000,false); CHECK(rc[4]==1);
  CHECK(rsdkModelName(0x33FF0000) && !strcmp(rsdkModelName(0x33FF0000),"Osmo Action 4"));
  CHECK(rsdkModelName(0xFF330000) && !strcmp(rsdkModelName(0xFF330000),"Osmo Action 4"));
  CHECK(rsdkModelName(0x0000FF44) && !strcmp(rsdkModelName(0x0000FF44),"Osmo Action 5 Pro"));
  CHECK(rsdkModelName(0x12345678)==nullptr);
  printf(fails?"FAILURES: %d\n":"ALL PASS\n",fails); return fails;
}
