#pragma once
#include <Arduino.h>
#include <list>
#include <string>
#include <vector>
typedef int esp_power_level_t;
struct ble_gap_conn_desc { struct { unsigned encrypted:1, authenticated:1, bonded:1; } sec_state; };
enum { ESP_PWR_LVL_N9, ESP_PWR_LVL_N0, ESP_PWR_LVL_P9 };
#define BLE_HS_IO_NO_INPUT_OUTPUT 3
class NimBLEUUID { public: NimBLEUUID(const char*){} std::string toString() const {return "0xfff0";} };
class NimBLEAddress { public: NimBLEAddress(){} NimBLEAddress(const std::string&){} NimBLEAddress(const char*){} std::string toString() const {return "aa:bb:cc:dd:ee:ff";} };
class NimBLERemoteCharacteristic;
typedef void (*notify_callback)(NimBLERemoteCharacteristic*,uint8_t*,size_t,bool);
#include <vector>
extern std::vector<std::vector<uint8_t>> g_writes; extern notify_callback g_notify;
class NimBLERemoteCharacteristic { public: bool canWrite(){return 1;} bool canWriteNoResponse(){return 1;} bool canNotify(){return 1;}
  bool writeValue(const uint8_t*d,size_t n,bool=false){g_writes.emplace_back(d,d+n);return 1;} bool subscribe(bool, notify_callback cb, bool=true){g_notify=cb;return 1;} };
class NimBLERemoteService { public: NimBLERemoteCharacteristic* getCharacteristic(const NimBLEUUID&){static NimBLERemoteCharacteristic c; return &c;}
  NimBLEUUID getUUID(){return NimBLEUUID("0000fff0-0000-1000-8000-00805f9b34fb");} };
class NimBLEClient;
class NimBLEClientCallbacks { public: virtual ~NimBLEClientCallbacks(){} virtual void onConnect(NimBLEClient*){} virtual void onDisconnect(NimBLEClient*){} virtual void onAuthenticationComplete(ble_gap_conn_desc*){} };
class NimBLEClient { public: void setClientCallbacks(NimBLEClientCallbacks*,bool){} void setConnectTimeout(int){} bool connect(const NimBLEAddress&){return 1;}
  bool isConnected(){return 1;} int disconnect(){extern int g_disconnects; g_disconnects++; return 0;} NimBLERemoteService* getService(const NimBLEUUID&){static NimBLERemoteService sv; return &sv;}
  std::vector<NimBLERemoteService*>* getServices(bool=false){static std::vector<NimBLERemoteService*> v{getService(NimBLEUUID(""))}; return &v;} };
class NimBLEAdvertisedDevice { public: NimBLEAddress getAddress(){return {};} bool haveName(){return 1;} std::string getName(){return "";} int getRSSI(){return -50;}
  std::string getManufacturerData(){return "";} bool isAdvertisingService(const NimBLEUUID&){return 0;} };
class NimBLEAdvertisedDeviceCallbacks { public: virtual ~NimBLEAdvertisedDeviceCallbacks(){} virtual void onResult(NimBLEAdvertisedDevice*)=0; };
class NimBLEScanResults { public: int getCount(){return 0;} };
class NimBLEScan { public: void setAdvertisedDeviceCallbacks(NimBLEAdvertisedDeviceCallbacks*,bool){} void setActiveScan(bool){} void setInterval(int){} void setWindow(int){}
  void setMaxResults(int){} bool start(uint32_t, void(*)(NimBLEScanResults), bool){return 1;} bool isScanning(){return 0;} bool stop(){return 1;} };
class NimBLEDevice { public: static void init(const std::string&){} static void setPower(esp_power_level_t){} static void setSecurityAuth(bool,bool,bool){}
  static void setSecurityIOCap(int){} static NimBLEScan* getScan(){static NimBLEScan s; return &s;} static NimBLEClient* createClient(){return new NimBLEClient;}
  static std::list<NimBLEClient*>* getClientList(){static std::list<NimBLEClient*> l; return &l;} static NimBLEAddress getAddress(){return {};} };
