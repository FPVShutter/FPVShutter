#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <string>
extern unsigned long g_millis; inline unsigned long millis(){return g_millis;}
inline uint32_t esp_random(){return 4;}
inline size_t strlcpy(char*d,const char*s,size_t n){size_t l=strlen(s); if(n){size_t c=l<n-1?l:n-1; memcpy(d,s,c); d[c]=0;} return l;}
#define log_printf(...) printf(__VA_ARGS__)
class String { public: String(const char*s=""):s_(s){} const char*c_str()const{return s_.c_str();} std::string s_; };
