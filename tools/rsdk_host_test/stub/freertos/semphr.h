#pragma once
typedef void* SemaphoreHandle_t;
inline SemaphoreHandle_t xSemaphoreCreateMutex(){return (void*)1;}
inline int xSemaphoreTake(SemaphoreHandle_t,unsigned){return 1;}
inline int xSemaphoreGive(SemaphoreHandle_t){return 1;}
