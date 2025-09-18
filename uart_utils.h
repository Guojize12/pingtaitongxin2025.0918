#pragma once
#include <Arduino.h>

void log2(const char* msg);
void log2Val(const char* k, int v);
void log2Str(const char* k, const char* v);

void sendRaw(const char* s);
void sendCmd(const char* cmd);

void readDTU();
void setLineHandler(void (*handler)(const char*));