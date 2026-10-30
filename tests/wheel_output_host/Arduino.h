#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stddef.h>
#include <algorithm>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define PI 3.14159265358979323846
#define HEX 16
class Print {
 public:
  virtual ~Print()=default;
  virtual size_t write(uint8_t){return 1;}
  template<class T> size_t print(T){return 1;}
  template<class T> size_t print(T,int){return 1;}
  template<class T> size_t println(T){return 1;}
  size_t println(){return 1;}
};
class Stream:public Print {};
inline uint32_t test_us=1000000;
inline unsigned long micros(){return test_us;}
inline unsigned long millis(){return test_us/1000;}
inline void delay(unsigned long ms){test_us+=ms*1000;}
inline void delayMicroseconds(unsigned int us){test_us+=us;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 1;}

using std::min;using std::max;
