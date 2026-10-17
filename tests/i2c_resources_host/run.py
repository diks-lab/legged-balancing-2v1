"""Exercise the installed Wire allocation path with allocation failures injected.

Production Wire methods and application reservation code are compiled verbatim;
only allocator, HAL and RTOS are doubled. No claim about actual ESP32 heap size.
"""
from pathlib import Path
import re
import os
import subprocess
import tempfile

core = Path(os.environ.get('PLATFORMIO_CORE_DIR', Path.home()/'.platformio'))
framework = Path(os.environ.get('ARDUINO_ESP32_DIR', core/'packages/framework-arduinoespressif32'))
wire = (framework/'libraries/Wire/src/Wire.cpp').read_text()
header = (framework/'libraries/Wire/src/Wire.h').read_text()
app = Path('src/main.cpp').read_text()

def extract(source, signature):
    start = source.index(signature)
    pos = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]

size = int(re.search(r'#define I2C_BUFFER_LENGTH (\d+)', header).group(1))
assert size == 128
assert 'bufferSize(I2C_BUFFER_LENGTH)' in wire
assert 'setBufferSize(' not in app  # Preserve capacity, including after reservation.
code = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#define CONFIG_DISABLE_HAL_LOCKS 0
#define SOC_I2C_SUPPORT_SLAVE 0
constexpr size_t I2C_BUFFER_LENGTH=128;
constexpr int ESP_OK=0,pdTRUE=1,portMAX_DELAY=-1;
using esp_err_t=int;
int alloc_count=0,fail_alloc=0,live_allocs=0,mutex_calls=0,fail_mutex=0;
int driver_calls=0,fail_driver=0;
bool initialized[2]={};
std::vector<size_t> sizes;
void* wireMalloc(size_t size){
  sizes.push_back(size);
  if(++alloc_count==fail_alloc)return nullptr;
  ++live_allocs;return std::malloc(size);
}
void wireFree(void* ptr){if(ptr){--live_allocs;std::free(ptr);}}
int mutex_storage;
void* xSemaphoreCreateMutex(){return ++mutex_calls==fail_mutex?nullptr:&mutex_storage;}
int xSemaphoreTake(void*,int){return pdTRUE;}
void xSemaphoreGive(void*){}
bool i2cIsInit(int num){return initialized[num];}
int i2cInit(int num,int,int,uint32_t){
  if(++driver_calls==fail_driver)return -1;
  initialized[num]=true;return ESP_OK;
}
void log_e(const char*,...){}
void log_w(const char*,...){}
class TwoWire {
 public:
  int num,sda=-1,scl=-1;size_t bufferSize=I2C_BUFFER_LENGTH;
  uint8_t *rxBuffer=nullptr,*txBuffer=nullptr;void* lock=nullptr;
  unsigned timeout=0;
  explicit TwoWire(int n):num(n){}
  ~TwoWire(){freeWireBuffer();}
  bool initPins(int a,int b){sda=a;scl=b;return true;}
  bool allocateWireBuffer();void freeWireBuffer();
  bool begin(int sdaPin,int sclPin,uint32_t frequency);
  void setTimeOut(unsigned value){timeout=value;}
};
'''
for signature in ['bool TwoWire::allocateWireBuffer()', 'void TwoWire::freeWireBuffer()',
                  'bool TwoWire::begin(int sdaPin, int sclPin, uint32_t frequency)']:
    code += extract(wire, signature).replace('malloc(', 'wireMalloc(').replace('free(', 'wireFree(') + '\n'
code += r'''
TwoWire I2Cone(0),I2Ctwo(1);
bool i2c_resources_attempted=false,i2c0_resources_ready=false,i2c1_resources_ready=false;
bool outputs=true;
void gateWheelOutputsOff(){outputs=false;}
std::vector<std::string> stages;
void printHttpStartupMemory(const char* stage){stages.push_back(stage);}
struct {
  void print(const char*){}
} Serial;
'''
code += extract(app, 'void prepareI2cResources() {')
code += r'''
void reset(){
  I2Cone.freeWireBuffer();I2Ctwo.freeWireBuffer();assert(live_allocs==0);
  I2Cone.lock=I2Ctwo.lock=nullptr;
  initialized[0]=initialized[1]=false;
  I2Cone.timeout=I2Ctwo.timeout=0;
  i2c_resources_attempted=i2c0_resources_ready=i2c1_resources_ready=false;
  alloc_count=fail_alloc=mutex_calls=fail_mutex=driver_calls=fail_driver=0;
  outputs=true;sizes.clear();stages.clear();
}
int main(){
  for(int fail=0;fail<=4;++fail){
    reset();fail_alloc=fail;prepareI2cResources();
    assert(!outputs && i2c_resources_attempted);
    assert(i2c0_resources_ready==(fail!=1 && fail!=2));
    assert(i2c1_resources_ready==(fail!=3 && fail!=4));
    assert((stages==std::vector<std::string>{"before_i2c0","after_i2c0","before_i2c1","after_i2c1"}));
    for(auto size:sizes)assert(size==128);
    assert(live_allocs==(fail?2:4)); // Failed TX releases that bus's RX as well.
    if(!fail){
      assert(I2Cone.timeout==5 && I2Ctwo.timeout==5);
      assert(I2Cone.sda==19 && I2Cone.scl==18 && I2Ctwo.sda==23 && I2Ctwo.scl==5);
      const auto count=alloc_count;
      assert(I2Cone.begin(19,18,400000) && I2Ctwo.begin(23,5,400000));
      assert(alloc_count==count); // Idempotent begin preserves reserved buffers.
    }
    const auto count=alloc_count;prepareI2cResources();assert(alloc_count==count);
  }
  for(int fail=1;fail<=2;++fail){
    reset();fail_mutex=fail;prepareI2cResources();
    assert(!outputs && i2c0_resources_ready==(fail!=1) && i2c1_resources_ready==(fail!=2));
    assert(live_allocs==2);
    reset();fail_driver=fail;prepareI2cResources();
    assert(!outputs && i2c0_resources_ready==(fail!=1) && i2c1_resources_ready==(fail!=2));
    assert(live_allocs==2); // Driver failure frees that bus's Wire buffers.
  }
  reset();puts("PASS: real Wire RX/TX 128-byte allocations, all four failures, locks/driver failures, cleanup and preallocated bus reuse");
}
'''
with tempfile.TemporaryDirectory(prefix='i2c-resources-') as directory:
    cpp = Path(directory)/'test.cpp'
    binary = Path(directory)/'test'
    cpp.write_text(code)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
