from pathlib import Path
import re, subprocess

def check(tmp, flags):
    app=Path('src/main.cpp').read_text()
    def function(name):
        match=re.search(r'^(?:void|bool|int16_t) '+name+r'\([^;]*?\)\s*\{',app,re.M)
        assert match,name
        pos=match.end();depth=1
        while depth:
            depth+=(app[pos]=='{')-(app[pos]=='}');pos+=1
        return app[match.start():pos]
    constants=app[app.index('constexpr uint8_t kRightLegServoId'):app.index('\n\n',app.index('constexpr uint16_t kBothLegsTestSpeed'))]
    stubs=r'''
#include "LegGeometry.h"
#include <cassert>
#include <initializer_list>
#include <cmath>
#include <cstdio>
unsigned ons=0,writes=0;bool stopped=true;
struct {void println(const char*){} template<class...T>void printf(const char*,T...){} }Serial;
struct {void SyncWritePosEx(uint8_t*ids,unsigned n,int16_t*p,uint16_t*,uint8_t*){
 for(unsigned i=0;i<n;++i)assert(leg::inRange(ids[i]==1,p[i]));
 ++writes;
}}sms_sts;
bool prepareLegacyLegAction(){return stopped;}
void ensureLegServoTorqueEnabled(){++ons;}
float constrain(float p,float lo,float hi){return fmaxf(lo,fminf(hi,p));}
'''
    body='\n'.join(function(n) for n in ('moveRightLegForTest','moveLeftLegForTest','rightLegPositionFromPercent','leftLegPositionFromPercent','moveBothLegsToPercent'))
    tests=r'''
int main(){
 assert(leg::position(true,0)==1997&&leg::position(false,0)==2081);
 assert(leg::position(true,10)==1974&&leg::position(false,10)==2105);
 for(float p:{-10.f,0.f,10.f,20.f,30.f,100.f}){assert(leg::inRange(true,rightLegPositionFromPercent(p)));
  assert(leg::inRange(false,leftLegPositionFromPercent(p)));moveBothLegsToPercent(p);}
 moveRightLegForTest(leg::position(true,0));moveLeftLegForTest(leg::position(false,0),"INNER_HOME");
 const auto before=writes,torque=ons;
 moveRightLegForTest(2007);moveRightLegForTest(1675);moveLeftLegForTest(2071,"MEASURED_END");moveLeftLegForTest(2412,"MEASURED_END");
 assert(writes==before&&ons==torque);
 stopped=false;moveBothLegsToPercent(10);moveRightLegForTest(1974);moveLeftLegForTest(2105,"10%");
 assert(writes==before&&ons==torque);
 puts("PASS: production legacy HOME/10%/percentage helpers use shared inner normal limits; invalid/end/active requests cannot enable torque or write goals");
}
'''
    source=Path(tmp)/'legacy.cpp';binary=Path(tmp)/'legacy'
    source.write_text(stubs+constants+body+tests)
    subprocess.run(['c++','-include','tests/leg_host/fixture.h','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(source),'-o',str(binary),*flags],check=True)
    subprocess.run([str(binary)],check=True)
    for path in ('src/main.cpp','src/bus_diagnostic.cpp','include/LegAcceptance.h','include/LegControl.h','include/LegSerialDiagnostics.h'):
        text=Path(path).read_text()
        for number in (2061,1892,2026,2208,2010):
            assert not re.search(r'\b'+str(number)+r'\b',text),(path,number)
