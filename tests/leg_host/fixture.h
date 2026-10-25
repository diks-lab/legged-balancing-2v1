#pragma once
#include "LegGeometry.h"
// Historical fixture: prove old control/protocol behavior is unchanged under
// the same geometry. New persisted geometry is covered without this fixture.
static struct LegFixture { LegFixture(){ leg::activeGeometry()={2007,1675,2071,2412}; } } leg_fixture;
