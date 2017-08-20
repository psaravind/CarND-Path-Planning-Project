#include <string>
#include "snapshot.h"

Snapshot::Snapshot(int _lane, 
	double _s, 
	double _v, 
	double _a, 
	string _state) {

	lane = _lane;
	s = _s;
	v = _v;
	a = _a;
	state = _state;
}

Snapshot::~Snapshot() {}