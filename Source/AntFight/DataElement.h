#pragma once
#include "Tri.h"

struct DataElement {
	DataElement() : occupied{false} {} 
	~DataElement() {}
	Tri tri;
	bool occupied;
};