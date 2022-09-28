#include "Tri.h"

Tri::Tri() {}

Tri::Tri(
	const FVector& _a,
	const FVector &_b,
	const FVector &_c,
	uint16 _mesh_index
) {
	a = _a;
	b = _b;
	c = _c;
	center = (a + b + c) * ONE_THIRD;
	area_plus_near_epsilon = get_area(a, b, c) + NEAR_EPSILON;
	normal = FVector::CrossProduct(a-c, a-b).GetSafeNormal();
	mesh_index=  _mesh_index;
	flags = A_OK | B_OK | C_OK;
}

Tri::~Tri() {}

bool Tri::operator == (const Tri& other) const {
	return (*this).a == other.a && (*this).b == other.b && (*this).c == other.c;
}

float Tri::get_area(const FVector& a, const FVector& b, const FVector& c) {
	const FVector ab = a - b;
	const FVector ac = a - c;
	return FVector::CrossProduct(ab, ac).Size() * 0.5f;
}

bool Tri::point_near(const FVector& p) const {
	const float area_1 = get_area(p, a, b);	
	const float area_2 = get_area(p, a, c);	
	const float area_3 = get_area(p, b, c);

	if (area_1 + area_2 + area_3 > area_plus_near_epsilon) {
		return false;
	}
	return true;
}