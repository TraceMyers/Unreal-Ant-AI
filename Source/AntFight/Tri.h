#pragma once

struct Tri {
	
	static constexpr float NEAR_EPSILON = 3.00f;
	static constexpr float ONE_THIRD = 1.0f / 3.0f;

	FVector a;
	FVector b;
	FVector c;
	FVector normal;
	FVector center;
	float area_plus_near_epsilon;
	uint16 mesh_index;
	uint32 flags;

	enum TRI_FLAGS {
		A_OK = 0x0001,
		B_OK = 0x0002,
		C_OK = 0x0004,
		OVERLAP = 0x0008
	};

	Tri();
	Tri(
		const FVector& _a,
		const FVector& _b,
		const FVector& _c,
		uint16 _mesh_index
	);
	~Tri();
	bool operator == (const Tri& other) const;
	static float get_area(const FVector& a, const FVector& b, const FVector& c);
	bool point_near(const FVector& p) const;
	
};

struct TriEdge {
	
	TriEdge(const FVector& _origin, const FVector& __end) {
		origin = _origin;
		_end = __end;
		const FVector edge_vec = __end - _origin;
		len = edge_vec.Size();
		norm = edge_vec * (1.0f / len);
	}

	FVector origin;
	FVector _end;
	FVector norm;
	float len;
};

struct PolyPoint {
	FVector pt;
	PolyPoint* edge;
	TArray<int> pending_edges; // edge array indices added w/ connection direction yet unknown
	int tri_edge_flags;
	static constexpr float EPSILON = 0.1f;

	PolyPoint() {
		edge = nullptr;
		tri_edge_flags = 0;
	}

	PolyPoint(const FVector& poi) {
		pt = poi;
		edge = nullptr;
		tri_edge_flags = 0;
	}

	void set(const FVector& poi) {
		pt = poi;
	}

	void operator = (const PolyPoint& other) {
		this->pt = other.pt;
		this->edge = other.edge;
		this->pending_edges = other.pending_edges;
		this->tri_edge_flags = other.tri_edge_flags;
	}	

	bool operator == (const PolyPoint& other) const {
		return FVector::DistSquared((*this).pt, other.pt) < EPSILON;
	}

	bool operator != (const PolyPoint& other) const {
		return FVector::DistSquared((*this).pt, other.pt) >= EPSILON;
	}
	
	bool already_pending(const int other_i) {
		for (int i = 0; i < pending_edges.Num(); i++) {
			if (pending_edges[i] == other_i) {
				return true;
			}
		}
		return false;
	}
};

struct Polygon {
	TArray<FVector> points;
	int overlapping_tri_index;
	FVector normal;
	int mesh_index;
};

// 10/3/22: a little double precision helps TriGrid::set_point_of_intersection(), seeking as much accuracy as possible.
struct DoubleVector {
	DoubleVector(double a, double b, double c) : X{a}, Y{b}, Z{c} {}
	DoubleVector(float a, float b, float c) : X{a}, Y{b}, Z{c} {}
	DoubleVector(const FVector& vec) : X{vec.X}, Y{vec.Y}, Z{vec.Z} {}
	
	double X;
	double Y;
	double Z;

	static double DotProduct(const DoubleVector& a, const DoubleVector& b) {
		return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
	}

	static DoubleVector CrossProduct(const DoubleVector& a, const DoubleVector& b) {
		return DoubleVector(
			a.Y * b.Z - a.Z * b.Y,
			a.Z * b.X - a.X * b.Z,
			a.X * b.Y - a.Y * b.X	
		);
	}
};
