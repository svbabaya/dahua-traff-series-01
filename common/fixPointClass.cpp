#include "fixPointClass.h"
#include "_common.h"

#ifdef _FixPoint_

const unsigned fixPoint::frac;

fixPoint::fixPoint(int32_t _val) { 
	val = fx_itox(_val, frac); 
}

fixPoint::fixPoint(float _val) { 
	val = fx_ftox(_val, frac); 
}

fixPoint::fixPoint(double _val) { 
	val = fx_dtox(_val, frac); 
}

fixPoint::fixPoint(size_t _val) { 
	val = fx_itox(int32_t(_val), frac); 
}

fixPoint::fixPoint(fixed_t _val, bool baseType) : val(_val) {}

fixPoint fixPoint::operator-() const { 
	return fixPoint(-val, true); 
}

int32_t fixPoint::toInt32() const { 
	return fx_xtoi(val, frac); 
}

float fixPoint::toFloat() const { 
	return fx_xtof(val, frac); 
}

double fixPoint::toDouble() const { 
	return fx_xtod(val, frac); 
}

int32_t round(const fixPoint& rght) { 
	return fx_roundx(rght.val, rght.frac); 
}

fixPoint abs(const fixPoint& rght) { 
	return rght < 0 ? -rght : rght; 
}

fixPoint sqrt(const fixPoint& rght) { 
	return fixPoint(fx_sqrtx(rght.val, rght.frac), true); 
}

bool operator==(const fixPoint& left, const fixPoint& rght) { 
	return left.val == rght.val; 
}

bool operator!=(const fixPoint& left, const fixPoint& rght) { 
	return left.val != rght.val; 
}

bool operator> (const fixPoint& left, const fixPoint& rght) { 
	return left.val > rght.val; 
}

bool operator< (const fixPoint& left, const fixPoint& rght) { 
	return left.val < rght.val; 
}

bool operator>=(const fixPoint& left, const fixPoint& rght) { 
	return left.val >= rght.val; 
}

bool operator<=(const fixPoint& left, const fixPoint& rght) { 
	return left.val <= rght.val; 
}

fixPoint operator+(const fixPoint& left, const fixPoint& rght) {
	 return fixPoint(left.val+rght.val, true); 
	}

fixPoint operator-(const fixPoint& left, const fixPoint& rght) { 
	return fixPoint(left.val-rght.val, true); 
}

fixPoint operator*(const fixPoint& left, const fixPoint& rght) { 
	return fixPoint(fx_mulx(left.val,rght.val,left.frac), true); 
}

fixPoint operator/(const fixPoint& left, const fixPoint& rght) { 
	return fixPoint(fx_divx(left.val,rght.val,left.frac), true); 
}

fixPoint& fixPoint::operator+=(const fixPoint& rght) {
	val += rght.val;
	return *this;
}

fixPoint& fixPoint::operator-=(const fixPoint& rght) {
	val -= rght.val;
	return *this;
}

fixPoint& fixPoint::operator*=(const fixPoint& rght) {
	val = fx_mulx(val, rght.val, frac);
	return *this;
}

fixPoint& fixPoint::operator/=(const fixPoint& rght) {
	val = fx_divx(val, rght.val, frac);
	return *this;
}

#endif