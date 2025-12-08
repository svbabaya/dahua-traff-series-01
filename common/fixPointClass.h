#ifndef __FIXPOINTCLASS_H__
#define __FIXPOINTCLASS_H__

#include "fixmath/fixmath.h"

#define DEFAULT_FRAC 11

class fixPoint {
public:
	fixPoint(int32_t _val = 0);
	fixPoint(float _val);
	fixPoint(double _val);
	fixPoint(size_t _val);
	fixPoint(fixed_t _val, bool baseType);

	fixPoint operator-() const;
	int32_t toInt32() const;
	float toFloat() const;
	double toDouble() const;
	friend int32_t round(const fixPoint& rght);
	friend fixPoint abs(const fixPoint& rght);
	friend fixPoint sqrt(const fixPoint& rght);
	
	friend bool operator==(const fixPoint& left, const fixPoint& rght);
	friend bool operator!=(const fixPoint& left, const fixPoint& rght);
	friend bool operator> (const fixPoint& left, const fixPoint& rght);
	friend bool operator< (const fixPoint& left, const fixPoint& rght);
	friend bool operator>=(const fixPoint& left, const fixPoint& rght);
	friend bool operator<=(const fixPoint& left, const fixPoint& rght);

	friend fixPoint operator+(const fixPoint& left, const fixPoint& rght);
	friend fixPoint operator-(const fixPoint& left, const fixPoint& rght);
	friend fixPoint operator*(const fixPoint& left, const fixPoint& rght);
	friend fixPoint operator/(const fixPoint& left, const fixPoint& rght);

	fixPoint& operator+=(const fixPoint& rght);
	fixPoint& operator-=(const fixPoint& rght);
	fixPoint& operator*=(const fixPoint& rght);
	fixPoint& operator/=(const fixPoint& rght);
private:
	static const unsigned frac = DEFAULT_FRAC;
	fixed_t val;
};

#endif
