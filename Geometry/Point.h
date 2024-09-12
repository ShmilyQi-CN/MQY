#ifndef _MESHLIB_POINT_H_
#define _MESHLIB_POINT_H_

#include <assert.h>
#include <math.h>

namespace MeshLib{

class CPoint{

public:

	CPoint( double x, double y, double z ){ v[0] = x; v[1] = y; v[2] = z;};
	CPoint() { v[0] = v[1] = v[2] = 0; };
	~CPoint(){};

	double & operator[]( int i)		  { assert( 0<=i && i<3 ); return v[i]; };
	double   operator()( int i) const { assert( 0<=i && i<3 ); return v[i]; };
	double   operator[]( int i) const { assert( 0<=i && i<3 ); return v[i]; };
	double norm() const { return sqrt( fabs( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] ) ); };

	CPoint  & operator += ( const CPoint & p) { v[0] += p(0); v[1] += p(1); v[2] += p(2); return *this; }; 
	CPoint  & operator -= ( const CPoint & p)  { v[0] -= p(0); v[1] -= p(1); v[2] -= p(2); return *this; };
	CPoint  & operator *= ( const double  s) { v[0] *= s   ; v[1] *=    s; v[2] *=    s; return *this; };
	CPoint  & operator /= ( const double  s) { v[0] /= s   ; v[1] /=    s; v[2] /=    s; return *this; };

	double   operator*( const CPoint & p ) const 
	{
		return v[0] * p[0] + v[1] * p[1] + v[2] * p[2]; 
	};

	CPoint   operator+( const CPoint & p  ) const
	{
		CPoint r( v[0] + p[0], v[1] + p[1], v[2] + p[2] );
		return r;
	};
	CPoint   operator-( const CPoint & p  ) const
	{
		CPoint r( v[0] - p[0], v[1] - p[1], v[2] - p[2] );
		return r;
	};
	CPoint   operator*( const double s  ) const
	{
		CPoint r( v[0] * s, v[1] * s, v[2] * s );
		return r;
	};
	CPoint   operator/( const double s  ) const
	{
		CPoint r( v[0] / s, v[1] / s, v[2] / s );
		return r;
	};

	CPoint operator^( const CPoint & p2) const
	{
		CPoint r( v[1] * p2[2] - v[2] * p2[1],
			              v[2] * p2[0] - v[0] * p2[2],
		                v[0] * p2[1] - v[1] * p2[0]);
		return r;
	};

	CPoint operator-() const
	{
		CPoint p(-v[0],-v[1],-v[2]);
		return p;
	};
protected:
	double v[3];
};


}//name space MeshLib

#endif //_MESHLIB_POINT_H_ defiined