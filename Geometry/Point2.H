#ifndef _MESHLIB_POINT2_H_
#define _MESHLIB_POINT2_H_


namespace MeshLib{

class CPoint2
{

	public:
    CPoint2(){ m_c[0] = 0; m_c[1] = 0; };

    CPoint2( const CPoint2 & uv ) {	m_c[0] = uv.m_c[0]; m_c[1] = uv.m_c[1];};

    ~CPoint2(){};

    CPoint2( double  x, double y ) { m_c[0] = x; m_c[1] = y; };

    double & operator[]( int i ) { 	assert( i < 2 && i >= 0 ); return m_c[i]; };

		bool operator==( const CPoint2 & uv ) 
		{
			return ( m_c[0] == uv.m_c[0] && m_c[1] == uv.m_c[1] );
		}

    void operator/=( double s )
    {
      m_c[0]/=s;
      m_c[1]/=s;
    }
	private:
		double m_c[2];

};

inline CPoint2 operator+(  CPoint2 & uv0,  CPoint2 & uv1 )
{
	CPoint2 uv( uv0[0]+uv1[0], uv0[1] + uv1[1] );
	return uv;
};

inline CPoint2 operator-(  CPoint2 & uv0,  CPoint2 & uv1 )
{
	CPoint2 uv( uv0[0]-uv1[0], uv0[1] - uv1[1] );
	return uv;
};

inline CPoint2 operator*(  CPoint2 & uv0, const double s )
{
	CPoint2 uv( uv0[0] * s, uv0[1] * s );
	return uv;
};

inline CPoint2 operator/(  CPoint2 & uv0, const double s )
{
	CPoint2 uv( uv0[0] / s, uv0[1] / s );
	return uv;
};


inline double mag2(  CPoint2 & uv )
{
	return uv[0] * uv[0] + uv[1] * uv[1];
};

inline double mag(  CPoint2 & uv )
{
	return sqrt(uv[0] * uv[0] + uv[1] * uv[1]);
};

inline double cross(  CPoint2 uv1,  CPoint2 uv2 )
{
	return (uv1[0] * uv2[1] - uv1[1] * uv2[0]);
};

inline double operator*( CPoint2 a, CPoint2 b )
{
    return a[0] * b[0] + a[1] * b[1];
}


}; //namespace

#endif
