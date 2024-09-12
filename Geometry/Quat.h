#ifndef _QUAT_H_
#define _QUAT_H_

#include <math.h>
#include "Point.h"

typedef double mat44[4][4];

namespace MeshLib
{

  class CQrot
{
public:
  CQrot(){m_w=1; m_x=m_y=m_z=0;};

  CQrot( double w, double x, double y, double z)
  {
    m_w = w;
    m_x = x;
    m_y = y;
    m_z = z;
  };

  CQrot( const CQrot & q )
  {
    m_w = q.m_w; 
    m_x =  q.m_x; 
    m_y =  q.m_y; 
    m_z =  q.m_z;
  }

	CQrot & operator=(const CQrot & q)
  {
      m_w = q.m_w;
      m_x  = q.m_x;
      m_y  = q.m_y;
      m_z  = q.m_z;
      return *this;
  }

  ~CQrot(){};
  
  CQrot& operator^(double p);
  void convert( double * );
  CPoint operator*( const CPoint & v );

public:
  double m_w,m_x,m_y,m_z;
  void normalize();

   CQrot operator*( const CQrot & q ) const;
  double operator^( const CQrot & q ) const;
};

}
#endif