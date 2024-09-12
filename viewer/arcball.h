#ifndef _ARCBALL_H_
#define _ARCBALL_H_

#include "../geometry/Point.h"
#include "../geometry/Point2.h"
#include "../geometry/quat.h"

namespace MeshLib
{
class CArcball
{
public:
  CArcball(){};

  CArcball( int width, int height, int ox, int oy );
  CQrot update( int nx, int ny );


private:
  void _plane2sphere( const CPoint2 & v, CPoint & r );

  CPoint   m_position;
  double   m_radius;
  CPoint2 m_center;
};

}
#endif