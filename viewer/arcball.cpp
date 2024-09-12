#include "Arcball.h"
#include <stdio.h>

using namespace MeshLib;

CArcball::CArcball( int win_width, int win_height, int ox, int oy )
{
	m_radius = (win_width < win_height )? win_width/2: win_height/2;

  m_center = CPoint2( win_width/2, win_height/2 );

  CPoint2 p(ox,oy);

  _plane2sphere( p, m_position );
}

void CArcball::_plane2sphere( const CPoint2 & p, CPoint & q )
{

  CPoint2 f = p;
  f /= m_radius;

  double l = sqrt( f*f );

  if( l > 1.0 ){
      q=CPoint( f[0]/l, f[1]/l,0);
      return;
  }

  double fz = sqrt( 1 - l * l );

  q = CPoint( f[0],f[1],fz );
}

CQrot CArcball::update( int nx, int ny )
{
    CPoint position;
    _plane2sphere( CPoint2(nx,ny), position );
    CPoint cp = m_position^position;
    CQrot r(m_position * position, cp[0],cp[1],cp[2]);
    m_position = position;

    return r;
}