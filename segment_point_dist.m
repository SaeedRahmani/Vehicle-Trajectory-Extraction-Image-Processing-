function dist = segment_point_dist ( p1, p2, p )

%*****************************************************************************80
%
%% SEGMENT_POINT_DIST: distance ( line segment, point ).
%
%  Discussion:
%
%    A line segment is the finite portion of a line that lies between
%    two points.
%
%    The nearest point will satisfy the condition
%
%      PN = (1-T) * P1 + T * P2.
%
%    T will always be between 0 and 1.
%
%  Licensing:
%
%    This code is distributed under the GNU LGPL license.
%
%  Modified:
%
%    04 December 2010
%
%  Author:
%
%    John Burkardt
%
%  Parameters:
%
%    Input, real P1(2,1), P2(2,1), the endpoints of the line segment.
%
%    Input, real P(2,1), the point whose nearest neighbor on the line
%    segment is to be determined.
%
%    Output, real DIST, the distance from the point to the line segment.
%

%
%  Destroy all row vectors!
%
  p1 = p1(:);
  p2 = p2(:);
  p = p(:);
%
%  If the line segment is actually a point, then the answer is easy.
%
  if ( p1(1:2,1) == p2(1:2,1) )

    t = 0.0;

  else

    bot = sum ( ( p2(1:2,1) - p1(1:2,1) ).^2 );

    t = ( p(1:2,1) - p1(1:2,1) )' * ( p2(1:2,1) - p1(1:2,1) ) / bot;

    t = max ( t, 0.0 );
    t = min ( t, 1.0 );

  end

  pn(1:2,1) = p1(1:2,1) + t * ( p2(1:2,1) - p1(1:2,1) );

  dist = sqrt ( sum ( ( pn(1:2,1) - p(1:2,1) ).^2 ) );

  return
end
