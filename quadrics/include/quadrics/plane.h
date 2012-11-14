#ifndef PLANE_H
#define PLANE_H

#include <quadrics/quadric.h>

/*******************************************************************
 * 			SMALL EXPLANATION :
 *
 * Simple equation of a 3D plane :
 *	a.x + b.y + c.z + 1 = 0
 *
 * 	Using matrices, it gives :
 *
 * 	( x )T ( 0 0 0 )     ( a/2 ) 
 *	( y )  ( 0 0 0 ) + 2.( b/2 ) + 1 [OP] 0
 *	( z )  ( 0 0 0 )     ( c/2 )
 *
 * 	Form to respect : p'.(m_).p + 2.(v_)'.p + s_ [OP] 0
 *
 * 	so it gives for a quadric :
 * 		- m_ = 0
 * 		- v_ = values to calc
 * 		- s_ = 1
 *
 * 	We want a plane on the XY plane of the given pose. 
 * 	So we can construct with Cramer to get a, b and c.
 * 	Ref : http://en.wikipedia.org/wiki/Plane_(mathematics) (see method2)
 *
 *	
 *
 * 	We want to keep what is on the z > 0 Side to OP is ...
 **********************************************************************/

namespace quadrics
{
	// Inherit the generic quadric type.
	class Plane : public Quadric
	{
		public :
			Plane( geometry_msgs::PoseStamped pose ) : 
				Quadric( msg )
			{
				// Operator
				// TODO
				op_ = pcl::ComparisonOps::GE;

				geometry_msgs::Point p = pose.pose.position;

				// Use coma initializer
				
				// Empty matrix
				m_ << 0, 0, 0,
				      0, 0, 0,
				      0, 0, 0;
				
				s_ = 1;

				// Use Kramer to calc a, b and c

				// Get 3 points of the plane XY, 
				// 	i.e same z as given pose
				geometry_msgs::Point( 0.0, 0.0, p.z)
				
				// Set vector	
				v_ <<  a_ / 2, b_ / 2, c_ / 2;


				// Change type to Sphere
				type_ = quadrics::PlaneT;
			}

		private:
			float a_, b_, c_;
	
	};	// End of class
}		// End of namespace "quadrics"

#endif
