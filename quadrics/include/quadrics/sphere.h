#ifndef SPHERE_H
#define SPHERE_H

#include <quadrics/quadric.h>

/*******************************************************************
 * 			SMALL EXPLANATION :
 *
 * Simple equation of a sphere :
 *
 * 	(x - x0)² + (y - y0)² + (z - z0)² = radius²
 *
 * 	where (x0 y0 z0) is the origin of the sphere
 *
 * 	Using matrices, it gives :
 *
 * 	(x)T(1 0 0)(x)     (x)( -x0 ) 
 *	(y) (0 1 0)(y) + 2.(y)( -y0 ) - (radius² + x0² + y0² + z0²) [OP] 0
 *	(z) (0 0 1)(z)     (z)( -z0 )
 *
 * 	Form to respect : p'.(m_).p + 2.(v_)'.p + s_ [OP] 0
 *
 * 	so it gives for a quadric :
 * 		- an identity matrix in m_
 * 		- the position of the sphere by -1 in v_
 * 		- s_ =  - ( radius² + x0² + y0² + z0² )
 *
 * 	We want to keep what is OUTSIDE the sphere so : op_ is ">=" (GE)
 **********************************************************************/

namespace quadrics
{
	// Inherit the generic quadric type.
	class Sphere : public Quadric
	{
		public :
			Sphere( geometry_msgs::PoseStamped pose,
							float radius ) : 
				Quadric( msg ), radius_( radius )
			{
				// Operator
				op_ = pcl::ComparisonOps::GE;

				geometry_msgs::Point p = pose.pose.position;

				// Use coma initializer
				
				// Identity matrix
				m_ << 1, 0, 0,
				      0, 1, 0,
				      0, 0, 1;

				// Position in message
				v_ <<   - p.x, - p.y, - p.z;

				s_ = - ( radius * radius + p.x * p.x +
					 p.y * p.y + p.z * p.z );

				// Change type to Sphere
				type_ = quadrics::SphereT;
			}

			float radius()
			{
				return radius_;
			}

		private:
			float radius_;
	
	};	// End of class
}		// End of namespace "quadrics"

#endif
