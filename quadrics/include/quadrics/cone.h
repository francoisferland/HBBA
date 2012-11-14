#ifndef CONE_H
#define CONE_H

#include <quadrics/quadric.h>

/*******************************************************************
 * 			SMALL EXPLANATION :
 *
 * Simple equation of a sphere of a circular cone along the z axis :
 *
* 	(x - x0)² / a² + (y - y0)² / a² = (z - z0)² / b²
 * 	(x - x0)² * b² + (y - y0)² * b² - (z - z0)² * a² = 0
 *
 * 	where (x0 y0 z0) is the origin,
 * 	a = sin( aperture )
 *	b = cos( aperture )
 *
 * 	Ref : http://en.wikipedia.org/wiki/Conical_surface
 * 		http://en.wikipedia.org/wiki/Quadric
 *
 * 	Using matrices, it gives :
 *
 * 	(x)T( b² 0 0 )(x)     (x) ( - x0 * b² ) 
 *	(y) ( 0 b² 0 )(y) + 2.(y).( - y0 * b² ) - (x0² + y0² - z0²) [OP] 0
 *	(z) ( 0 0 -a²)(z)     (z) ( + z0 * a² )
 *
 * 	Form to respect : p'.(m_).p + 2.(v_)'.p + s_ [OP] 0
 *
 * 	so it gives for a quadric :
 * 		- an identity matrix in m_
 * 		- the position of the sphere by -1 in v_
 * 		- s_ =  - ( x0² + y0² + z0² )
 *
 * 	We want to keep what is OUTSIDE the cone so : op_ is ">=" (GE)
 **********************************************************************/

namespace quadrics
{
	// Inherit the generic quadric type.
	class Cone : public Quadric
	{
		public :
			Cone( geometry_msgs::PoseStamped origin, 
				geometry_msgs::PoseStamped toward, 
						float aperture ): 
				Quadric( origin ), aperture_( aperture );
				a_( 0.0 ), b_( 0.0 ),

			{
				// Operator
				op_ = pcl::ComparisonOps::GE;

				// Set origin
				geometry_msgs::Point p = 
						origin.pose.position;

				// Calc a_ and b_ from aperture
				a_ = cos( aperture );
				b_ = sin( aperture );

				// Use coma initializer
				m_ <<   b_ * b_,   0    ,    0      ,
					  0    , b_ * b_,    0      ,
				   	  0    ,   0    , - a_ * a_ ;

				// Vector
				v_ << 	- p.x * b_ * b_, 
					- p.y * b_ * b_,
					  p.z * a_ * a_;

				// Scalar value
				s_ = - p.x * p.x - p.y * p.y + p.z * p.z ;

				// Change type to Cone
				type_ = quadrics::ConeT;
			}

			// Get cone parameters
			float aperture()    {	return aperture_;  }
			float a()	{	return a_;	}
			float b()	{	return b_;	}

		private:
			float aperture_, a_, b_;
	
	};	// End of class
}		// End of namespace "quadrics"

#endif
