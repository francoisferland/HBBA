#ifndef QUADRIC_H
#define QUADRIC_H

#include <ros/ros.h>	// For ros::Time

// For the comparison operator and access to eigen
#include <pcl/pcl.h>
#include <pcl/filters/conditional_removal.h>

#include <geometry_msgs/PoseStamped.h>

#include <math.h>

namespace quadrics
{
	// Types of quadric ( Warning : Cone or Cylinder are infinite so
	// 			it needs to be used along with Planes. )
	enum Type 
	{ 
		QuadricT, // Base type. Should not be used in filters.
		SphereT, // Used, for example, on localized sound 
		ConeT, // Used for poses coming from NoiseDetector
		PlaneT, // Could be used for limiting heights (of people), 
			// or restricted areas (from landmarks)
		CylinderT // Used for people ( along with planes )
	};

	// Reference on quadrics : http://en.wikipedia.org/wiki/Quadric
	class Quadric
	{
		public:
			Quadric( geometry_msgs::PoseStamped msg )
			{
				pose_ = msg.header.stamp;

				// Default operator (means "on or inside")
				op_ = pcl::ComparisonOps::GE; // GreatEqual

				// Init comparison values
				m_ = Eigen::Matrix3f::Zero();
				v_ = Eigen::Vector3f::Zero();
				s_ = 0.0;

				// Init type to generic Quadric
				type_ = QuadricT;
			}

			// Rotate the quadric
			void rotate()
			{
				// Construct rotation matrix from quaternion
				Eigen::Matrix3f rot( pose_.orientation.x,
						pose_.orientation.y,
						pose_.orientation.z,
						pose_.orientation.q );
				
				// Rotate matrix and vector
				m_ = rot * m_;
				v_ = rot * v_;

			}

			// Get the stamp
			ros::Time stamp()
			{
				return pose_.header.stamp;
			}

			std::string frame()
			{
				return pose_.header.frame_id;
			}

			// Get the comparison operator
			pcl::ComparisonOps::CompareOp op()
			{
				return op_;
			}

			Eigen::Matrix3f m()
			{
				return m_;
			}

			Eigen::Vector3f v()
			{
				return v_;
			}

			float s()
			{
				return s_;
			}

			Type type()
			{
				return type_;
			}

		private:

			// Initalisation pose
			geometry_msgs::PoseStamped pose_;

			/******************************************
			 * 	Using : http://docs.pointclouds.org/trunk/classpcl_1_1_tf_quadratic_x_y_z_comparison.html#details
			 * 		Equation will be :
			 * 	(p').m_.p + 2.v_.p + s_ (op_) 0
			 * 	where :
			 *	 - p is the 3D point to test (x, y, z)
			 *	 - (op_) is the operator of comparison
			 *	 - v_ is often the origin of the quadric
			 *	 	(x0, y0, z0) by -1
			 *	 - m_ is the quadritic matrix (3x3)
			 *	 	often diagonal with parameters
			 *	 - s_ is a float value.
			 * **********************************************/

			// Comparison operator
			pcl::ComparisonOps::CompareOp op_;

			// Need to be init in 
			Eigen::Matrix3f m_; // Comparison matrix
			Eigen::Vector3f v_; 
			float s_; // Comparison scalar

			// Type of quadric
			Type type_;
	
	}; // End of class
}	// End of namespace "quadrics"

#endif
