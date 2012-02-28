#include "iw_solver_interface/iw_solver_base.hpp"
#include "iw_solver_interface/solver_node.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

namespace {
	/// \brief A dummy solver to test compilations issues with the interface.
	class dummy_solver 
	{
	public:
		void set_resource_max(unsigned int i, int max)
		{
			res_[i] = max;
		}

		void set_util_min(unsigned int i, int min)
		{
			info_[i] = min;
		}

        void set_util_int(unsigned int i, int intensity)
        {
            intensity_[i] = intensity;
        }

		void add_strategy(const unsigned int id, const std::vector<int>& cs,
			const std::vector<int>& us)
		{
		}

		void clear_model(size_t r, size_t i)
		{
			
			std::cout << "Refreshing model..." << std::endl;
			res_ = std::vector<int>(r);
			info_ = std::vector<int>(i);
		}

		void clear_reqs()
		{
			std::fill(info_.begin(), info_.end(), 0);
		}

		void solve(std::vector<bool>& res)
		{
			std::fill(res.begin(), res.end(), false);
		}


	private:
		std::vector<int> res_;
		std::vector<int> info_;
        std::vector<int> intensity_;
		std::vector<int> strat_;
	};


}

int main(int argc, char** argv)
{
    using namespace iw_solver_interface;

    typedef SolverNode<dummy_solver> node_t;

    ros::init(argc, argv, "dummy_solver");
    node_t n;

	n.add_resource("CPU");
	n.add_util("INFO");
	node_t::strat_vec_t cs, us, u_min;
	n.clear_model();
	cs.push_back(std::make_pair("CPU", 5));
	us.push_back(std::make_pair("INFO", 2));
	u_min.push_back(std::make_pair("",0));
	n.add_strategy("S1", cs, us, u_min);
	us[0].second = 3;
	u_min[0].first="INFO";
	u_min[0].second=2;
	n.add_strategy("S2", cs, us,u_min);
	n.set_resource_max("CPU", 10);
	n.set_util_min("INFO", 1);
	n.clear_reqs();
	node_t::sol_vec_t res(n.strat_count());
	n.solve(res);

    ros::spin();
}


