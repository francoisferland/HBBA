#include "iw_solver_interface/iw_solver_base.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

namespace {
	/// \brief A dummy solver to test compilations issues with the interface.
	class dummy_solver: 
		public iw_solver_interface::iw_solver_base<dummy_solver>
	{
	public:
		typedef iw_solver_interface::iw_solver_base<dummy_solver> base_t;

		void set_resource_max(unsigned int i, int max)
		{
			res_[i] = max;
		}

		void set_info_min(unsigned int i, int min)
		{
			info_[i] = min;
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
		std::vector<int> strat_;
	};


}

int main(int argc, char** argv)
{
	dummy_solver::base_t b = dummy_solver();

	b.add_resource("CPU");
	b.add_info("INFO");
	dummy_solver::base_t::strat_vec_t cs, us;
	b.clear_model();
	cs.push_back(std::make_pair("CPU", 5));
	us.push_back(std::make_pair("INFO", 2));
	b.add_strategy("S1", cs, us);
	us[0].second = 3;
	b.add_strategy("S2", cs, us);
	b.set_resource_max("CPU", 10);
	b.set_info_min("INFO", 1);
	b.clear_reqs();
	dummy_solver::base_t::sol_vec_t res(b.strat_count());
	b.solve(res);
}


