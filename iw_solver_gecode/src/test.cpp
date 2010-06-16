#include "iw_solver_gecode/iw_solver_gecode.hpp"

int main(int argc, char** argv)
{
	typedef iw_solver_gecode::gecode_solver::base_t base_t;
	iw_solver_gecode::gecode_solver solver;
	base_t& b = solver;

	// This is now automatic:
	// b.add_resource("CPU");
	// b.add_resource("Sensor");
	// b.add_info("INFO");
	base_t::strat_vec_t cs, us;
	b.clear_model();
	cs.push_back(std::make_pair("CPU", 4));
	cs.push_back(std::make_pair("Sensor", 1));
	us.push_back(std::make_pair("INFO", 2));
	b.add_strategy("S1", cs, us);
	cs[0].second = 5;
	us[0].second = 3;
	b.add_strategy("S2", cs, us);
	cs[0].second = 9;
	us[0].second = 2;
	b.add_strategy("S3", cs, us);
	b.set_resource_max("CPU", 10);
	b.set_resource_max("Sensor", 1);
	b.clear_reqs();
	b.set_info_min("INFO", 2);
	base_t::sol_vec_t res(b.strat_count());

	b.solve(res);
}

