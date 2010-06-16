#include <iw_solver_interface/iw_solver_base.hpp>
#include <vector>
#include <map>

namespace iw_solver_gecode 
{

	typedef std::vector<int> vec_t;
	typedef std::map<unsigned int, vec_t> map_t;

	class gecode_solver: 
		public iw_solver_interface::iw_solver_base<gecode_solver>
	{
	public:
		typedef iw_solver_interface::iw_solver_base<gecode_solver> base_t;

		gecode_solver();
		~gecode_solver();

		void set_resource_max(unsigned int i, int max);
		void set_info_min(unsigned int i, int min);

		void add_strategy(const unsigned int i, const std::vector<int>& cs,
			const std::vector<int>& us);

		void clear_model(size_t r, size_t i);
		void clear_reqs();

		void solve(std::vector<bool>& res);

	private:
		vec_t cmax_;
		vec_t umin_;
		// Cost of each strat i for resource m.
		map_t cm_;
		// Utility effect of strat i on info p;
		map_t up_;

	};

}

