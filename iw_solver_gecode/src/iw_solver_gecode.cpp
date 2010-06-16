#include "iw_solver_gecode/iw_solver_gecode.hpp"
#include <iw_solver_interface/iw_solver_base.hpp>
#include <gecode/driver.hh>
#include <gecode/int.hh>
#include <gecode/minimodel.hh>
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

namespace iw_solver_gecode 
{
	class gecode_script: public Gecode::Script
	{
	public:
		typedef std::vector<int> vec_t;
		typedef std::map<unsigned int, vec_t> map_t;

		gecode_script(const vec_t& cmax, const vec_t& umin, 
			const map_t& cm, const map_t& up);

		gecode_script(bool share, gecode_script& s): 
			Gecode::Script(share, s),
			cmax_(s.cmax_), umin_(s.umin_), cm_(s.cm_), up_(s.up_)
		{
			a_.update(*this, share, s.a_);

		}

		~gecode_script();

		virtual Gecode::Script* copy(bool share)
		{
			return new gecode_script(share, *this);
		}


		void constrain(const Space& b)
		{
			//const gecode_script& p = 
			//static_cast<const gecode_script&>(b);
		}

		const Gecode::IntVarArray& a() const { return a_; }

		void build_res(std::vector<bool>& res)
		{
			res = std::vector<bool>(a_.size());
			for (size_t i = 0; i < res.size(); ++i)
				res[i] = a_[i].val();
		}

	private:
		const vec_t& cmax_;
		const vec_t& umin_;
		
		// Cost of each strat i for resource m.
		const map_t& cm_;
		// Utility effect of strat i on info p;
		const map_t& up_;

		Gecode::IntVarArray a_;

	};
}

using namespace iw_solver_gecode; 

void gecode_solver::set_resource_max(unsigned int i, int max)
{
	cmax_[i] = max;
}

void gecode_solver::set_info_min(unsigned int i, int min)
{
	umin_[i] = min;
}

void gecode_solver::add_strategy(const unsigned int i, 
	const std::vector<int>& cs, const std::vector<int>& us)
{
	cm_[i] = cs;
	up_[i] = us;
}

void gecode_solver::clear_model(size_t r, size_t i)
{
	cmax_ = std::vector<int>(r);
	umin_ = std::vector<int>(i);
}

void gecode_solver::clear_reqs()
{
	std::fill(umin_.begin(), umin_.end(), 0);
}

void gecode_solver::solve(std::vector<bool>& res)
{
	gecode_script* m = new gecode_script(cmax_, umin_, cm_, up_);
	// *solve*
	Gecode::DFS<gecode_script> e(m);
	m->status();
	std::cout << m->a() << std::endl;
	gecode_script* s = e.next();
	while (s)
	{
		std::cout << s->a() << std::endl;
		gecode_script* n;
		// If we're at the last state, build the result vector.
		if (!(n = e.next()))
			s->build_res(res);
		delete s;
		s = n;
	} 

	delete m;

}

gecode_script::gecode_script(const vec_t& cmax, const vec_t& umin, 
		const map_t& cm, const map_t& up): 
	cmax_(cmax), umin_(umin), cm_(cm), up_(up)
{
	using namespace Gecode;

	size_t sc = cm_.size();

	// Activation vector.
	a_ = IntVarArray(*this, sc, 0, 1);
	IntVarArgs a(a_);

	// Create contraints for each resource.
	int ii = 0;
	std::vector<IntArgs> r(cmax_.size(), IntArgs(sc));	
	for (vec_t::const_iterator i = cmax_.begin(); i != cmax_.end(); ++i) 
	{
		for (unsigned int j = 0; j < sc; ++j)
		{
			map_t::const_iterator vi = cm_.find(j);
			if (vi == cm_.end())
				throw;
			r[ii][j] = vi->second[ii];
		}

		linear(*this, r[ii], a, IRT_LQ, *i);
		ii++;
	}
	// Create contraints for each requested info.
	ii = 0;
	std::vector<IntArgs> u(umin_.size(), IntArgs(sc));
	for (vec_t::const_iterator i = umin_.begin(); i != umin_.end(); ++i) 
	{
		for (unsigned int j = 0; j < sc; ++j)
		{
			map_t::const_iterator vi = up_.find(j);
			if (vi == up_.end())
				throw;
			u[ii][j] = vi->second[ii];
		}
		linear(*this, u[ii], a, IRT_GQ, *i);
		ii++;
	}

	branch(*this, a_, INT_VAR_SIZE_MIN, INT_VAL_MIN);

}

gecode_script::~gecode_script() 
{
}

gecode_solver::gecode_solver()
{
}

gecode_solver::~gecode_solver() 
{
}

