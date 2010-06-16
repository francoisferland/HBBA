#ifndef IW_SOLVER_BASE_HPP
#define IW_SOLVER_BASE_HPP

#include <tr1/unordered_map>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/vector_of.hpp>
#include <string>
#include <vector>
#include <iostream>

namespace iw_solver_interface
{
	template <class T>
	class iw_solver_base
	{
	public:
		typedef std::vector< std::pair< std::string, int > > strat_vec_t;
		typedef std::vector< std::pair< std::string, bool > > sol_vec_t;

		iw_solver_base(): impl_(static_cast<T*>(this)) 
		{
		}

		~iw_solver_base()
		{
		}

		/// \brief Add a resource to the solver's set.
		///
		/// We index resources by name in their order of addition.
		/// Adding a second resource with the same name isn't a good idea,
		/// since the original will no longer be accessible and might break
		/// constraints in the actual solver implementation.
		void add_resource(const std::string& name) 
		{
			update_index(res_map_.left, name);
		}

		/// \brief Sets a resource utilization limit.
		void set_resource_max(const std::string& name, const int max)
		{
			unsigned int i = index(res_map_.left, name);
			impl_->set_resource_max(i, max);	
		}

		/// \brief Add an information to provide to the solver's set.
		///
		/// We index informations by name in their order of addition.
		/// Adding a second information with the same name isn't a good idea,
		/// since the original will no longer be accessible and might break
		/// constraints in the actual solver implementation.
		void add_info(const std::string& name) 
		{
			update_index(info_map_.left, name);
		}

		/// \brief Sets a desired information minimum utility.
		void set_info_min(const std::string& name, const int min)
		{
			unsigned int i = index(info_map_.left, name);
			impl_->set_info_min(i, min);
			/*
			index_map_t::left_map::const_iterator i = info_map_.left.find(name);
			if (i == info_map_.left.end())
			{
				std::cerr << "Invalid info source: " << name << std::endl;
				print_info_set();
			}
			else
				impl_->set_info_min(i->second, min);
			*/

		}

		/// \brief Refresh the model according to the added resources and info
		/// identifiers.
		///
		/// This absolutely needs to be called before changing any model
		/// parameters with set_resource_max and set_info_min or adding
		/// strategies.
		void clear_model()
		{
			impl_->clear_model(res_map_.size(), info_map_.size());
		}

		/// \brief Clears the requirements (info_min) for the solution.
		void clear_reqs()
		{
			impl_->clear_reqs();
		}

		/// \brief Add a strategy to the set.
		///
		/// If a resource or info doesn't exist in the index, it will be added
		/// automatically.
		///
		/// \param c Cost vector.
		/// \param u Info utility effect vector.
		void add_strategy(const std::string& id, const strat_vec_t& c,
			const strat_vec_t& u)
		{
			std::vector<int> cs, us;
			build_indexed_vector(c, cs, res_map_);
			build_indexed_vector(u, us, info_map_);
			clear_model(); // Necessary to propagate new names. 
			unsigned int idx = update_index(strat_map_.left, id);
			impl_->add_strategy(idx, cs, us);

			// TODO: Add a "Bake model" method to fix the internal cost and
			// utility matrices according to the size of the strategies set.
		}

		/// \brief Solves the constraint problem.
		///
		/// \param ref The result will be added to this vector. Each strategy
		/// will be listed as to be activated (true) or not (false). It assumes
		/// the vector has been fully allocated.
		void solve(sol_vec_t& res)
		{
			typedef std::vector<bool> iv_t;
			std::vector<bool> strats(strat_map_.size());
			impl_->solve(strats);
			// TODO: We only keep the first solution for now, will need to add a
			// maximization contraint, ex. max/min CPU.
			index_map_t::right_map::const_iterator i = strat_map_.right.begin();
			for (; i != strat_map_.right.end(); ++i)
				res[i->first] = std::make_pair(i->second, strats[i->first]);
					
		}

		/// \brief Returns the amount of strategies in the set.
		unsigned int strat_count() const { return strat_map_.size(); }

	private:
		typedef boost::bimap< 
			boost::bimaps::unordered_set_of<std::string>, 
			boost::bimaps::vector_of<unsigned int> >
			index_map_t;

		template <class M>
		unsigned int update_index(M& map, const std::string& name)
		{
			unsigned int i = map.size();
			map[name] = i;
			//std::cerr << "Added " << i << ": " << name << std::endl;
			return i;
		}

		/// \brief Retreive the index in a map, create the pair if it isn't
		/// available.
		template <class M>
		int index(M& map, const std::string& name)
		{
			typename M::const_iterator i = map.find(name);
			if (i == map.end())
				return update_index(map, name);
			else
				return i->second;
		}

		/// \brief Builds an indexed vector from a name/value pairs vector.
		///
		/// Updates index when encountering new names.
		void build_indexed_vector(const strat_vec_t& in, std::vector<int>& out,
			index_map_t& index)
		{
			out = std::vector<int>(in.size());
			std::fill(out.begin(), out.end(), 0);
			for (strat_vec_t::const_iterator i = in.begin(); i != in.end(); ++i)
			{
				const std::string& name = i->first;
				const unsigned int& value = i->second;
				unsigned int idx;
				index_map_t::left_map::const_iterator j;
				if ((j = index.left.find(name)) == index.left.end())
					idx = update_index(index.left, name);
				else
					idx = j->second;
				out[idx] = value;
			}
		}

		void print_info_set()
		{
			std::cerr << "Available info sources: " << std::endl;
			index_map_t::left_map::const_iterator i;
			for (i = info_map_.left.begin(); i != info_map_.left.end(); ++i)
				std::cerr << "  " << i->second << ": " << i->first << std::endl;
		}

		T* impl_;

		index_map_t res_map_;
		index_map_t info_map_;
		index_map_t strat_map_;

	};
	
}

#endif

