#ifndef IW_SOLVER_BASE_HPP
#define IW_SOLVER_BASE_HPP

#include <tr1/unordered_map>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/vector_of.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/multi_array.hpp>
#include <string>
#include <vector>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>

namespace iw_solver_interface
{
    /// \brief A base interface class for IW constraint solvers.
    ///
    /// This class acts as a wrapper for a low-level constraint solver.
    /// The aim is to provide a single interface to other classes that would
    /// like to access to an IW constraint solver.
    /// A client template example is available in the SolverNode class.
    ///
    /// This wrapper manages mapping named resources to dense variable vectors
    /// so that the low-level solver doesn't have to be aware of resources/costs
    /// tags and such.
    ///
    /// The client side has to be aware of the impact of adding strategies or 
    /// setting resources and costs min/max values on the ordering of variables.
    /// For instance, adding a strategy might reorder the whole costs and 
    /// utilities vectors and clear all previously entered data.
    /// The usual client usage should be this:
    ///  - Add all strategies.
    ///  - Set resources maximum costs.
    ///  - Wait for an incoming desires set.
    ///  - Clear the requirements and set them to the current desires.
    ///  - Solve - send strategies, max costs and min utilities to the low-level
    ///    solver if anything changed since the last solve call.
    ///  - Return to the waiting step.
    ///
    /// For better efficiency with the current implementation, you shouldn't
    /// modify the strategies set more than once per session.
    ///
    /// Your T class needs to provide a few methods to make it work:
    ///  - set_resource_max(int id, int v)
    ///    Set the resource at index id to maximum cost v.
    ///  - set_util_min(int id, int v)
    ///    Set the information at index id to minimum utility v.
    ///  - set_util_int(int id, int i)
    ///    Set the information at index id to intensity i.
    ///  - add_strategy(int id, std::vector<int> c, std::vector<int> u)
    ///  - Add a strategy at index id with costs c and utilities u.
    ///  - solve(std::vector<bool>& result)
    ///    Solve for the given constraints, set the strategies' activation 
    ///    states in result.
    ///  - clear_model(int rs, int is)
    ///    Clear constraints and set resource and information vectors to size rs 
    ///    and is. Needs to be done before solving if new resources or 
    ///    information have been added. Currently called after adding a
    ///    strategy. Will clear any previous data inside the vectors.
    ///  - clear_reqs()
    ///    Clear information utility requirements.
    ///
    template <class T, class S>
    class iw_solver_base
    {
    public:
        /// \brief Scalar type used for values (cost, utility, intensity).
        typedef S Scalar;
        typedef std::pair<std::string, Scalar> ScalarTag;
        /// \brief Vector type for cost/utility tag/value pairs.
        typedef std::vector<ScalarTag> strat_vec_t;
        /// \brief Type for strategy activation tag/value pairs.
        typedef std::pair<std::string, bool> sol_t;
        /// \brief Vector type for strategy activation tag/value pairs.
        typedef std::vector<sol_t> sol_vec_t;
        /// \brief Vector type for ordered cost and utility values.
        typedef std::vector<Scalar> costs_t;

        iw_solver_base(const boost::shared_ptr<T>& impl): impl_(impl)
        {
        }

        iw_solver_base(): impl_(new T()) 
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
        void set_resource_max(const std::string& name, const Scalar max)
        {
            unsigned int i = index(res_map_.left, name);
            if (i >= cmax_.size())
                cmax_.resize(i+1);
            cmax_[i] = max;
            res_updated_ = true;
        }

        /// \brief Add an information to provide to the solver's set.
        ///
        /// We index informations by name in their order of addition.
        /// Adding a second information with the same name isn't a good idea,
        /// since the original will no longer be accessible and might break
        /// constraints in the actual solver implementation.
        void add_util(const std::string& name) 
        {
            update_index(util_map_.left, name);
        }

        /// \brief Sets a desired information minimum utility.
        void set_util_min(const std::string& name, const Scalar min)
        {
            unsigned int i = index(util_map_.left, name);
            if (i >= umin_.size())
            {
                umin_.resize(i + 1);
                model_updated_ = true;
            }
            umin_[i] = min;
            umin_updated_ = true;

        }

        /// \brief Sets a desired information intensity.
        void set_util_int(const std::string& name, const Scalar intensity)
        {
            unsigned int i = index(util_map_.left, name);
            if (i >= uint_.size())
            {
                uint_.resize(i + 1);
                model_updated_ = true;
            }
            uint_[i] = intensity;
            uint_updated_ = true;
        }
        
        /// \brief Refresh the model according to the added resources and info
        /// identifiers.
        ///
        /// This absolutely needs to be called before changing any model
        /// parameters with set_resource_max and set_util_min or adding
        /// strategies.
        void clear_model()
        {
            impl_->clear_model(res_map_.size(), util_map_.size());
        }

        /// \brief Clears the requirements (util_min, util_int) for the 
        /// solution.
        void clear_reqs()
        {
            impl_->clear_reqs();
            umin_.resize(0);
            uint_.resize(0);
            umin_updated_ = true;
            uint_updated_ = true;
        }

        /// \brief Add a strategy to the set.
        ///
        /// If a resource or util doesn't exist in the index, it will be added
        /// automatically.
        ///
        /// \param c Cost vector.
        /// \param u utility effect vector.
        void add_strategy(const std::string& id, const strat_vec_t& c,
            const strat_vec_t& u, const strat_vec_t& u_min)
        {
            //  - Add costs and utilities in both matrices:
            //    - Look for a cost/utility name in the map.
            //    - Resize the matrix if it's a new value. 
            //  - Flag model_updated_.
            update_matrix(cost_mtx_, res_map_, id, c);
            update_matrix(util_mtx_, util_map_, id, u);
            update_matrix(u_min_mtx_, util_map_, id, u_min);
            model_updated_ = true;

        }

        /// \brief Solves the constraint problem.
        ///
        /// \param ref The result will be added to this vector. Each strategy
        /// will be listed as to be activated (true) or not (false). It assumes
        /// the vector has been fully allocated.
        void solve(sol_vec_t& res)
        {
            if (model_updated_)
            {
                // Clear model and send the strategy set to the low-level 
                // solver.
                clear_model();
                for (size_t i = 0; i < cost_mtx_.shape()[0]; ++i)
                {
                    // TODO: Remove this vector copy and find a way to template 
                    // the add_strategy method in the implementation to accept
                    // anything that will respond to std::copy.
                    costs_t cs(cost_mtx_.shape()[1]);
                    costs_t us(util_mtx_.shape()[1]);
                    costs_t u_min(u_min_mtx_.shape()[1]);
                    std::copy(cost_mtx_[i].begin(), cost_mtx_[i].end(), 
                        cs.begin());
                    std::copy(util_mtx_[i].begin(), util_mtx_[i].end(),
                        us.begin());
                    std::copy(u_min_mtx_[i].begin(), u_min_mtx_[i].end(), 
                        u_min.begin());

                    costs_t uf(util_mtx_.shape()[1]);   
                    //printf("Utilité finale strat %d  ",i);
                    for(size_t j =0; j<us.size(); ++j)
                    {
                        if (j < u_min.size())
                            uf[j] = us[j] - u_min[j];
                        else
                            uf[j] = us[j];
                        //printf(" %d ", uf[j]);
                    }
                   printf("\n");
                    
                    impl_->add_strategy(i, cs, uf);
                }
                model_updated_ = false;
            }

            if (umin_updated_)
            {
                for (unsigned int i = 0; i < umin_.size(); ++i)
                    impl_->set_util_min(i, umin_[i]);
                umin_updated_ = false;
            }
            if (uint_updated_)
            {
                for (unsigned int i = 0; i < uint_.size(); ++i)
                    impl_->set_util_int(i, uint_[i]);
                uint_updated_ = false;
            }
            if (res_updated_)
            {
                for (unsigned int i = 0; i < cmax_.size(); ++i)
                    impl_->set_resource_max(i, cmax_[i]);
                res_updated_ = false;
            }

            typedef std::vector<bool> iv_t;
            std::vector<bool> strats(strat_map_.size());
            impl_->solve(strats);
            // We only keep the first solution for now, optimizations are
            // currently fixed in the lower level solver.
            index_map_t::right_map::const_iterator i = strat_map_.right.begin();
            for (; i != strat_map_.right.end(); ++i)
                res[i->first] = std::make_pair(i->second, strats[i->first]);
                    
        }

        /// \brief Returns the amount of strategies in the set.
        unsigned int strat_count() const { return strat_map_.size(); }

    private:
        /// \brief Bidirectional map type for tag/index retrieval. 
        typedef boost::bimap< 
            boost::bimaps::unordered_set_of<std::string>, 
            boost::bimaps::vector_of<unsigned int> >
            index_map_t;
        /// \brief Matrix type for costs and utilities. One strategy per row.
        typedef boost::multi_array<int, 2> matrix_t;

        /// \brief Take an input tag-indexed vector and add it to the 
        /// corresponding matrix.
        ///
        /// \param mtx The matrix to update.
        /// \param map The index map to use to retrieve column indices. 
        /// \param strat_name The name of the strategy.
        /// \param in The input vector.
        void update_matrix(matrix_t& mtx, index_map_t& map, 
            const std::string& strat_name, const strat_vec_t& in)
        {
            size_t i = index(strat_map_.left, strat_name);
            // If the strategy's index is higher than the number of rows in 
            // the matrix, we need to resize it to accommodate the new strategy.
            if (i >= mtx.shape()[0])
                mtx.resize(boost::extents[i + 1][mtx.shape()[1]]);

            typedef typename strat_vec_t::const_iterator jj_t;
            for (jj_t jj = in.begin(); jj != in.end(); ++jj)
            {
                size_t j = index(map.left, jj->first);
                // If jj is greater than the matrix column count, we need to
                // resize it to accommodate the newest resource or utility 
                // class. 
                if (j >= mtx.shape()[1])
                    mtx.resize(boost::extents[mtx.shape()[0]][j + 1]);

                mtx[i][j] = jj->second;
            }
        }

        /// \brief Add a new tag to an tag-index map, return the index.
        ///
        /// Warning: Does not check for existing values.
        /// Use index() to check for existing values, which will call this
        /// function if it doesn't exist in the map.
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

        void print_util_set()
        {
            std::cerr << "Available util sources: " << std::endl;
            index_map_t::left_map::const_iterator i;
            for (i = util_map_.left.begin(); i != util_map_.left.end(); ++i)
                std::cerr << "  " << i->second << ": " << i->first << std::endl;
        }

        boost::shared_ptr<T> impl_;

        index_map_t res_map_;
        index_map_t util_map_;
        index_map_t strat_map_;

        // Model and requirements data.
        // We're currently keeping 2-3 copies of the same data since we force
        // the use of vectors in the low-level solver calls.
        // If the low-level class is templated, it should be able to do a single
        // std::copy call from these vectors or matrices to the specific vector
        // types used by the solver library.
        
        matrix_t util_mtx_;
        matrix_t u_min_mtx_;
        matrix_t cost_mtx_;
        bool model_updated_;

        costs_t cmax_;
        bool res_updated_;
        costs_t umin_;
        bool umin_updated_;
        costs_t uint_;
        bool uint_updated_;
        
    };
    
}

#endif

