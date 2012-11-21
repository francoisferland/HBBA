#include "iw_solver_interface/iw_solver_base.hpp"
#include "iw_solver_interface/solver_node.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

#include "base/commandlineflags.h"
#include "base/commandlineflags.h"
#include "base/integral_types.h"
#include "base/logging.h"
#include "base/stringprintf.h"
#include "constraint_solver/constraint_solver.h"

namespace iw_solver_ortools 
{	
    template <class S>
	class Strategy 
	{
	public :
        typedef S Scalar;
	
		Strategy()
		{
		}
		
		Strategy(const unsigned int id, 
            const std::vector<Scalar>& cs, const std::vector<Scalar>& us)
		{
			id_ = id;
			cs_ = cs;
			us_ = us;
		}
		
		Scalar get_cost(int j) const
		{
			return cs_[j];
		}
	
		Scalar get_utility(int k) const
		{
			return us_[k];
		}
		
		int get_id() const
		{
			return id_;
		}
		
	private:  
		int id_;
		typename std::vector<Scalar> cs_;
		typename std::vector<Scalar> us_;

	};

	/// \or-tools solver 
    template <class S>
	class ortools_solver 
	{
	public:
        typedef int64 ORScalar; 
        typedef S Scalar;
        typedef Strategy<Scalar> StrategyT;

        /// \brief Optimization flags
        enum Opts
        {
            OPT_TOTAL_INT_MAX = 1,  // Maximal total intensity
            OPT_TOTAL_UTIL_MAX = 2, // Maximal total utility
            OPT_CLASS_ACT_MAX = 4,  // Maximal activation in each desire class
            OPT_EACH_RES_MAX = 8,   // Maximize resource allocation in each class.
            DEBUG_PRINT = 16,       // Print debug info to stdout.
        };

        ortools_solver(Scalar scaling, int flags): scaling_(scaling), flags_(flags)
        {
        }

		void set_resource_max(unsigned int i, Scalar max)
		{
			cMax_[i] = ORScalar(scaling_ * max);
		}

		void set_util_min(unsigned int i, Scalar min)
		{
			uMin_[i] = ORScalar(scaling_ * min);
		}

        void set_util_int(unsigned int i, Scalar intensity)
        {
            uInt_[i] = ORScalar(scaling_ * intensity);
        }

		void add_strategy(const unsigned int id, const std::vector<Scalar>& cs,
			const std::vector<Scalar>& us)
		{
			strat_[id] = StrategyT(id, cs, us); // Converted and scaled later.
		}

		void clear_model(size_t r, size_t i)
		{	
			std::cout << "Refreshing model..." << std::endl;
			cMax_ = std::vector<ORScalar>(r,0);
			uMin_ = std::vector<ORScalar>(i,0);
            uInt_ = std::vector<ORScalar>(i,0);
		}

		void clear_reqs()
		{
			std::fill(uMin_.begin(), uMin_.end(), 0);
			std::fill(uInt_.begin(), uInt_.end(), 0);
		}

		void solve(std::vector<bool>& res)
		{
			std::fill(res.begin(), res.end(), false);
            ROS_DEBUG("Solving ...");  
            StrategySelection(res);
		}
		
		void StrategySelection(std::vector<bool>& res) 
		{
			using namespace operations_research;
			
		 	//Create the parameters 
			int nbResources = cMax_.size(); 
			int nbClass = uMin_.size(); 
			
			int nbStrat = 0; 
			typedef typename std::map< unsigned int, StrategyT >::const_iterator CI ;
			for(CI i = strat_.begin(); i != strat_.end(); ++i)			
			{ 
				nbStrat++;
			}
			
			//Create the Solver
			Solver solver("strategyselection"); 

			//Create the variables
			vector<IntVar*> a;
			solver.MakeIntVarArray(nbStrat, 0, 1, &a);

			//Create Resource constraints
			vector<IntVar*> objectivesRes_var_array;		
            if (flags_ & DEBUG_PRINT)
                printf("c_ji:\n");
			for(int j = 0; j<nbResources; j++) //For each resources
			{ 
				int cMax_j = cMax_[j];
				
				vector<int64> c_j;
				typedef typename std::map<unsigned int, StrategyT >::const_iterator CI ;
				for(CI i = strat_.begin(); i != strat_.end(); ++i)			
				{ 
					StrategyT s = i->second;
					c_j.push_back(ORScalar(scaling_ * s.get_cost(j)));
                    if (flags_ & DEBUG_PRINT)
                        printf("%i ", (int)(scaling_ * s.get_cost(j)));
				}
                if (flags_ & DEBUG_PRINT)
                    printf("| max: %i\n", (int)cMax_j);
				
				solver.AddConstraint(solver.MakeScalProdLessOrEqual(a,c_j, cMax_j));
				objectivesRes_var_array.push_back(solver.MakeScalProd(a,c_j)->Var());
			}

			//Create Utility constraints		
			vector<IntVar*> objectiveUti_var_array; 
			vector<IntVar*> objectiveInt_var_array; 
            if (flags_ & DEBUG_PRINT)
                printf("u_ki:\n");
			for(int k = 0; k<nbClass; k++) //For each class
			{ 
				int uMin_k = uMin_[k]; 
                ORScalar s_k = uInt_[k]; 

				vector<ORScalar> u_k;
                vector<ORScalar> u_k_s_k;
				typedef typename std::map< unsigned int, StrategyT >::const_iterator CI;
				for(CI i = strat_.begin(); i != strat_.end(); ++i)
				{
					StrategyT s = i->second;
                    ORScalar u_ik = ORScalar(scaling_ * s.get_utility(k)); 
					u_k.push_back(u_ik);
                    u_k_s_k.push_back(u_ik * s_k);
                    if (flags_ & DEBUG_PRINT)
                        printf("%i ", (int)u_ik);
				}
                if (flags_ & DEBUG_PRINT)
                    printf(" | min: %i \n", (int)uMin_k);

				solver.AddConstraint(solver.MakeScalProdGreaterOrEqual(a,u_k, uMin_k));
				objectiveUti_var_array.push_back(solver.MakeScalProd(a,u_k)->Var());
                objectiveInt_var_array.push_back(solver.MakeScalProd(a, u_k_s_k)->Var());
			}
			
			vector<SearchMonitor*> monitors;

			//Optimization 
            
            if (flags_ & OPT_EACH_RES_MAX)
            {
                vector<OptimizeVar*> objectivesRes;
                for(int j = 0; j<nbResources; j++) //For each resources 
                {  
                    objectivesRes.push_back(solver.MakeOptimize(0,objectivesRes_var_array[j], 1));//Optimization of one resource 
                    monitors.push_back(objectivesRes[j]);
                }
	        }

            if (flags_ & OPT_TOTAL_UTIL_MAX)
            {
                IntVar* objectiveUti_var = solver.MakeSum(objectiveUti_var_array)->Var(); // Total utility
                OptimizeVar* const objectiveUti = solver.MakeMaximize(objectiveUti_var, 1);//Maximization of total utility
                    monitors.push_back(objectiveUti);
            }

            // Intensity maximization.
            if (flags_ & OPT_TOTAL_INT_MAX)
            {
                IntVar* objectiveInt_var = solver.MakeSum(
                    objectiveInt_var_array)->Var();
                OptimizeVar* const objectiveInt = solver.MakeMaximize(objectiveInt_var, 1);
                monitors.push_back(objectiveInt);
            }
			
            if (flags_ & OPT_CLASS_ACT_MAX)
            {
                for(int k = 0; k<nbClass; k++) //For each class
                { 
                    vector<IntVar*> aDes ; //Variables corresponding of the class k 
                    typedef typename std::map< unsigned int, StrategyT >::const_iterator CI;
                    for(CI i = strat_.begin(); i != strat_.end(); ++i)
                    {
                        StrategyT s = i->second;
                        if (s.get_utility(k) > 0)
                        {
                            aDes.push_back(a[s.get_id()]);
                        }
                    }
                    //Maximization of activated strategies for each class
                    IntVar* objectiveDes_var = solver.MakeSum(aDes)->Var() ; 
                    OptimizeVar* const objectiveDes = solver.MakeMaximize(objectiveDes_var, 1);
                    monitors.push_back(objectiveDes);
                }
		    }

			//Searching solutions
			DecisionBuilder* const db = solver.MakePhase(a, Solver::CHOOSE_RANDOM, Solver::ASSIGN_MAX_VALUE);

            //if (flags_ & DEBUG_PRINT)
            //{
            //    SearchMonitor* const log = solver.MakeSearchLog(100000);
            //    monitors.push_back(log);
            //}

			solver.NewSearch(db, monitors);

            int nbSolutions = 0;
			while (solver.NextSolution()) 
			{
                ++nbSolutions;
				for(int i = 0; i<nbStrat; i++)
				{
					if(a[i]->Value() == 1){res[i] = true; }
					else{res[i] = false;}
                    //if (flags_ & DEBUG_PRINT)
                    //    printf(" %3lld \n", a[i]->Value());
				}
			} 

            if (flags_ & DEBUG_PRINT)
                printf("Solution(s) found : %i \n", nbSolutions);

			//if (solver.NextSolution() == false) 
			//{
                //if (flags_ & DEBUG_PRINT)
                //    printf("No more solutions found \n");
			//}		
	
			solver.EndSearch();
		}
		
	private:
		typename std::vector<ORScalar> cMax_;
		typename std::vector<ORScalar> uMin_;
        typename std::vector<ORScalar> uInt_;
		typename std::map<unsigned int, StrategyT > strat_;
        Scalar scaling_; // Conversion ratio when goint to int64.
        int flags_;     // Optimization flags.
	};
}

int main(int argc, char** argv)
{
    using namespace iw_solver_interface;
    using namespace iw_solver_ortools;
    typedef ortools_solver<DefaultScalar> SolverT;
    typedef SolverNode<SolverT, DefaultScalar> NodeT; 

    ros::init(argc, argv, "ortools_solver");
    ros::NodeHandle np("~");

    double scaling;
    np.param("scaling_factor", scaling, 1000.0); 
    int flags = 0;
    bool b;
    np.param("optimize_intensity", b, false);
    if (b)
        flags |= SolverT::OPT_TOTAL_INT_MAX;
    np.param("optimize_utility", b, false);
    if (b)
        flags |= SolverT::OPT_TOTAL_UTIL_MAX;
    np.param("debug_print", b, false);
    if (b)
        flags |= SolverT::DEBUG_PRINT;
    
    boost::shared_ptr<SolverT> solver(new SolverT(scaling, flags));
    NodeT node(solver);

    ros::spin();
}


