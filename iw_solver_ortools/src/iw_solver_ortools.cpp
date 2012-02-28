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
		
		Scalar get_cost(int j)
		{
			return cs_[j];
		}
	
		Scalar get_utility(int k)
		{
			return us_[k];
		}
		
		int get_id()
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
        typedef S Scalar;
        typedef Strategy<S> StrategyT;

		void set_resource_max(unsigned int i, Scalar max)
		{
			cMax_[i] = max;
		}

		void set_util_min(unsigned int i, Scalar min)
		{
			uMin_[i] = min;
		}

        void set_util_int(unsigned int i, Scalar intensity)
        {
            uInt_[i] = intensity;
        }

		void add_strategy(const unsigned int id, const std::vector<Scalar>& cs,
			const std::vector<Scalar>& us)
		{
			strat_[id] = Strategy<Scalar>(id, cs, us); 
		}

		void clear_model(size_t r, size_t i)
		{	
			std::cout << "Refreshing model..." << std::endl;
			cMax_ = std::vector<Scalar>(r,0);
			uMin_ = std::vector<Scalar>(i,0);
		}

		void clear_reqs()
		{
			std::fill(uMin_.begin(), uMin_.end(), 0);
		}

		void solve(std::vector<bool>& res)
		{
			std::fill(res.begin(), res.end(), false);
            ROS_INFO("Solving ...");  
            StrategySelection(res);
		}
		
		void StrategySelection(std::vector<bool>& res) 
		{
			using namespace operations_research;
			
		 	//Create the parameters 
			int nbResources = cMax_.size(); 
			int nbClass = uMin_.size(); 
			
			int nbStrat = 0; 
			typedef typename map< unsigned int, StrategyT >::const_iterator CI ;
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
			for(int j = 0; j<nbResources; j++) //For each resources
			{ 
				int cMax_j = cMax_[j];
				
				vector<Scalar> c_j;
				typedef typename map<unsigned int, StrategyT >::const_iterator CI ;
				for(CI i = strat_.begin(); i != strat_.end(); ++i)			
				{ 
					Strategy<Scalar> s = i->second;
					c_j.push_back(s.get_cost(j));
				}
				
				solver.AddConstraint(solver.MakeScalProdLessOrEqual(a,c_j, cMax_j));
				objectivesRes_var_array.push_back(solver.MakeScalProd(a,c_j)->Var());
			}

			//Create Utility constraints		
			vector<IntVar*> objectiveUti_var_array; 
			for(int k = 0; k<nbClass; k++) //For each class
			{ 
				int uMin_k = uMin_[k]; 
				vector<Scalar> u_k;
				typedef typename map< unsigned int, StrategyT >::const_iterator CI;
				for(CI i = strat_.begin(); i != strat_.end(); ++i)
				{
					Strategy<Scalar> s = i->second;
					u_k.push_back(s.get_utility(k));
				}

				solver.AddConstraint(solver.MakeScalProdGreaterOrEqual(a,u_k, uMin_k));
				objectiveUti_var_array.push_back(solver.MakeScalProd(a,u_k)->Var());
			}
			/*
			//Utility of each strategy display
			typedef map<unsigned int, Strategy>::const_iterator CI ;
			for(CI i = strat_.begin(); i != strat_.end(); ++i)			
			{ 
				Strategy s = i->second;
				printf("Strategie %d : ", s.get_id()); 
				for (int j = 0 ; j <  nbClass ; j++) 
				{
					printf(" %d ", s.get_utility(j)) ; 
				}
				printf("\n"); 
			}
			*/
			vector<SearchMonitor*> monitors;

			//Optimization 
			vector<OptimizeVar*> objectivesRes;
			for(int j = 0; j<nbResources; j++) //For each resources 
			{  
				objectivesRes.push_back(solver.MakeOptimize(0,objectivesRes_var_array[j], 1));//Optimization of one resource 
				monitors.push_back(objectivesRes[j]);
			}
	
			IntVar* objectiveUti_var = solver.MakeSum(objectiveUti_var_array)->Var(); // Total utility
			OptimizeVar* const objectiveUti = solver.MakeMaximize(objectiveUti_var, 1);//Maximization of total utility
			monitors.push_back(objectiveUti);
			
			
			for(int k = 0; k<nbClass; k++) //For each class
			{ 
				vector<IntVar*> aDes ; //Variables corresponding of the class k 
				typedef typename map< unsigned int, StrategyT >::const_iterator CI;
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
		
			//Searching solutions
			DecisionBuilder* const db = solver.MakePhase(a, Solver::CHOOSE_RANDOM, Solver::ASSIGN_MAX_VALUE);
			SearchMonitor* const log = solver.MakeSearchLog(100000);
			monitors.push_back(log);

			solver.NewSearch(db, monitors);

			while (solver.NextSolution()) 
			{
				printf("Solution found : \n");
				for(int i = 0; i<nbStrat; i++)
				{
					if(a[i]->Value() == 1){res[i] = true; }
					else{res[i] = false;}
					printf(" %3lld \n", a[i]->Value());
				}
			} 
			if (solver.NextSolution() == false) 
			{
				printf("No more solutions found \n");
			}		
	
			solver.EndSearch();
		}
		
	private:
		typename std::vector<Scalar> cMax_;
		typename std::vector<Scalar> uMin_;
        typename std::vector<Scalar> uInt_;
		typename std::map<unsigned int, Strategy<Scalar> > strat_;
	};
}

int main(int argc, char** argv)
{
    using namespace iw_solver_interface;
    using namespace iw_solver_ortools;
    
    typedef SolverNode< ortools_solver<DefaultScalar>, DefaultScalar> node_t;

    ros::init(argc, argv, "ortools_solver");
    node_t n;

    ros::spin();
}


