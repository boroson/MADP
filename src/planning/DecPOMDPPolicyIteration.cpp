/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Bas Terwijn
 * Xuanjie Liu
 *
 * For contact information please see the included AUTHORS file.
 */


#include <fstream>
#include <time.h>
#include "directories.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "TransitionModelMapping.h"
#include "TransitionModelMappingSparse.h"
#include "DecPOMDPPolicyIteration.h"
//#include "SystemOfLinearEquationsSolver.h"

using namespace std;
#define DEBUG_DecPOMDPPolicyIteration 0


DecPOMDPPolicyIteration::DecPOMDPPolicyIteration(const PlanningUnitDecPOMDPDiscrete& pu) :
    MDPSolver(pu)
{
    _m_initialized = false;
}


DecPOMDPPolicyIteration::~DecPOMDPPolicyIteration()
{
}

void DecPOMDPPolicyIteration::Initialize()
{
    StartTimer("Initialize");


    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();
    size_t nrA = GetPU()->GetNrAgents();

    size_t nrQfunctions;
    if(horizon==MAXHORIZON)
    {
        _m_finiteHorizon=false;
        nrQfunctions=1;
    }
    else
    {
        _m_finiteHorizon=true;
        nrQfunctions=horizon;
    }

    QTable tempTable(nrS,nrJA);
    for(unsigned int s=0;s!=nrS;++s)
        for(unsigned int ja=0;ja!=nrJA;++ja)
            tempTable(s,ja)=0;

    for(Index t=0; t < nrQfunctions; t++)
        _m_QValues.push_back(tempTable);

    _m_initialized = true;


    StopTimer("Initialize");
}

QTables DecPOMDPPolicyIteration::GetQTables() const
{
    return(_m_QValues);
}

QTable DecPOMDPPolicyIteration::GetQTable(Index time_step) const
{
    return(_m_QValues.at(time_step));
}

void DecPOMDPPolicyIteration::SetQTables(const QTables &Qs)
{
    _m_QValues=Qs;
}

void DecPOMDPPolicyIteration::SetQTable(const QTable &Q, Index time_step)
{
    _m_QValues[time_step]=Q;
}

void DecPOMDPPolicyIteration::Plan()
{
    PlanSlow();
}

void DecPOMDPPolicyIteration::PlanWithCache(bool computeIfNotCached){}

void DecPOMDPPolicyIteration::PlanWithCache(const string &filenameCache,
        bool computeIfNotCached){}





void DecPOMDPPolicyIteration::PlanSlow()
{
	printf("Initialize the problem\n");
    if(!_m_initialized)
        Initialize();

    StartTimer("Plan");

    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();
    
    // For now, nodes and observations are the same, so number of nodes 
    // in FSC is number of observations
    // Also assuming that all agents have same observation/action space
    // Probably should fix this at some point
    size_t nrQ = GetPU()->GetNrObservations();
    size_t nrN = GetPU()->GetNrAgents();
    size_t nrA = GetPU()->GetNrActions();
    
    // Q-value table for states

    double R_i,R_f,maxQsuc;

    // cache immediate reward for speed
    QTable immReward(nrS,nrJA);
    printf("Get immediate reward...");
    for(Index sI = 0; sI < nrS; sI++){
        for(Index jaI = 0; jaI < nrJA; jaI++){
            immReward(sI,jaI)=GetPU()->GetReward(sI, jaI);
        }
    }
    printf("Done.\n");



    if(_m_finiteHorizon)
    {
        throw(E("Sorry this function does not support finite horizon now"));
    }
    else // infinite horizon problem
    {
        double maxDelta=DBL_MAX;
        double gamma=GetPU()->GetDiscount();


        // in infinite-horizon case, it is typically worth to cache
        // the transition model
        typedef boost::numeric::ublas::compressed_matrix<double> CMatrix;
        vector<CMatrix*> T;
        CMatrix *Ta;
        double p;
        printf("Get transition model...");
        for(unsigned int a=0;a!=nrJA;++a)
        {
        	//printf("a%d:\n",a);
#if DEBUG_DecPOMDPPolicyIteration
            PrintTimersSummary();
#endif
            StartTimer("CacheTransitionModel");
            Ta=new CMatrix(nrS,nrS);

            for(unsigned int s=0;s!=nrS;++s)
            {
                for(unsigned int s1=0;s1!=nrS;++s1)
                {
                    p=GetPU()->GetTransitionProbability(s,a,s1);

                    (*Ta)(s,s1)=p;
                    double sss = (*Ta)(s,s1);
                }
            }

            T.push_back(Ta);
            StopTimer("CacheTransitionModel");
        }
        printf("Done.\n");

        //Initialize state/node values for each agent
        // V(s,q,n)
        //double *SV = (double *)malloc(sizeof(double)*nrS);
        double SV[nrS][nrQ][nrN]
        for(int i = 0; i < nrS; i++)
        {
			for(int j = 0; j < nrQ; j++)
			{
				for(int n = 0; n < nrN; n++)
					SV[i][j][n] = 0;
			}
		}

        //Initialize a random policy for each agent
        //vector<int> policy;
        int policy[nrQ][nrN];
        srand(time(NULL));
        for(int qI = 0; qI != nrQ; ++qI)
        {
			for(int n = 0; n < nrN; n++)
			{
				int aI;
				aI = (rand() % nrA);
				policy[qI][n] = aI;
			}
        }

        //Initialize policy-statable flag
        bool psf = 0;
        const double THETA = 0.0001;
        double delta;
        //Set a small positive number as number flag to prevent from 0.8273 > 0.8273 due to a 0.0000001 error. If a - b > NUM_FLAG, we think that a > b.
        const double NUM_FLAG = 0.001;

        //Set timer.
        int num_iter = 0;
        time_t eva_start, eva_end, imp_end;
        double time_eva, time_imp, time_iter,time_total;
        double total_eva = 0;
        double total_imp = 0;

        //Policy iteration
        while(psf == 0){
        	//policy evaluation
        	num_iter += 1;
        	eva_start = clock();
        	do
        	{
        		delta = 0;
        		for(int s1I = 0; s1I != nrS; ++s1I)
        		{
					for(int qJ = 0; qJ < nrQ; qJ++)
					{
						for(int n = 0; n < nrN; n++)
						{
							double v = SV[s1I][qj][n];
							int actionIndex = GetJointAction(s1I);//policy[qJ][n];
							double new_v = 0;
							//TODO: Get joint action for this state
							double r = immReward(s1I, actionIndex);
							for(int s2I = 0; s2I !=nrS; ++s2I)
							{
								double tp = (*T[actionIndex])(s1I,s2I);
								if(tp != 0)
								{
									//TODO: Get node/observation from state transition
									q2 = GetNodeFromStateTransition(s1I, qJ, s2I);
									new_v += tp * (r + gamma * SV[s2I][q2][n]);
								}
							}
							SV[s1I] = new_v;
							delta = std::max(delta, std::abs(v - SV[s1I]));
						}
					}
        		}

        	}while(delta > THETA);


        	eva_end = clock();
        	//Policy improvement
        	psf = 1;
        	for(int s1I = 0; s1I != nrS; ++s1I)
        	{
				for(int qJ = 0; qJ < nrQ; qJ++)
				{
					for(int n = 0; n < nrN; n++)
					{
						double ba = SV[s1I][qJ][n];
						for(int aI = 0; aI != nrA; ++aI)
						{
							double qa = 0;
							//TODO: get joint action from individual actions
							int ja = GetNewJointAction(s1I, n, aI);//policy[qJ][n];
							double r = immReward(s1I, aI);
							
							for(int s2I = 0; s2I != nrS; ++s2I)
							{
								double tp = (*T[aI])(s1I,s2I);
								if(tp != 0)
								{
									//TODO: Get node/observation from state transition
									q2 = GetNodeFromStateTransition(s1I, qJ, s2I);
									qa += tp * (r + gamma * SV[s2I][q2][n]);
								}
							}
							if(qa - ba > NUM_FLAG)
							{
								psf = 0;
								ba = qa;
								policy[qJ][n] = aI;
							}

						}
					}
				}
        	}

        	imp_end = clock();
        	time_eva = (double)(eva_end - eva_start)/CLOCKS_PER_SEC;
        	time_imp = (double)(imp_end - eva_end)/CLOCKS_PER_SEC;
        	time_iter = time_eva + time_imp;
        	total_eva += time_eva;
        	total_imp += time_imp;
        	printf("Iter:%d %f %f %f\n",num_iter, time_eva, time_imp, time_iter);

        }
        time_total = total_eva + total_imp;
        printf("Normal policy iteration completed...\n");
        for(int i=0; i<nrS; i++)
        {
        	printf("State %d: %f, %d  ", i, SV[i], policy[i]);
        }
        printf("\n");
        printf("States: %d  Actions: %d\n",nrS, nrJA);
        printf("Total policy iteration time: %f\nTotal policy evaluation time: %f\nTotal policy improvement time: %f\nNumber of iterations: %d\n", time_total, total_eva, total_imp,num_iter);

        for(unsigned int a=0;a!=nrJA;++a)
            delete T[a];
    }

    StopTimer("Plan");

#if DEBUG_DecPOMDPPolicyIteration
    PrintTimersSummary();
#endif
}
