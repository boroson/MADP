

/* Only include this header file once. */
#ifndef _DECPOMDPPOLICYITERATION_H_
#define _DECPOMDPPOLICYITERATION_H_ 1

/* the include directives */
#include <iostream>
#include <float.h>
#include "Globals.h"

#include "MDPSolver.h"
#include "TimedAlgorithm.h"

/**\brief DecPOMDPPolicyIteration implements policy iteration for Dec-POMDPs.
  */
class DecPOMDPPolicyIteration : public MDPSolver,
    public TimedAlgorithm
{
private:

    /**_m_QValues represents the non-stationary MDP Q function.
     * I.e. _m_QValues[t][sI][jaI] gives the expected reward at time-step
     * t (time-to-go = horizon - t). */
    QTables _m_QValues;

    /**Is the MDPValueIteration object initialized?.*/
    bool _m_initialized;

    /// Are we solving a finite-horizon problem?
    bool _m_finiteHorizon;

    void Initialize();

    /**Vector<const M*> T is the vector of matrices specifying the transition
       model (one matrix for each joint action). */
    template <class M>
    void Plan(std::vector<const M*> T);


protected:

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    DecPOMDPPolicyIteration(){};

    DecPOMDPPolicyIteration(const PlanningUnitDecPOMDPDiscrete& pu);
    /// Destructor.
    ~DecPOMDPPolicyIteration();

    /// Uses the GetTransitionProbability() interface, which is slow.
    void PlanSlow();


    void Plan();

    void PlanWithCache(bool computeIfNotCached=true);

    void PlanWithCache(const std::string &filenameCache,
                       bool computeIfNotCached=true);


    double GetQ(Index time_step, Index sI,
                Index jaI) const
        { return(_m_QValues[time_step](sI,jaI)); }

    double GetQ(Index sI, Index jaI) const
        { return(_m_QValues[0](sI,jaI)); }

    QTables GetQTables() const;
    QTable GetQTable(Index time_step) const;

    void SetQTables(const QTables &Qs);
    void SetQTable(const QTable &Q, Index time_step);

};


#endif /* !_DECPOMDPVALUEITERATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
