// #define SCIP_DEBUG
#include "prop_partialenum.hpp"
#include "probdata_sahlp.hpp"
#include "scip/scip_probing.h"
#include <vector>

#define PROP_NAME "partialenum"
#define PROP_DESC "partial enumeration of the hub opening decisions"
#define PROP_PRIORITY 1000000
#define PROP_FREQ 0 // only run at root node
#define PROP_DELAY 0
#define PROP_TIMING SCIP_PROPTIMING_DURINGLPLOOP
// we do not partake in presolving but still need to define these properties for
// the ObjProp constructor
#define PROP_PRESOLPRIORITY 0
#define PROP_PRESOLMAXROUNDS 0
#define PROP_PRESOLTIMING SCIP_PRESOLTIMING_FAST

using std::vector;

PropPartialEnum::PropPartialEnum(SCIP *scip)
    : ObjProp(scip, PROP_NAME, PROP_DESC, PROP_PRIORITY, PROP_FREQ, PROP_DELAY,
              PROP_TIMING, PROP_PRESOLPRIORITY, PROP_PRESOLMAXROUNDS,
              PROP_PRESOLTIMING) {}

SCIP_DECL_PROPEXEC(PropPartialEnum::scip_exec)
{
    double maxgap;
    SCIP_CALL(
        SCIPgetRealParam(scip_, "propagating/" PROP_NAME "/maxgap", &maxgap));
    *result = SCIP_DIDNOTRUN;

    // check if we want to run the propagator
    bool no_upper_bound = SCIPisInfinity(scip_, SCIPgetUpperbound(scip_));
    bool LP_not_solved_optimially =
        SCIPgetLPSolstat(scip_) != SCIP_LPSOLSTAT_OPTIMAL;
    bool gap_too_large = SCIPgetGap(scip_) > maxgap;
    bool gap_not_improved_enough = SCIPgetGap(scip_) >= 0.99 * last_gap;

    if (no_upper_bound || SCIPinProbing(scip_) || LP_not_solved_optimially ||
        gap_too_large || gap_not_improved_enough)
        return SCIP_OKAY;

    *result = SCIP_DIDNOTFIND;
    last_gap = SCIPgetGap(scip_);

    // always try to fix variables to zero
    SCIP_CALL(SCIPstartProbing(scip_));
    SCIP_CALL(enumerateHubs(true, result));

    // only try to fix variables to one after the LP node was fully solved
    if (proptiming == SCIP_PROPTIMING_AFTERLPLOOP)
    {
        SCIP_CALL(enumerateHubs(false, result));
    }
    SCIP_CALL(SCIPendProbing(scip_));
    return SCIP_OKAY;
}

SCIP_RETCODE PropPartialEnum::enumerateHubs(bool to_zero, SCIP_RESULT *result)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    vector<SCIP_VAR *> vars_to_probe;
    vector<SCIP_Var *> vars_to_fix;
    double timelimit;
    SCIP_CALL(SCIPgetRealParam(scip_, "limits/time", &timelimit));

    // only look at variables that are close to their bounds
    for (auto hub : probdata->still_possible_hubs)
    {
        SCIP_VAR *var = probdata->open_hub_vars[hub];
        // variable might have been fixed somewhere else in the meantime, in this
        // case we don't need to look at it only look at variables that are close to
        // their bounds
        if (SCIPvarGetUbGlobal(var) > 0.5 && SCIPvarGetLbGlobal(var) < 0.5 &&
            ((to_zero && SCIPgetSolVal(scip_, NULL, var) < 0.2) ||
             (!to_zero && SCIPgetSolVal(scip_, NULL, var) > 0.8)))
        {
            vars_to_probe.push_back(var);
        }
    }

    for (auto var : vars_to_probe)
    {

        if (to_zero)
        {
            // variable might have been fixed in the meanwhile due to conflict
            // analysis or similar
            if ((SCIPvarGetUbLocal(var) < 0.5) || (SCIPvarGetLbLocal(var) > 0.5))
                continue;
            // try fixing the variable to one, if objective value increases above
            // primal bound, then we can fix the variable to zero globally
            SCIP_CALL(SCIPnewProbingNode(scip_));
            SCIP_CALL(SCIPchgVarLbProbing(scip_, var, 1.0));
        }
        else if (!to_zero)
        {
            if ((SCIPvarGetLbLocal(var) > 0.5) || (SCIPvarGetUbLocal(var) < 0.5))
                continue;
            // try fixing the variable to zero, if objective value increases above
            // primal bound, then we can fix the variable to one globally
            SCIP_CALL(SCIPnewProbingNode(scip_));
            SCIP_CALL(SCIPchgVarUbProbing(scip_, var, 0.0));
        }
        else
        {
            continue;
        }

        SCIP_Bool error, cutoff;

        SCIP_CALL(SCIPsolveProbingLP(scip_, 10000000, &error, &cutoff));
        assert(!error);

        double lb = SCIPgetLPObjval(scip_);

        if (cutoff || SCIPisGT(scip_, lb, SCIPgetPrimalbound(scip_)))
        {
            SCIPdebugMsg(scip_, "fixing %s to %d \n", SCIPvarGetName(var),
                         to_zero ? 0 : 1);
            *result = SCIP_REDUCEDDOM;
            vars_to_fix.push_back(var);
        }
        // reset the probing bounds
        SCIP_CALL(SCIPbacktrackProbing(scip_, 0));

        // check if we are running out of time, in that case stop solving probing
        // nodes
        if (SCIPisStopped(scip_) || SCIPgetSolvingTime(scip_) > timelimit - 10)
            break;
    }

    // confirm that global bound was changed (should be done automatically by
    // SCIPs conflict analysis, but sometimes this didn't happen)
    SCIP_Bool infeasible, fixed;
    for (auto var : vars_to_fix)
    {
        if (to_zero)
        {
            if (!(SCIPvarGetUbGlobal(var) == 0.0))
            {
                SCIPfixVar(scip_, var, 0.0, &infeasible, &fixed);
                assert(fixed);
                assert(!infeasible);
            }
        }
        else
        {
            if (!(SCIPvarGetLbGlobal(var) == 1.0))
            {
                SCIPfixVar(scip_, var, 1.0, &infeasible, &fixed);
                assert(fixed);
                assert(!infeasible);
            }
        }
    }

    return SCIP_OKAY;
}

SCIP_RETCODE SCIPincludeObjPropPartialEnum(SCIP *scip)
{
    SCIP_CALL(SCIPincludeObjProp(scip, new PropPartialEnum(scip), TRUE));

    SCIP_CALL(SCIPaddRealParam(
        scip, "propagating/" PROP_NAME "/maxgap",
        "maximum relative gap to allow before running the propagator", NULL,
        FALSE, 0.01, 0.0, 1.0, NULL, NULL));
    return SCIP_OKAY;
}