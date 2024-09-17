#include "cons_benderslp_sahlp.hpp"
#include "scip/pub_cons.h"
#include <algorithm>

// constraint handler to separate benders cuts from fractional solutions
// parameters for this constraint handler that differ from the integral benders constraint handler
#define CONSHDLR_NAME "BenderslpSAHLP"
#define CONSHDLR_DESC "Benders' decomposition for the single assignment hub location problems (fractional solutions)"
#define CONSHDLR_ENFOPRIORITY 1
#define CONSHDLR_CHECKPRIORITY -1000000000

BenderslpSAHLP::BenderslpSAHLP(SCIP *scip)
    : BendersSAHLP(
          scip,
          CONSHDLR_NAME,
          CONSHDLR_DESC,
          CONSHDLR_ENFOPRIORITY,
          CONSHDLR_CHECKPRIORITY)
{
}

SCIP_DECL_CONSENFOLP(BenderslpSAHLP::scip_enfolp)
{
    SCIP_CALL(SCIPstartClock(scip_, callback_clock));
    *result = SCIP_FEASIBLE;

    /**
     * checking whether or not to enforce the LP solution
     * - always at the root node
     * - afterwards at a given frequency
     * - for sub scips only in the root node
     */
    int depth = SCIPgetDepth(scip);
    if (!(depth == 0 || ((lpfreq > 0) && (depth % lpfreq == 0))))
    {
        SCIP_CALL(SCIPstopClock(scip_, callback_clock));
        return SCIP_OKAY;
    }
    if (SCIPgetSubscipDepth(scip) > 0 && SCIPgetDepth(scip) > 0)
    {
        SCIP_CALL(SCIPstopClock(scip_, callback_clock));
        return SCIP_OKAY;
    }

    SCIP_CALL(enforceSolution(NULL, result, FALSE, SCIP_BENDERSENFOTYPE_LP));

    SCIP_CALL(SCIPstopClock(scip_, callback_clock));
    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOPS(BenderslpSAHLP::scip_enfops)
{
    SCIP_CALL(SCIPstartClock(scip_, callback_clock));

    SCIP_CALL(enforceSolution(NULL, result, FALSE, SCIP_BENDERSENFOTYPE_PSEUDO));

    SCIP_CALL(SCIPstopClock(scip_, callback_clock));
    return SCIP_OKAY;
}

SCIP_DECL_CONSCHECK(BenderslpSAHLP::scip_check)
{
    // feasibility of integer solutions is enforced in the integral benders constraint handler
    *result = SCIP_FEASIBLE;
    return SCIP_OKAY;
}

SCIP_DECL_CONSLOCK(BenderslpSAHLP::scip_lock)
{
    // done in the integral benders constraint handlers
    return SCIP_OKAY;
}

SCIP_DECL_CONSINITPRE(BenderslpSAHLP::scip_initpre)
{
    BendersSAHLP *integralbenders = dynamic_cast<BendersSAHLP *>(SCIPfindObjConshdlr(scip, "BendersSAHLP"));
    bendersdata = integralbenders->bendersdata;
    auxvars = integralbenders->auxvars;
    auxobj = integralbenders->auxobj;

    // capture variable, is released when constraint handler exits
    for (auto auxvar : auxvars)
    {
        SCIP_CALL(SCIPcaptureVar(scip, auxvar));
    }
    return SCIP_OKAY;
}

SCIP_RETCODE SCIPincludeConshdlrBenderslpSAHLP(SCIP *scip)
{
    BenderslpSAHLP *newcons = new BenderslpSAHLP(scip);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, newcons, TRUE));

    SCIP_CALL(SCIPaddIntParam(scip, "constraints/" CONSHDLR_NAME "/lpfreq", "frequency for checking LP solution", &(newcons->lpfreq), TRUE, 1, 0, INT_MAX, NULL, NULL));

    return SCIP_OKAY;
}

SCIP_RETCODE SCIPsetupBenderslp(
    SCIP *scip,
    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs)
{
    BenderslpSAHLP *benders = dynamic_cast<BenderslpSAHLP *>(SCIPfindObjConshdlr(scip, CONSHDLR_NAME));

    assert(benders != NULL);
    benders->node_pairs = node_pairs;
    benders->n_subproblems = node_pairs.size();
    benders->subproblemobjvals = std::vector<double>(benders->n_subproblems, 0);

    return SCIP_OKAY;
}