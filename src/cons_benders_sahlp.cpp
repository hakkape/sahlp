// #define SCIP_DEBUG
// #define WRITE_SUBPROBLEM_W_NUMERICAL_PROBLEMS
#include "cons_benders_sahlp.hpp"
#include "lpi/lpi.h"
#include "matheuristic_sahlp.hpp"
#include "scip/cons_linear.h"
#include "scip/heur_trysol.h"
#include "scip/pub_cons.h"
#include "scip/type_benders.h"
#include <algorithm>
#include <functional>
#include <string>
#ifdef WITHCPLEX
#include <ilcplex/cplex.h>
#endif
#ifdef WITHMCF
#include "MCFSimplex.h"
#endif

#define CONSHDLR_NAME "BendersSAHLP"
#define CONSHDLR_DESC "Benders' decomposition for the single assignment hub location problems (integer solutions)"
#define CONSHDLR_SEPAPRIORITY 0    // we do not separate LP solutions
#define CONSHDLR_ENFOPRIORITY -100 // priority < 0: we only enforce integer solutions
#define CONSHDLR_CHECKPRIORITY -5000000
#define CONSHDLR_SEPAFREQ 1
#define CONSHDLR_PROPFREQ -1
#define CONSHDLR_EAGERFREQ 1
#define CONSHDLR_MAXPREROUNDS 0
#define CONSHDLR_DELAYSEPA FALSE
#define CONSDHLR_DELAYPROP FALSE
#define CONSHDLR_NEEDSCONS FALSE                     // including this constraint handler implicitly activates it
#define CONSHDLR_PROPTIMING SCIP_PROPTIMING_BEFORELP // we do not partake in propagation
#define CONSHDLR_PRESOLTIMING SCIP_PRESOLTIMING_FAST // we do not partake in presolving

using std::vector;

BendersSAHLP::BendersSAHLP(SCIP *scip)
    : ObjConshdlr(
          scip,
          CONSHDLR_NAME,
          CONSHDLR_DESC,
          CONSHDLR_SEPAPRIORITY,
          CONSHDLR_ENFOPRIORITY,
          CONSHDLR_CHECKPRIORITY,
          CONSHDLR_SEPAFREQ,
          CONSHDLR_PROPFREQ,
          CONSHDLR_EAGERFREQ,
          CONSHDLR_MAXPREROUNDS,
          CONSHDLR_DELAYSEPA,
          CONSDHLR_DELAYPROP,
          CONSHDLR_NEEDSCONS,
          CONSHDLR_PROPTIMING,
          CONSHDLR_PRESOLTIMING)
{
}

BendersSAHLP::BendersSAHLP(
    SCIP *scip,
    const char *name,
    const char *desc,
    const int enfopriority,
    const int checkpriority)
    : ObjConshdlr(
          scip,
          name,
          desc,
          CONSHDLR_SEPAPRIORITY,
          enfopriority,
          checkpriority,
          CONSHDLR_SEPAFREQ,
          CONSHDLR_PROPFREQ,
          CONSHDLR_EAGERFREQ,
          CONSHDLR_MAXPREROUNDS,
          CONSHDLR_DELAYSEPA,
          CONSDHLR_DELAYPROP,
          CONSHDLR_NEEDSCONS,
          CONSHDLR_PROPTIMING,
          CONSHDLR_PRESOLTIMING)
{
}

SCIP_DECL_CONSINIT(BendersSAHLP::scip_init)
{
    // set up the clocks for timing the subproblem solving
    SCIP_CALL(SCIPcreateClock(scip_, &subproblem_solving_clock));
    SCIP_CALL(SCIPcreateClock(scip_, &subproblem_setup_clock));
    SCIP_CALL(SCIPcreateClock(scip_, &callback_clock));

    return SCIP_OKAY;
}

SCIP_DECL_CONSEXIT(BendersSAHLP::scip_exit)
{
    // free the clocks
    SCIP_CALL(SCIPfreeClock(scip_, &subproblem_solving_clock));
    SCIP_CALL(SCIPfreeClock(scip_, &subproblem_setup_clock));
    SCIP_CALL(SCIPfreeClock(scip_, &callback_clock));

    // release auxiliary variables that were added by this constraint handler
    for (auto auxvar : auxvars)
    {
        SCIP_CALL(SCIPreleaseVar(scip_, &auxvar));
    }
    return SCIP_OKAY;
}

SCIP_DECL_CONSTRANS(BendersSAHLP::scip_trans)
{
    // map aux variables to transformed variables
    for (size_t i = 0; i < auxvars.size(); i++)
    {
        SCIP_VAR *orig_var = auxvars[i];
        assert(orig_var != NULL);
        SCIP_CALL(SCIPreleaseVar(scip, &(auxvars[i])));
        SCIP_CALL(SCIPgetTransformedVar(scip, orig_var, &(auxvars[i])));
        SCIP_CALL(SCIPcaptureVar(scip, auxvars[i]));
    }
    return SCIP_OKAY;
}

SCIP_DECL_CONSLOCK(BendersSAHLP::scip_lock)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip);
    // rounding up or down the assignment variables might make this constraint infeasible
    for (auto [ik, var] : probdata->z_vars)
    {
        SCIP_CALL(SCIPaddVarLocksType(scip, var, SCIP_LOCKTYPE_MODEL, nlockspos + nlocksneg, nlocksneg + nlockspos));
    }
    // rounding down the aux variables might make this constraint infeasible
    for (auto auxvar : auxvars)
    {
        SCIP_CALL(SCIPaddVarLocksType(scip, auxvar, SCIP_LOCKTYPE_MODEL, nlockspos, nlocksneg));
    }
    return SCIP_OKAY;
}

bool BendersSAHLP::solutionCheckedBefore(
    SCIP_SOL *sol)
{
    int solindex = SCIPsolGetIndex(sol);
    // iterate over already checked solutions from the back, since we are more likely to be asked to check a recently added solution again
    for (auto it = checked_solution_ids.rbegin(); it != checked_solution_ids.rend(); it++)
    {
        if (*it == solindex)
        {
            return true;
        }
    }
    return false;
}

void BendersSAHLP::registerCheckedSolution(SCIP_SOL *sol)
{
    int solindex = SCIPsolGetIndex(sol);
    checked_solution_ids.push_back(solindex);
}

SCIP_RETCODE BendersSAHLP::constructValidSolution(
    SCIP_SOL *sol,
    SCIP_BENDERSENFOTYPE type)
{
    SCIP_SOL *newsol;
    SCIP_Bool success = TRUE;
    // this function is  mostly copied from the SCIP benders code
    /* don't propose new solutions if not in presolve or solving */
    if (SCIPgetStage(scip_) < SCIP_STAGE_INITPRESOLVE || SCIPgetStage(scip_) >= SCIP_STAGE_SOLVED)
        return SCIP_OKAY;

    /* if the solution is NULL, then we create the solution from the LP sol */
    if (sol != NULL)
    {
        assert(type == SCIP_BENDERSENFOTYPE_CHECK);
        SCIP_CALL(SCIPcreateSolCopy(scip_, &newsol, sol));
    }
    else
    {
        switch (type)
        {
        case SCIP_BENDERSENFOTYPE_LP:
            SCIP_CALL(SCIPcreateLPSol(scip_, &newsol, NULL));
            break;
        case SCIP_BENDERSENFOTYPE_PSEUDO:
            SCIP_CALL(SCIPcreatePseudoSol(scip_, &newsol, NULL));
            break;
        case SCIP_BENDERSENFOTYPE_RELAX:
            SCIP_CALL(SCIPcreateRelaxSol(scip_, &newsol, NULL));
            break;
        default:
            SCIP_CALL(SCIPcreateLPSol(scip_, &newsol, NULL));
            break;
        } /*lint !e788*/
    }
    SCIP_CALL(SCIPunlinkSol(scip_, newsol));

    // set all auxiliary variables to their values
    for (size_t i = 0; i < n_subproblems; i++)
    {
        double objval = subproblemobjvals[i];
        double auxval = objval / auxobj; // new value for the auxiliary variable

        // check that we can actually set the auxiliary variable to the value we want
        if (SCIPvarGetStatus(auxvars[i]) == SCIP_VARSTATUS_FIXED && !SCIPisEQ(scip_, SCIPgetSolVal(scip_, newsol, auxvars[i]), auxval))
        {
            success = FALSE;
            break;
        }
        else if (SCIPisLT(scip_, SCIPgetSolVal(scip_, newsol, auxvars[i]), auxval))
        {
            SCIP_CALL(SCIPsetSolVal(scip_, newsol, auxvars[i], auxval));
        }
    }
    assert(success);

    // try submitting the solution
    // since constraint handlers can not register primal solutions, they need to be passed to the trysol heuristic that will do it once its called the next time.
    registerCheckedSolution(newsol);
    SCIP_HEUR *heurtrysol = SCIPfindHeur(scip_, "trysol");
    SCIP_CALL(SCIPcheckSol(scip_, newsol, TRUE, FALSE, TRUE, TRUE, TRUE, &success));
    assert(success);
    SCIP_CALL(SCIPheurPassSolAddSol(scip_, heurtrysol, newsol));

    SCIP_CALL(SCIPfreeSol(scip_, &newsol));

    return SCIP_OKAY;
}

SCIP_DECL_CONSCHECK(BendersSAHLP::scip_check)
{
    SCIP_CALL(SCIPstartClock(scip_, callback_clock));
    assert(scip != NULL);
    *result = SCIP_FEASIBLE;
    bool auxviolated;

    // check if solution was constructed by this constraint handler
    if (solutionCheckedBefore(sol))
    {
        SCIP_CALL(SCIPstopClock(scip_, callback_clock));
        return SCIP_OKAY;
    }

    // solution has not yet been checked so we solve the subproblems
    // if the auxiliary variables that values that are lower than optimal values of the subproblems, we
    // * reject the solution and construct a valid solution with appropriate auxiliary variables and
    // * add a cut to the master problem
    SCIP_CALL(separateBendersCuts(sol, &auxviolated));
    if (auxviolated)
    {
        *result = SCIP_INFEASIBLE;
        if (printreason)
            SCIPmessagePrintInfo(SCIPgetMessagehdlr(scip), "subproblems are feasible but auxiliary variables were violated\n");
        if (!SCIPsolIsOriginal(sol))
        {
            SCIP_CALL(constructValidSolution(sol, SCIP_BENDERSENFOTYPE_CHECK));
        }
    }
    else
    {
        checked_solution_ids.push_back(SCIPsolGetIndex(sol));
    }

    SCIP_CALL(SCIPstopClock(scip_, callback_clock));
    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOLP(BendersSAHLP::scip_enfolp)
{
    SCIP_CALL(SCIPstartClock(scip_, callback_clock));
    assert(scip != NULL);

    SCIP_CALL(enforceSolution(NULL, result, TRUE, SCIP_BENDERSENFOTYPE_LP));

    SCIP_CALL(SCIPstopClock(scip_, callback_clock));
    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOPS(BendersSAHLP::scip_enfops)
{
    SCIP_CALL(SCIPstartClock(scip_, callback_clock));
    assert(scip != NULL);

    SCIP_CALL(enforceSolution(NULL, result, TRUE, SCIP_BENDERSENFOTYPE_PSEUDO));

    SCIP_CALL(SCIPstopClock(scip_, callback_clock));
    return SCIP_OKAY;
}

SCIP_RETCODE BendersSAHLP::enforceSolution(
    SCIP_SOL *sol,
    SCIP_RESULT *result,
    SCIP_Bool intsol,
    SCIP_BENDERSENFOTYPE type)
{
    bool auxviolated; // some auxillary variable values were too low?

    SCIP_CALL(separateBendersCuts(sol, &auxviolated));

    // if the master problem solution was integral but the auxiliary variable values were too low, we need to construct a valid solution
    if (intsol && auxviolated)
    {
        SCIP_CALL(constructValidSolution(sol, type));
    }
    if (auxviolated)
    {
        *result = SCIP_CONSADDED;
    }
    else
    {
        *result = SCIP_FEASIBLE;
    }

    return SCIP_OKAY;
}

SCIP_RETCODE BendersSAHLP::initializeCorePoint()
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    assert(probdata != NULL);
    auto &corepoint = bendersdata->corepoint; // get reference to corepoint shared by both constraint handlers
    int n_nodes = probdata->nodes.size();

    // reset point to zero
    corepoint = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));

    int n_possible_hubs = 0;
    for (auto [k, var] : probdata->open_hub_vars)
    {
        bool fixed_to_zero = SCIPvarGetUbGlobal(var) < 0.5;
        if (!fixed_to_zero)
        {
            n_possible_hubs++;
        }
    }

    // core point for p_median problem variant
    if (probdata->with_p_constraint)
    {
        double opening_var_val = (probdata->p - 0.5) / n_possible_hubs;
        for (auto node : probdata->nodes)
        {
            int i = node->node_id;
            if (node->still_allowed_hubs.size() == 1)
            {
                int k = node->still_allowed_hubs[0]->node_id;
                corepoint[i][k] = 1.0;
            }
            else
            {
                // see if node is a hub and if yes, can still be opened
                Hub *hubnode = dynamic_cast<Hub *>(node); // try casting to hub
                if (dynamic_cast<Hub *>(node) != nullptr && SCIPvarGetUbGlobal(probdata->open_hub_vars[hubnode]) > 0.5)
                {
                    // set all z_ik for i that can still be a hub
                    for (auto hub : node->still_allowed_hubs)
                    {
                        int k = hub->node_id;
                        corepoint[i][k] = (1.0 - opening_var_val) / (node->still_allowed_hubs.size() - 1.0);
                    }
                    // overwrite z_ii
                    corepoint[i][i] = opening_var_val;
                }
                else // node can not be hub
                {
                    // set all z_ik for i that can not be hub anymore
                    for (auto hub : node->still_allowed_hubs)
                    {
                        int k = hub->node_id;
                        corepoint[i][k] = 1.0 / node->still_allowed_hubs.size();
                    }
                }
            }
        }
    }
    else // core point for variant without p constraint
    {
        for (auto node : probdata->nodes)
        {
            int i = node->node_id;
            if (node->still_allowed_hubs.size() == 1)
            {
                int k = node->still_allowed_hubs[0]->node_id;
                corepoint[i][k] = 1;
            }
            else
            {
                // see if node is a hub and if yes, can still be opened
                Hub *hubnode = dynamic_cast<Hub *>(node); // try casting to hub
                if (dynamic_cast<Hub *>(node) != nullptr && SCIPvarGetUbGlobal(probdata->open_hub_vars[hubnode]) > 0.5)
                {
                    // set all z_ik for i that can still be a hub
                    for (auto hub : node->still_allowed_hubs)
                    {
                        int k = hub->node_id;
                        corepoint[i][k] = (0.5) / (node->still_allowed_hubs.size() - 1);
                    }
                    // overwrite z_ii
                    corepoint[i][i] = 0.5;
                }
                else // node can not be hub
                {
                    // set all z_ik for i that can not be hub anymore
                    for (auto hub : node->still_allowed_hubs)
                    {
                        int k = hub->node_id;
                        corepoint[i][k] = 1 / node->still_allowed_hubs.size();
                    }
                }
            }
        }
    }
    SCIPdebugMsg(scip_, "initialized core point\n");
    return SCIP_OKAY;
}

SCIP_RETCODE BendersSAHLP::updateCorePoint(SCIP_SOL *sol)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    const double alpha = 0.5;
    for (auto node : probdata->nodes)
    {
        int i = node->node_id;
        for (auto hub : node->still_allowed_hubs)
        {
            int k = hub->node_id;
            SCIP_VAR *z_var = probdata->z_vars_vector[i][k];
            double val = SCIPgetSolVal(scip_, sol, z_var);

            bendersdata->corepoint[i][k] = alpha * bendersdata->corepoint[i][k] + (1 - alpha) * val;
        }
    }
    SCIPdebugMsg(scip_, "updated core point\n");
    return SCIP_OKAY;
}

SCIP_RETCODE BendersSAHLP::separateBendersCuts(
    SCIP_SOL *sol,
    bool *auxviolated)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    int n_added = 0;
    *auxviolated = false;

    SCIPdebugMsg(scip_, "separating benders cuts\n");
#ifdef MORE_DEBUG
    SCIPdebugMsg(scip_, "solution to separate:\n");
    SCIPprintSol(scip_, sol, NULL, TRUE);
#endif

    bool is_first_round = bendersdata->n_separation_rounds == 0;
    if (probdata->updateNecessary() || is_first_round)
    {
        SCIP_CALL(probdata->updateAllowedHubs());
        SCIP_CALL(initializeCorePoint());
    }
    SCIP_CALL(updateCorePoint(sol));

    // iterate over subproblems consisting of customer pairs
    // for every customer pair we can solve a separate subproblem and combine the coefficients to generate a cut for the set of pairs
    for (size_t subproblem_idx = 0; subproblem_idx < n_subproblems; subproblem_idx++)
    {
        boost::unordered_map<SCIP_VAR *, SCIP_Real> coefficients;
        SCIP_VAR *auxvar = auxvars[subproblem_idx];
        SCIP_CONS *cons;

        for (auto [i, j] : node_pairs[subproblem_idx])
        {
            SCIP_CALL(solveSubproblem(sol, i, j, coefficients));
        }

        // only add cuts if problem has not been solved
        bool problem_is_solved = SCIPgetStage(scip_) >= SCIP_STAGE_SOLVED;
        if (!problem_is_solved)
        {
            SCIP_CALL(constructBendersCut(&cons, coefficients, auxvar));

            double activity = SCIPgetActivityLinear(scip_, cons, sol);
            bool cut_is_violated = activity > SCIPfeastol(scip_);
            if (cut_is_violated)
            {
                *auxviolated = true;
                SCIP_CALL(SCIPaddCons(scip_, cons));
                n_added += 1;
                bendersdata->n_cuts_generated++;
            }
            SCIP_CALL(SCIPreleaseCons(scip_, &cons));

            // compute the value that the auxiliary variable should take to be feasible
            // since the auxiliary variable is in the cut with a negative coefficient, we need to add its contribution again (since we only care about the activity of the assignment variables)
            double aux_sol_val = SCIPgetSolVal(scip_, sol, auxvar);
            subproblemobjvals[subproblem_idx] = activity + aux_sol_val * auxobj;
        }
    }

    SCIPdebugMsg(scip_, "added %i cons\n", n_added);
    bendersdata->n_separation_rounds++;
    return SCIP_OKAY;
}

#ifdef WITHMCF
SCIP_RETCODE BendersSAHLP::solveSubproblemMCF(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems)
{
    MCFClass_di_unipi_it::MCFSimplex mcfsolver(n_nodes, n_arcs);
    // need to adjust the input data for this solver
    // invert supply sign..
    for (int i = 0; i < n_nodes; i++)
    {
        supply[i] = -supply[i];
    }
    // convert head and tail to unsigned int
    unsigned int *tail_unsigned = new unsigned int[n_arcs];
    unsigned int *head_unsigned = new unsigned int[n_arcs];
    for (int i = 0; i < n_arcs; i++)
    {
        tail_unsigned[i] = tail[i] + 1; // offset by one since MCFClass starts node indices with 1 instead of 0
        head_unsigned[i] = head[i] + 1;
    }

    // create instance
    mcfsolver.LoadNet(n_nodes, n_arcs, n_nodes, n_arcs, ub, obj, supply, tail_unsigned, head_unsigned);
    SCIP_CALL(SCIPstopClock(scip_, subproblem_setup_clock));

    // solve problem
    SCIP_CALL(SCIPstartClock(scip_, subproblem_solving_clock));
    double tolerance = 1e-6;
    mcfsolver.SetPar(MCFClass_di_unipi_it::MCFClass::kEpsFlw, tolerance);
    mcfsolver.SetPar(MCFClass_di_unipi_it::MCFClass::kEpsDfct, tolerance);
    mcfsolver.SetPar(MCFClass_di_unipi_it::MCFClass::kEpsCst, tolerance);
    mcfsolver.SolveMCF();
    SCIP_CALL(SCIPstopClock(scip_, subproblem_solving_clock));
    if (mcfsolver.MCFGetStatus() != mcfsolver.kOK)
    {
        numerical_problems = true;
        n_subproblems_numerical_problems++;
        SCIPwarningMessage(scip_, "MCF solver returned with status %i (0 = feasible, 2 = infeasible, 3 = unbounded), now solving subproblem with SCIP\n", mcfsolver.MCFGetStatus());
        SCIP_CALL(SCIPstartClock(scip_, subproblem_setup_clock));
        // reinvert supply values
        for (int i = 0; i < n_nodes; i++)
        {
            supply[i] = -supply[i];
        }
        SCIP_CALL(solveSubproblemSCIP(n_nodes, n_arcs, supply, tail, head, lb, ub, obj, dual_sol, numerical_problems));
    }
    else
    {
        // get solution
        mcfsolver.MCFGetPi(dual_sol);

        // subtract the average value from all dual variables
        double average_dual = 0;
        for (int i = 0; i < n_nodes; i++)
        {
            average_dual += dual_sol[i];
        }
        average_dual /= n_nodes;
        for (int i = 0; i < n_nodes; i++)
        {
            dual_sol[i] -= average_dual;
            dual_sol[i] = -dual_sol[i];
        }
    }

    // free memory
    delete[] tail_unsigned;
    delete[] head_unsigned;
    return SCIP_OKAY;
}
#endif

SCIP_RETCODE BendersSAHLP::solveSubproblemSCIP(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems)
{
    SCIP_LPI *lpi = NULL;
    SCIP_CALL(SCIPlpiCreate(&lpi, NULL, "subproblem", SCIP_OBJSEN_MINIMIZE));

    // coefficient matrix is passed as one long array.
    // for every variable we need to store at which index in the array its coefficients begin
    // and for every array entry we need to store to which row it belongs

    int nnonz = n_arcs * 2; // coefficient matrix has two non-zero entries per arc
    SCIP_Real *val = new double[nnonz];
    int *ind = new int[nnonz];
    int *beg = new int[n_arcs];

    for (int i = 0; i < n_arcs; i++)
    {
        beg[i] = i * 2;
        val[i * 2] = 1;
        val[i * 2 + 1] = -1;
        ind[i * 2] = tail[i];
        ind[i * 2 + 1] = head[i];
    }

    SCIP_CALL(SCIPlpiLoadColLP(lpi, SCIP_OBJSEN_MINIMIZE, n_arcs, obj, lb, ub, NULL, n_nodes, supply, supply, NULL, nnonz, beg, ind, val));

    SCIP_CALL(SCIPstopClock(scip_, subproblem_setup_clock));

    // solve problem and retrieve dual solution
    SCIP_CALL(SCIPstartClock(scip_, subproblem_solving_clock));
    SCIP_CALL(SCIPlpiSolveDual(lpi));
    SCIP_CALL(SCIPstopClock(scip_, subproblem_solving_clock));

    SCIP_Bool primalfeasible;
    SCIP_Bool dualfeasible;
    SCIP_CALL(SCIPlpiGetSolFeasibility(lpi, &primalfeasible, &dualfeasible));
    assert(primalfeasible);
    assert(dualfeasible);
    SCIP_CALL(SCIPlpiGetSol(lpi, NULL, NULL, dual_sol, NULL, NULL));

    // freeing memory
    delete[] val;
    delete[] ind;
    delete[] beg;
    SCIPlpiFree(&lpi);

    return SCIP_OKAY;
}

#ifdef WITHCPLEX
SCIP_RETCODE BendersSAHLP::solveSubproblemCPLEX(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems)
{
    int status;
    CPXENVptr env = NULL; // cplex environment
    CPXNETptr net = NULL; // cplex network problem

    // --- set up cplex ---
    env = CPXopenCPLEX(&status);
    assert(env != NULL);
    net = CPXNETcreateprob(env, &status, "subproblem");
    assert(net != NULL);
    // CPXsetdblparam(env, CPXPARAM_Network_Tolerances_Feasibility, 1e-06);
    // CPXsetdblparam(env, CPXPARAM_Network_Tolerances_Optimality, 1e-06);

    // construct problem
    status = CPXNETchgobjsen(env, net, CPX_MIN);
    assert(status == 0);
    status = CPXNETaddnodes(env, net, n_nodes, supply, NULL);
    assert(status == 0);
    status = CPXNETaddarcs(env, net, n_arcs, tail, head, lb, ub, obj, NULL);
    assert(status == 0);

    SCIPstopClock(scip_, subproblem_setup_clock);

    // -- solve and obtain solution---
    SCIP_CALL(SCIPstartClock(scip_, subproblem_solving_clock));
    status = CPXNETprimopt(env, net);
    SCIP_CALL(SCIPstopClock(scip_, subproblem_solving_clock));
    assert(status == 0);
    int solstat;
    double objval;
    status = CPXNETsolution(env, net, &solstat, &objval, NULL, dual_sol, NULL, NULL);
    assert(status == 0);
    // problem should always be feasible and bounded, but sometimes the primal network simplex
    // claims that the problem is unbounded or infeasible because of numerical problems
    // in these cases, we try to resolve again with the dual simplex of cplex, which seems to suffer less from this problem
    if (solstat != CPX_STAT_OPTIMAL)
    {
        numerical_problems = true;
        SCIPwarningMessage(scip_, "CPLEX network simplex returned with status %i, now trying to resolve with dual simplex\n", solstat);
        n_subproblems_numerical_problems++;
        auto lp = CPXcreateprob(env, &status, "subproblem_lp");
        assert(net != NULL);
        CPXcopynettolp(env, lp, net);
        SCIP_CALL(SCIPstartClock(scip_, subproblem_solving_clock));
        status = CPXdualopt(env, lp);
        SCIP_CALL(SCIPstopClock(scip_, subproblem_solving_clock));
        assert(status == 0);
        status = CPXsolution(env, lp, &solstat, &objval, NULL, dual_sol, NULL, NULL);
        assert(status == 0);
        assert(solstat == CPX_STAT_OPTIMAL);
        if (solstat != CPX_STAT_OPTIMAL)
        {
            SCIPwarningMessage(scip_, "CPLEX dual simplex returned with status %i, now solving subproblem with SCIP\n", solstat);
            solveSubproblemSCIP(n_nodes, n_arcs, supply, tail, head, lb, ub, obj, dual_sol, numerical_problems);
        }
        status = CPXfreeprob(env, &lp);
        assert(status == 0);
    }
    // close down cplex
    status = CPXNETfreeprob(env, &net);
    assert(status == 0);
    status = CPXcloseCPLEX(&env);
    assert(status == 0);
    return SCIP_OKAY;
}
#endif

static void writeSubproblemToFile(int n_origin_nodes, int n_dest_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, int i, int j, int iteration)
{
    std::string filename = "subproblem_" + std::to_string(i) + "_" + std::to_string(j) + "_n" + std::to_string(iteration) + ".tp";
    std::ofstream file;
    file.open(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("could not open file " + filename + " for writing benders subproblem");
    }
    // nodes
    file << n_origin_nodes << " " << n_dest_nodes << std::endl;
    file << std::setprecision (15);
    // supplies
    for (int i = 0; i < n_origin_nodes; i++)
    {
        file << supply[i] << "\n";
    }
    for (int i = n_origin_nodes; i < n_origin_nodes + n_dest_nodes; i++)
    {
        file << -supply[i] << "\n";
    }
    for (int k = 0; k < n_origin_nodes; k++)
    {
        file << obj[k * n_dest_nodes]; // first arc
        for (int l = 1; l < n_dest_nodes; l++)
        {
            int arc_idx = k * n_dest_nodes + l;
            file << " " << obj[arc_idx];
        }
        file << std::endl;
    }
    file.close();
}

SCIP_RETCODE BendersSAHLP::solveSubproblem(
    SCIP_SOL *sol,
    Node *i,
    Node *j,
    boost::unordered_map<SCIP_VAR *, SCIP_Real> &coefficients)
{
    SCIP_CALL(SCIPstartClock(scip_, subproblem_setup_clock));
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    vector<vector<SCIP_VAR *>> &z_vars_vector = probdata->z_vars_vector;

    // --- set up model ---
    // determine size of graph
    int n_origin_hubs = i->still_allowed_hubs.size();
    int n_dest_hubs = j->still_allowed_hubs.size();
    int n_nodes = n_origin_hubs + n_dest_hubs;
    int n_arcs = n_origin_hubs * n_dest_hubs;
    double *dual_sol = new double[n_nodes];

    // allocate memory for node and arc values
    int *tail = new int[n_arcs];
    int *head = new int[n_arcs];
    double *obj = new double[n_arcs];
    double *lb = new double[n_arcs];
    double *ub = new double[n_arcs];
    double *supply = new double[n_nodes];

    // determine node supplies
    int i_idx = i->node_id;
    int j_idx = j->node_id;

    for (int k = 0; k < n_origin_hubs; k++)
    {
        int k_idx = i->still_allowed_hubs[k]->node_id;
        SCIP_VAR *master_var = z_vars_vector[i_idx][k_idx];
        double corepointval = bendersdata->corepoint[i_idx][k_idx];
        supply[k] = SCIPgetSolVal(scip_, sol, master_var) + corepointval;
    }
    for (int l = 0; l < n_dest_hubs; l++)
    {
        int l_idx = j->still_allowed_hubs[l]->node_id;
        SCIP_VAR *master_var = z_vars_vector[j_idx][l_idx];
        double corepointval = bendersdata->corepoint[j_idx][l_idx];
        supply[n_origin_hubs + l] = -SCIPgetSolVal(scip_, sol, master_var) - corepointval;
    }

    // set arc parameters
    double ij_demand = probdata->demands[i_idx][j_idx];
    double ji_demand = probdata->demands[j_idx][i_idx];

    int arc_idx = 0;

    for (int k = 0; k < n_origin_hubs; k++)
    {
        for (int l = 0; l < n_dest_hubs; l++)
        {
            int k_idx = i->still_allowed_hubs[k]->node_id;
            int l_idx = j->still_allowed_hubs[l]->node_id;
            tail[arc_idx] = k;
            head[arc_idx] = n_origin_hubs + l;
            obj[arc_idx] = probdata->transfer_costs[k_idx][l_idx] * ij_demand + probdata->transfer_costs[l_idx][k_idx] * ji_demand;
            ub[arc_idx] = 1e+20;
            lb[arc_idx] = 0;
            arc_idx++;
        }
    }

    bool numerical_problems = false;
#ifdef WITHCPLEX
    SCIP_CALL(solveSubproblemCPLEX(n_nodes, n_arcs, supply, tail, head, lb, ub, obj, dual_sol, numerical_problems));
#else
#ifdef WITHMCF
    SCIP_CALL(solveSubproblemMCF(n_nodes, n_arcs, supply, tail, head, lb, ub, obj, dual_sol, numerical_problems));
#else
    SCIP_CALL(solveSubproblemSCIP(n_nodes, n_arcs, supply, tail, head, lb, ub, obj, dual_sol, numerical_problems));
#endif
#endif
// optionally write subproblem to file
#ifdef WRITE_SUBPROBLEM_W_NUMERICAL_PROBLEMS
    if (numerical_problems)
    {
        writeSubproblemToFile(n_origin_hubs, n_dest_hubs, n_arcs, supply, tail, head, lb, ub, obj, i_idx, j_idx, bendersdata->n_separation_rounds);
    }
#endif

    // read out coefficients
    for (int k = 0; k < n_origin_hubs; k++)
    {
        int k_idx = i->still_allowed_hubs[k]->node_id;
        SCIP_VAR *master_var = z_vars_vector[i_idx][k_idx];

        coefficients[master_var] += dual_sol[k];
    }
    for (int l = 0; l < n_dest_hubs; l++)
    {
        int l_idx = j->still_allowed_hubs[l]->node_id;
        SCIP_VAR *master_var = z_vars_vector[j_idx][l_idx];
        coefficients[master_var] -= dual_sol[n_origin_hubs + l];
    }

    // deallocate memory
    delete[] tail;
    delete[] head;
    delete[] obj;
    delete[] lb;
    delete[] ub;
    delete[] supply;

    return SCIP_OKAY;
}

SCIP_RETCODE BendersSAHLP::constructBendersCut(
    SCIP_CONS **cons,
    boost::unordered_map<SCIP_VAR *, SCIP_Real> &coefficients,
    SCIP_VAR *auxvar)
{
    // obtain variables and values
    // constraint form : -infinity <= coef * z - theta <= 0
    std::vector<SCIP_VAR *> vars;
    std::vector<SCIP_Real> vals;
    double lhs = -SCIPinfinity(scip_);
    double rhs = 0;

    for (auto &[var, val] : coefficients)
    {
        if (ABS(val) < 1e-5)
        {
            continue;
        }
        vars.push_back(var);
        vals.push_back(val);
    }

    vars.push_back(auxvar);
    vals.push_back(-1 * auxobj);

    // create constraint
    std::string consname = "benderscut" + std::to_string(bendersdata->n_cuts_generated);
    // SCIP_CALL(SCIPcreateConsBasicLinear(scip_, cons, consname.c_str(), vars.size(), vars.data(), vals.data(), lhs, rhs));
    SCIP_CALL(SCIPcreateConsLinear(
        scip_,
        cons,
        consname.c_str(),
        vars.size(),
        vars.data(),
        vals.data(),
        lhs,
        rhs,
        TRUE,  // initial
        TRUE,  // should be separated
        TRUE,  // should be enforced
        TRUE,  // should be checked
        TRUE,  // should be propagated
        FALSE, // is not local
        FALSE, // is not modifiable
        FALSE, // is not dynamic
        FALSE, // is not removable
        FALSE  // not sticking to the transformed problem
        ));

#ifdef MORE_DEBUG
    SCIPdebugPrintCons(scip_, *cons, NULL);
#endif

    // settings as in the scip benders framework
    SCIP_CALL(SCIPsetConsDynamic(scip_, *cons, TRUE));
    SCIP_CALL(SCIPsetConsRemovable(scip_, *cons, TRUE));

    return SCIP_OKAY;
}

SCIP_RETCODE
SCIPincludeConshdlrBendersSAHLP(SCIP *scip)
{
    BendersSAHLP *newcons = new BendersSAHLP(scip);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, newcons, TRUE));

    return SCIP_OKAY;
}

SCIP_RETCODE SCIPsetupBenders(
    SCIP *scip,
    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs)
{
    BendersSAHLP *benders = dynamic_cast<BendersSAHLP *>(SCIPfindObjConshdlr(scip, CONSHDLR_NAME));
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip);

    assert(benders != NULL);
    benders->node_pairs = node_pairs;
    benders->n_subproblems = node_pairs.size();
    benders->subproblemobjvals = std::vector<double>(benders->n_subproblems, 0);
    benders->bendersdata = new BendersSharedData();

    // adding the auxiliary variables with an objective coefficient and constraint coefficients 1 led to many numerical troubles.
    // this seems to be more stable: we calculate a scaling factor by which the objective and the constraint coefficients are scaled.
    // the idea is that this scaling factor is approximatly as large as the coefficients of the other variables in the benders cuts.
    // empirically, this seems to work: we estimate the expected total transfer cost for a all demands in a subproblem and scale with this
    // this estimate is calculated as the average non-zero demand times the average transfer cost between hubs
    double average_demand = 0;
    int n_nonzero_demands = 0;
    double average_inter_hub_transfer_cost = 0;
    for (auto i : probdata->nodes)
    {
        for (auto j : probdata->nodes)
        {
            double demand = probdata->demands[i->node_id][j->node_id];
            if (demand > 0)
            {
                average_demand += demand;
                n_nonzero_demands++;
            }
        }
    }
    average_demand = average_demand / n_nonzero_demands;
    for (auto k : probdata->hubs)
    {
        for (auto l : probdata->hubs)
        {
            average_inter_hub_transfer_cost += probdata->transfer_costs[k->node_id][l->node_id];
        }
    }
    average_inter_hub_transfer_cost = average_inter_hub_transfer_cost / (probdata->hubs.size() * probdata->hubs.size());
    benders->auxobj = average_demand * average_inter_hub_transfer_cost;
    // scale with the maximum possible number of subproblems divided by the actual number
    benders->auxobj *= n_nonzero_demands / benders->n_subproblems;

    // add auxiliary variables
    for (size_t i = 0; i < benders->n_subproblems; i++)
    {
        SCIP_VAR *auxvar;
        std::string varname = "auxvar_" + std::to_string(i);
        SCIP_CALL(SCIPcreateVarBasic(scip, &auxvar, varname.c_str(), 0, SCIPinfinity(scip), benders->auxobj, SCIP_VARTYPE_CONTINUOUS));
        benders->auxvars.push_back(auxvar);
        SCIP_CALL(SCIPaddVar(scip, auxvar));

        // not sure if this is necessary, is done in the scip benders plugin:
        // it seems to me that adding the locks in the constraints callback is sufficient?
        // SCIP_CALL(SCIPaddVarLocksType(scip, auxvar, SCIP_LOCKTYPE_MODEL, 1, 0));
        // if we do this, we also need to remove the lock type later:
        // SCIP_CALL(SCIPaddVarLocksType(scip, auxvar, SCIP_LOCKTYPE_MODEL, -1, 0));
    }

    return SCIP_OKAY;
}

double BendersSAHLP::getSubproblemSolvingTime()
{
    return SCIPgetClockTime(scip_, subproblem_solving_clock);
}
double BendersSAHLP::getCallbackTime()
{
    return SCIPgetClockTime(scip_, callback_clock);
}
double BendersSAHLP::getSubproblemSetupTime()
{
    return SCIPgetClockTime(scip_, subproblem_setup_clock);
}

void BendersSAHLP::printTimeSpent()
{
    SCIPinfoMessage(scip_, NULL, "%s:\n", scip_name_);
    SCIPinfoMessage(scip_, NULL, "time spent in callback: %f\n", getCallbackTime());
    SCIPinfoMessage(scip_, NULL, "time spent in subproblem setup: %f\n", getSubproblemSetupTime());
    SCIPinfoMessage(scip_, NULL, "time spent in subproblem solving: %f\n", getSubproblemSolvingTime());
    SCIPinfoMessage(scip_, NULL, "number of subproblem solves with numerical trouble: %d\n", n_subproblems_numerical_problems);
}