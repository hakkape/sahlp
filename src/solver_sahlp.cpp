#include "solver_sahlp.hpp"
#include "cons_benders_sahlp.hpp"
#include "cons_benderslp_sahlp.hpp"
#include "matheuristic_sahlp.hpp"
#include "prop_partialenum.hpp"
#include "reader_sahlp.hpp"
#include "scip/scipdefplugins.h"
namespace sahlp
{
    SCIP_RETCODE buildSCIPSAHLP(SCIP **scip)
    {
        SCIP_CALL(SCIPcreate(scip));
        SCIP_CALL(SCIPincludeDefaultPlugins(*scip));
        SCIP_CALL(SCIPincludeConshdlrBendersSAHLP(*scip));
        SCIP_CALL(SCIPincludeConshdlrBenderslpSAHLP(*scip));
        SCIP_CALL(SCIPincludeObjReader(*scip, new ReaderHLP(*scip), TRUE));
        SCIP_CALL(SCIPincludeObjReader(*scip, new ReaderHLPS(*scip), TRUE));
        SCIP_CALL(SCIPincludeObjReader(*scip, new ReaderSAHLP(*scip), TRUE));
        SCIP_CALL(SCIPincludeObjPropPartialEnum(*scip));

        // disable heuristics and separation since it seems to hurt performance
        SCIP_CALL(SCIPsetHeuristics(*scip, SCIP_PARAMSETTING_OFF, TRUE));
        SCIP_CALL(SCIPsetSeparating(*scip, SCIP_PARAMSETTING_OFF, TRUE));
        // disable presolving and some propagation since it screws with the auxiliary variables
        SCIP_CALL(SCIPsetPresolving(*scip, SCIP_PARAMSETTING_OFF, TRUE));
        SCIP_CALL(SCIPsetIntParam(*scip, "propagating/symmetry/freq", -1));
        SCIP_CALL(SCIPsetIntParam(*scip, "propagating/symmetry/maxprerounds", 0));
        SCIP_CALL(SCIPsetIntParam(*scip, "misc/usesymmetry", 0));

        // re-enable the trysol heuristic as we use it in our benders separator to add solutions
        SCIP_CALL(SCIPsetIntParam(*scip, "heuristics/trysol/priority", 100000001));
        SCIP_CALL(SCIPsetIntParam(*scip, "heuristics/trysol/freq", 1));

        // add custom heuristic after setting the other heuristics to off
        SCIP_CALL(SCIPincludeObjHeur(*scip, new Matheuristic(*scip), TRUE));

        // add user parameters
        SCIP_CALL(SCIPaddIntParam(*scip, "sahlp/aggregation", "how to aggregate benders cuts: 0 (don't), 1 (by origin), 2 (all), 3 (random)", NULL, FALSE, 2, 0, 3, NULL, NULL));
        return SCIP_OKAY;
    }

    void printExtraTimeInfos(SCIP *scip)
    {
        assert(scip != NULL);
        BendersSAHLP *benders = dynamic_cast<BendersSAHLP *>(SCIPfindObjConshdlr(scip, "BendersSAHLP"));
        BenderslpSAHLP *benderslp = dynamic_cast<BenderslpSAHLP *>(SCIPfindObjConshdlr(scip, "BenderslpSAHLP"));
        assert(benders != NULL);
        assert(benderslp != NULL);
        SCIPinfoMessage(scip, NULL, "Solving Time (sec) : %.2f\n", SCIPgetSolvingTime(scip));
        benderslp->printTimeSpent();
        benders->printTimeSpent();
    }

    Solution *solveHLPS(const char *hlp_filename, const char *hlps_filename, const char *scip_setting_filename)
    {
        SCIP *scip = NULL;
        SCIP_CALL_ABORT(buildSCIPSAHLP(&scip));

        // read in files
        if (SCIPfileExists(scip_setting_filename))
        {
            SCIP_CALL_ABORT(SCIPreadParams(scip, scip_setting_filename));
        }
        SCIP_CALL_ABORT(SCIPreadProb(scip, hlp_filename, NULL));
        SCIP_CALL_ABORT(SCIPreadProb(scip, hlps_filename, NULL));

        // solve problem
        SCIP_CALL_ABORT(SCIPsolve(scip));
        SCIP_CALL_ABORT(SCIPprintStatistics(scip, NULL));
        printExtraTimeInfos(scip);

        Solution *solution = SCIPgetObjProbDataSAHLP(scip)->getSolution();
        SCIP_CALL_ABORT(SCIPfree(&scip));

        return solution;
    }

    Solution *solveSAHLP(const char *sahlp_filename, const char *scip_setting_filename)
    {
        SCIP *scip = NULL;
        SCIP_CALL_ABORT(buildSCIPSAHLP(&scip));

        // read in files
        if (SCIPfileExists(scip_setting_filename))
        {
            SCIP_CALL_ABORT(SCIPreadParams(scip, scip_setting_filename));
        }
        SCIP_CALL_ABORT(SCIPreadProb(scip, sahlp_filename, NULL));

        // solve problem
        SCIP_CALL_ABORT(SCIPsolve(scip));
        SCIP_CALL_ABORT(SCIPprintStatistics(scip, NULL));
        printExtraTimeInfos(scip);

        Solution *solution = SCIPgetObjProbDataSAHLP(scip)->getSolution();
        SCIP_CALL_ABORT(SCIPfree(&scip));

        return solution;
    }

    void runSAHLPShell(int argc, char **argv)
    {
        SCIP *scip = NULL;
        SCIP_CALL_ABORT(buildSCIPSAHLP(&scip));
        SCIP_CALL_ABORT(SCIPprocessShellArguments(scip, argc, argv, NULL));
        SCIP_CALL_ABORT(SCIPfree(&scip));

        BMScheckEmptyMemory();
    }

    Instance *toInstance(const CInstance *cins)
    {
        Instance *ins = new Instance;
        ins->n_nodes = cins->n_nodes;
        for (int i = 0; i < cins->n_hubs; i++)
        {
            ins->hubs.push_back(cins->hubs[i]);
        }
        for (int i = 0; i < cins->n_hubs; i++)
        {
            ins->capacities.push_back(cins->capacities[i]);
        }
        ins->demands = std::vector<std::vector<double>>(cins->n_nodes, std::vector<double>(cins->n_nodes, 0));
        for (int i = 0; i < cins->n_nodes; i++)
        {
            for (int j = 0; j < cins->n_nodes; j++)
            {
                ins->demands[i][j] = cins->demands[i][j];
            }
        }
        ins->transfer_costs = std::vector<std::vector<double>>(cins->n_hubs, std::vector<double>(cins->n_hubs, 0));
        for (int i = 0; i < cins->n_hubs; i++)
        {
            for (int j = 0; j < cins->n_hubs; j++)
            {
                ins->transfer_costs[i][j] = cins->transfer_costs[i][j];
            }
        }
        for (int i = 0; i < cins->n_nodes; i++)
        {
            std::vector<std::tuple<int, double>> allowed_assignments_and_costs;
            for (int j = 0; j < cins->n_allowed_hubs[i]; j++)
            {
                allowed_assignments_and_costs.push_back(std::make_tuple(cins->allowed_hubs[i][j], cins->assignment_costs[i][j]));
            }
            ins->allowed_assignments_and_costs.push_back(allowed_assignments_and_costs);
        }
        ins->p = cins->p;
        ins->with_p_constraint = cins->with_p_constraint;
        ins->with_capacities = cins->with_capacities;

        // "OPEN N out of M hubs"
        for (int i = 0; i < cins->n_open_hubs_constraints; i++)
        {
            std::vector<int> candidate_hubs;
            for (int j = 0; j < cins->n_candidate_hubs[i]; j++)
            {
                candidate_hubs.push_back(cins->candidate_hubs[i][j]);
            }
            ins->open_hubs_constraints.push_back(std::make_tuple(cins->n_open_hubs[i], candidate_hubs));
        }
        return ins;
    }

    Solution *solveSAHLPInstance(const Instance *ins, const char *scip_setting_filename, bool quiet, double timelimit)
    {
        SCIP *scip = NULL;
        SCIP_CALL_ABORT(sahlp::buildSCIPSAHLP(&scip));

        SCIPsetMessagehdlrQuiet(scip, quiet);
        // read in files
        if (SCIPfileExists(scip_setting_filename))
        {
            SCIP_CALL_ABORT(SCIPreadParams(scip, scip_setting_filename));
        }
        SCIP_CALL_ABORT(SCIPsetRealParam(scip, "limits/time", timelimit));
        ProbDataSAHLP *probdata = new ProbDataSAHLP(scip, ins);
        SCIP_CALL_ABORT(SCIPcreateObjProb(scip, "sahlp", probdata, TRUE));
        SCIP_CALL_ABORT(probdata->buildMasterProblem());

        // solve problem
        SCIP_CALL_ABORT(SCIPsolve(scip));

        sahlp::Solution *solution = SCIPgetObjProbDataSAHLP(scip)->getSolution();
        SCIP_CALL_ABORT(SCIPfree(&scip));

        // convert to C struct
        return solution;
    }
}

CSolution *sahlp::Solution::toCSolution()
{
    CSolution *csolution = new CSolution;
    csolution->n_nodes = this->assigned_hubs.size();
    csolution->solution_value = this->solution_value;
    csolution->cpu_time = this->cpu_time;
    csolution->assigned_hubs = new int[csolution->n_nodes];
    for (int i = 0; i < csolution->n_nodes; i++)
    {
        csolution->assigned_hubs[i] = this->assigned_hubs[i];
    }
    return csolution;
}

CSolution *solveHLPS_C(const char *hlp_filename, const char *hlps_filename, const char *scip_setting_filename)
{
    sahlp::Solution *solution = sahlp::solveHLPS(hlp_filename, hlps_filename, scip_setting_filename);

    // convert to C struct
    return solution->toCSolution();
}

CSolution *solveSAHLP_C(const char *sahlp_filename, const char *scip_setting_filename)
{
    sahlp::Solution *solution = sahlp::solveSAHLP(sahlp_filename, scip_setting_filename);

    // convert to C struct
    return solution->toCSolution();
}
CSolution *solveSAHLPInstance_C(CInstance *cins, const char *scip_setting_filename)
{
    sahlp::Instance *ins = sahlp::toInstance(cins);
    sahlp::Solution *solution = sahlp::solveSAHLPInstance(ins, scip_setting_filename, true, 1000000000);

    // convert to C struct
    return solution->toCSolution();
}

void freeCSolution(CSolution *sol)
{
    delete[] sol->assigned_hubs;
    delete sol;
}

void freeCInstance(CInstance *ins)
{
    delete[] ins->hubs;
    delete[] ins->n_allowed_hubs;
    delete[] ins->capacities;
    for (int i = 0; i < ins->n_nodes; i++)
    {
        delete[] ins->demands[i];
        delete[] ins->allowed_hubs[i];
        delete[] ins->assignment_costs[i];
    }
    delete[] ins->demands;
    delete[] ins->allowed_hubs;
    delete[] ins->assignment_costs;

    for (int i = 0; i < ins->n_hubs; i++)
    {
        delete[] ins->transfer_costs[i];
    }

    for (int i = 0; i < ins->n_open_hubs_constraints; i++)
    {
        delete[] ins->candidate_hubs[i];
    }
    delete[] ins->n_open_hubs;
    delete[] ins->n_candidate_hubs;
    delete[] ins;
}
