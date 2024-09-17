// #define SCIP_DEBUG
#include "probdata_sahlp.hpp"
#include "cons_benders_sahlp.hpp"
#include "cons_benderslp_sahlp.hpp"
#include "scip/cons_knapsack.h"
#include "scip/cons_linear.h"
#include "scip/cons_setppc.h"
#include "scip/pub_cons.h"
#include "scip/scip.h"
#include <string>

using namespace std;

enum CutAggregationScheme
{
    NONE = 0,
    BY_SENDER = 1,
    ALL = 2,
    RANDOM_NODE_PAIRS = 3
};

ProbDataSAHLP::ProbDataSAHLP(SCIP *scip, const sahlp::Instance *ins) : scip(scip)
{
    // create nodes and hubs
    nodes = vector<Node *>(ins->n_nodes, NULL);
    for (size_t i = 0; i < ins->hubs.size(); i++)
    {
        Hub *hub = new Hub();
        hub->node_id = ins->hubs[i];
        hub->capacity = ins->with_capacities ? ins->capacities[i] : SCIPinfinity(scip);
        nodes[hub->node_id] = hub;
        hubs.push_back(hub);
    }

    for (int i = 0; i < ins->n_nodes; i++)
    {
        if (nodes[i] == NULL)
        {
            nodes[i] = new Node();
            nodes[i]->node_id = i;
        }
    }

    // store allowed assignments
    assignment_costs = vector<vector<double>>(ins->n_nodes, vector<double>(ins->n_nodes, 0.0));
    for (int i = 0; i < ins->n_nodes; i++)
    {
        Node *node = nodes[i];
        for (auto [k, cost] : ins->allowed_assignments_and_costs[i])
        {
            assignment_costs[i][k] = cost;
            Hub *hub = dynamic_cast<Hub *>(nodes[k]);
            node->potential_hubs.push_back(hub);
            hub->potential_nodes.push_back(node);
        }
    }

    // store demands
    demands = vector<vector<double>>(ins->n_nodes, vector<double>(ins->n_nodes, 0.0));
    for (int i = 0; i < ins->n_nodes; i++)
    {
        for (int j = 0; j < ins->n_nodes; j++)
        {
            demands[i][j] = ins->demands[i][j];
        }
    }

    // store transfer costs
    transfer_costs = vector<vector<double>>(ins->n_nodes, vector<double>(ins->n_nodes, 0.0));
    for (size_t k_idx = 0; k_idx < ins->hubs.size(); k_idx++)
    {
        for (size_t l_idx = 0; l_idx < ins->hubs.size(); l_idx++)
        {
            int k = hubs[k_idx]->node_id;
            int l = hubs[l_idx]->node_id;
            transfer_costs[k][l] = ins->transfer_costs[k_idx][l_idx];
        }
    }

    // store problem parameters
    p = ins->p;
    with_p_constraint = ins->with_p_constraint;
    with_capacities = ins->with_capacities;

    // sum up node demands
    for (int i = 0; i < ins->n_nodes; i++)
    {
        for (int j = 0; j < ins->n_nodes; j++)
        {
            nodes[i]->in_demand += demands[j][i];
            nodes[i]->out_demand += demands[i][j];
        }
        nodes[i]->total_demand = nodes[i]->in_demand + nodes[i]->out_demand;
    }

    // store "open n out of m hubs" constraints
    for (size_t i = 0; i < ins->open_hubs_constraints.size(); i++)
    {
        vector<Hub *> hub_subset;
        auto [n_open, hubs] = ins->open_hubs_constraints[i];
        for (auto k : hubs)
        {
            hub_subset.push_back(dynamic_cast<Hub *>(nodes[k]));
        }
        open_n_of_subset_constraints.push_back({hub_subset, n_open});
    }
}

SCIP_RETCODE ProbDataSAHLP::freeVariables()
{
    for (auto &var : z_vars)
    {
        SCIP_CALL(SCIPreleaseVar(scip, &var.second));
    }
    return SCIP_OKAY;
}

SCIP_RETCODE
ProbDataSAHLP::scip_delorig(SCIP *scip)
{
    freeVariables();
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::scip_deltrans(SCIP *scip)
{
    freeVariables();
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::scip_trans(
    SCIP *scip,
    ObjProbData **objprobdata,
    SCIP_Bool *deleteobject)
{
    SCIP_VAR *transvar;
    assert(objprobdata != NULL);
    assert(deleteobject != NULL);

    // create new problem data
    ProbDataSAHLP *transprobdata = new ProbDataSAHLP(scip);

    // copy static data
    transprobdata->p = p;
    transprobdata->nodes = nodes;
    transprobdata->hubs = hubs;
    transprobdata->transfer_costs = transfer_costs;
    transprobdata->assignment_costs = assignment_costs;
    transprobdata->demands = demands;
    transprobdata->with_p_constraint = with_p_constraint;
    transprobdata->with_capacities = with_capacities;
    transprobdata->open_n_of_subset_constraints = open_n_of_subset_constraints;

    // copy variables and map to transformed variables
    transprobdata->z_vars = z_vars;
    transprobdata->z_vars_vector = z_vars_vector;
    transprobdata->open_hub_vars = open_hub_vars;

    for (auto &[ik, var] : z_vars)
    {
        SCIP_CALL(SCIPgetTransformedVar(scip, var, &transvar));
        assert(transvar != NULL);
        transprobdata->z_vars[ik] = transvar;
        SCIP_CALL(SCIPcaptureVar(scip, transvar));

        auto [i, k] = ik;
        transprobdata->z_vars_vector[i->node_id][k->node_id] = transvar;
    }

    for (auto &var : open_hub_vars)
    {
        SCIP_CALL(SCIPgetTransformedVar(scip, var.second, &transvar));
        assert(transvar != NULL);
        transprobdata->open_hub_vars[var.first] = transvar;
    }

    *deleteobject = TRUE;
    *objprobdata = transprobdata;
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::buildMasterProblem()
{
    SCIP_CALL(addAssignmentVariables());
    SCIP_CALL(addAssignmentConstraints());
    SCIP_CALL(addLinkingConstraints());

    if (with_p_constraint)
    {
        SCIP_CALL(addPConstraint());
    }

    if (with_capacities)
    {
        SCIP_CALL(addCapacityConstraint());
    }

    SCIP_CALL(addOpenNofSubsetConstraints());

    // decide how to aggregate benders cuts
    int aggregation;
    SCIP_CALL(SCIPgetIntParam(scip, "sahlp/aggregation", &aggregation));

    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs;
    if (aggregation == NONE)
    {
        // one subproblem per node pair
        for (size_t i = 0; i < nodes.size(); i++)
        {
            for (size_t j = i + 1; j < nodes.size(); j++)
            {
                if (demands[i][j] + demands[j][i] > 0)
                    node_pairs.push_back({{nodes[i], nodes[j]}});
            }
        }
    }
    else if (aggregation == BY_SENDER)
    {
        // one subproblem per node
        for (size_t i = 0; i < nodes.size(); i++)
        {
            node_pairs.push_back({});
            for (size_t j = i + 1; j < nodes.size(); j++)
            {
                if (demands[i][j] + demands[j][i] > 0)
                    node_pairs[i].push_back({nodes[i], nodes[j]});
            }
        }
    }
    else if (aggregation == ALL)
    {
        // one subproblem containing all node pairs
        node_pairs.push_back({});
        for (size_t i = 0; i < nodes.size(); i++)
        {
            for (size_t j = i + 1; j < nodes.size(); j++)
            {
                if (demands[i][j] + demands[j][i] > 0)
                    node_pairs[0].push_back({nodes[i], nodes[j]});
            }
        }
    }
    else if (aggregation == RANDOM_NODE_PAIRS)
    {
        int n_subproblems = nodes.size();

        // create one empty pair per subproblem
        for (int i = 0; i < n_subproblems; i++)
        {
            node_pairs.push_back({});
        }
        // randomly assign every possible pair to a subproblem
        for (size_t i = 0; i < nodes.size(); i++)
        {
            for (size_t j = i + 1; j < nodes.size(); j++)
            {
                if (demands[i][j] + demands[j][i] > 0)
                {
                    int random_index = rand() % n_subproblems;
                    node_pairs[random_index].push_back({nodes[i], nodes[j]});
                }
            }
        }
    }
    else
    {
        SCIPerrorMessage("Aggregation not yet implemented! \n");
        SCIPABORT();
    }

    SCIP_CALL(SCIPsetupBenders(scip, node_pairs));
    SCIP_CALL(SCIPsetupBenderslp(scip, node_pairs));

    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addAssignmentVariables()
{
    string varname;
    SCIP_VAR *var;

    z_vars_vector = vector<vector<SCIP_VAR *>>(nodes.size(), vector<SCIP_VAR *>(nodes.size(), NULL));

    for (auto i : nodes)
    {
        for (auto k : i->potential_hubs)
        {
            int i_idx = i->node_id;
            int k_idx = k->node_id;
            varname = "z_" + to_string(i_idx) + "_" + to_string(k_idx);
            SCIP_CALL(SCIPcreateVarBasic(scip, &var, varname.c_str(), 0.0, 1.0, assignment_costs[i_idx][k_idx], SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip, var));
            z_vars[{i, k}] = var;
            z_vars_vector[i_idx][k_idx] = var;
        }
    }

    // store the hub opening variables
    for (auto hub : hubs)
    {
        int k = hub->node_id;
        assert(z_vars_vector[k][k] != NULL);
        open_hub_vars[hub] = z_vars_vector[k][k];
    }
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addAssignmentConstraints()
{
    string consname;
    SCIP_CONS *cons;

    for (auto i : nodes)
    {
        consname = "assign_" + to_string(i->node_id);
        SCIP_CALL(SCIPcreateConsBasicSetpart(scip, &cons, consname.c_str(), 0, NULL));
        for (auto k : i->potential_hubs)
        {
            SCIP_CALL(SCIPaddCoefSetppc(scip, cons, z_vars[{i, k}]));
        }
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addLinkingConstraints()
{
    string consname;
    SCIP_CONS *cons;

    for (auto i : nodes)
    {
        for (auto k : i->potential_hubs)
        {
            consname = "link_" + to_string(i->node_id) + "_" + to_string(k->node_id);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, consname.c_str(), 0, NULL, NULL, -SCIPinfinity(scip), 0.0));
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, z_vars[{i, k}], 1));
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, open_hub_vars[k], -1));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIP_CALL(SCIPreleaseCons(scip, &cons));
        }
    }
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addPConstraint()
{
    SCIP_CONS *cons;

    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "p_median", 0, NULL, NULL, -SCIPinfinity(scip), p));
    for (auto k : hubs)
    {
        SCIP_CALL(SCIPaddCoefLinear(scip, cons, open_hub_vars[k], 1));
    }
    SCIP_CALL(SCIPaddCons(scip, cons));
    SCIP_CALL(SCIPreleaseCons(scip, &cons));

    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addCapacityConstraint()
{
    string consname;
    SCIP_CONS *cons;

    for (auto k : hubs)
    {
        consname = "capacity_" + to_string(k->node_id);
        SCIP_CALL(SCIPcreateConsBasicKnapsack(scip, &cons, consname.c_str(), 0, NULL, NULL, k->capacity));

        for (auto i : k->potential_nodes)
        {
            SCIP_CALL(SCIPaddCoefKnapsack(scip, cons, z_vars[{i, k}], i->in_demand));
        }
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }
    return SCIP_OKAY;
}

SCIP_RETCODE ProbDataSAHLP::addOpenNofSubsetConstraints()
{
    string consname;
    SCIP_CONS *cons;

    for (auto [hub_subset, n_open] : open_n_of_subset_constraints)
    {
        consname = "open_" + to_string(n_open) + "_of_subset";

        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, consname.c_str(), 0, NULL, NULL, n_open, n_open));
        for (auto k : hub_subset)
        {
            SCIP_CALL(SCIPaddCoefLinear(scip, cons, open_hub_vars[k], 1));
        }
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }

    return SCIP_OKAY;
}

sahlp::Solution *ProbDataSAHLP::getSolution()
{
    sahlp::Solution *solution = new sahlp::Solution();
    SCIP_SOL *scipsol = SCIPgetBestSol(scip);
    solution->solution_value = SCIPgetPrimalbound(scip);
    solution->cpu_time = SCIPgetSolvingTime(scip);
    solution->status = SCIPgetStatus(scip);
    if (scipsol == NULL)
    {
        return solution;
    }

    // hubs
    for (auto hub : hubs)
    {
        if (SCIPgetSolVal(scip, scipsol, open_hub_vars[hub]) > 0.5)
        {
            solution->hubs.push_back(hub->node_id);
        }
    }

    // assignments
    solution->assigned_hubs = vector<int>(nodes.size(), 0);

    for (auto node : nodes)
    {
        for (auto hub : node->potential_hubs)
        {
            if (SCIPgetSolVal(scip, scipsol, z_vars[{node, hub}]) > 0.5)
            {
                solution->assigned_hubs[node->node_id] = hub->node_id;
                break;
            }
        }
    }
    return solution;
}

ProbDataSAHLP *SCIPgetObjProbDataSAHLP(SCIP *scip)
{
    assert(scip != NULL);
    ProbDataSAHLP *probdata = dynamic_cast<ProbDataSAHLP *>(SCIPgetObjProbData(scip));
    assert(probdata != NULL);
    return probdata;
}

SCIP_RETCODE
ProbDataSAHLP::updateAllowedHubs()
{
    n_updates++;

    if (SCIPgetStage(scip) < SCIP_STAGE_SOLVED)
    {
        // empty all lists
        for (auto node : nodes)
        {
            node->still_allowed_hubs.clear();
        }
        for (auto hub : hubs)
        {
            hub->still_allowed_nodes.clear();
        }
        still_possible_hubs.clear();

        // todo: only init this once and then set to zero if assignments are forbidden
        still_possible_assignments = vector<vector<bool>>(nodes.size(), vector<bool>(nodes.size(), false));

        // insert assignment variables that can still take value 1
        for (auto [ik, var] : z_vars)
        {
            auto [i, k] = ik;
            double val = SCIPvarGetUbGlobal(var);
            if (val > 0.5)
            {
                i->still_allowed_hubs.push_back(k);
                k->still_allowed_nodes.push_back(i);
                still_possible_assignments[i->node_id][k->node_id] = true;
#ifdef MORE_DEBUG
                SCIPdebugMsg(scip_, "still_allowed: %i -> %i\n", (int)i->node_id, (int)k->node_id);
#endif
                if (i == k)
                {
                    still_possible_hubs.push_back(k);
                }
            }
        }
    }
    else
    {
        // if problem is solved to optimality, we check the global solution
        // without this check it happened that during the last check of the optimal solution some hubs were not locally allowed that were in the optimal solution
        for (auto node : nodes)
        {
            node->still_allowed_hubs = node->potential_hubs;
        }
        for (auto hub : hubs)
        {
            hub->still_allowed_nodes = hub->potential_nodes;
        }
        still_possible_hubs = hubs;
    }

    SCIPdebugMsg(scip, "updated allowed hubs\n");
    return SCIP_OKAY;
}

bool ProbDataSAHLP::updateNecessary()
{
    SCIPdebugMsg(scip, "checking if update of allowed hubs necessary\n");

    bool fixed_more_vars = SCIPgetNFixedVars(scip) > n_fixed;
    bool never_updated = n_updates == 0;
    if (fixed_more_vars || never_updated)
    {
        n_fixed = SCIPgetNFixedVars(scip);
        return true;
    }
    return false;
}