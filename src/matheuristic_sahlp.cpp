// #define SCIP_DEBUG
// #define LOCAL_SEARCH_DEBUG
#include "matheuristic_sahlp.hpp"
#include "scip/scipdefplugins.h"
#include <string>
#include "cons_benders_sahlp.hpp"

using std::vector;

SCIP_DECL_HEUREXEC(Matheuristic::scip_exec)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);

    if (!supportChanged())
    {
        *result = SCIP_DIDNOTRUN;
        SCIPdebugMsg(scip_, "skipped executing matheuristic\n");
        return SCIP_OKAY;
    }
    SCIPdebugMsg(scip_, "executing matheuristic\n");

    if (probdata->updateNecessary())
        probdata->updateAllowedHubs();

    // 1. solve facility location problem on reduced instance
    SCIPdebugMsg(scip_, "solving facility location problem\n");
    SCIP_CALL(solveFacilityModel());

    // 2. run local search to improve solution
    SCIPdebugMsg(scip_, "executing local search\n");
    localSearch();

    // 3. optimize assignments
    // TODO: implement

    // 4. write solution to SCIP
    double current_cost = calculateAssignmentCost(assign);
    if (SCIPisRelLT(scip_, current_cost, SCIPgetPrimalbound(scip_)))
    {
        SCIPdebugMsg(scip_, "matheuristic found better solution (%f < %f)!\n", current_cost, SCIPgetPrimalbound(scip_));
        SCIP_CALL(submitSolution(scip_, heur, assign));
        *result = SCIP_FOUNDSOL;
    }

    return SCIP_OKAY;
}

bool Matheuristic::supportChanged()
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);

    // get current support
    vector<Hub *> new_support;
    for (auto hub : probdata->hubs)
    {
        if (SCIPisPositive(scip_, SCIPgetVarSol(scip_, probdata->open_hub_vars[hub])))
        {
            new_support.push_back(hub);
        }
    }
    // add all hubs that are needed for the "open N of subset" constraints
    // since the variables in the LP relaxation support might not be sufficient to satisfy them
    for (auto [hub_subset, n_open] : probdata->open_n_of_subset_constraints)
    {
        for (auto hub : hub_subset)
        {
            if (std::find(new_support.begin(), new_support.end(), hub) == new_support.end())
            {
                new_support.push_back(hub);
            }
        }
    }
    std::sort(new_support.begin(), new_support.end(), [](Hub *a, Hub *b)
              { return a->node_id < b->node_id; });

    // check if support changed
    if (new_support.size() != support.size())
    {
        support = new_support;
        return true;
    }
    for (size_t i = 0; i < new_support.size(); ++i)
    {
        if (new_support[i] != support[i])
        {
            support = new_support;
            return true;
        }
    }
    return false;
}

SCIP_RETCODE Matheuristic::solveFacilityModel()
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);

    // set up scip instance
    SCIP *scip_facility = NULL;
    SCIP_CALL(SCIPcreate(&scip_facility));
    SCIP_CALL(SCIPincludeDefaultPlugins(scip_facility));
    SCIP_CALL(SCIPcreateProbBasic(scip_facility, "facility_location"));
    SCIPsetMessagehdlrQuiet(scip_facility, TRUE);

    // create empty assignment constraints to later fill with variables (easier to do in this order without introducing other data structures)
    std::string consname;
    SCIP_CONS *cons;
    vector<SCIP_CONS *> assignment_cons;

    for (size_t i = 0; i < probdata->nodes.size(); ++i)
    {
        consname = "assignment_" + std::to_string(i);
        SCIP_CALL(SCIPcreateConsBasicLinear(scip_facility, &cons, consname.c_str(), 0, NULL, NULL, 1.0, 1.0));
        assignment_cons.push_back(cons);
    }

    // create variables
    std::string varname;
    SCIP_VAR *var;
    boost::unordered_map<std::pair<Node *, Hub *>, SCIP_VAR *> z_vars;

    for (auto hub : support)
    {
        for (auto node : hub->still_allowed_nodes)
        {
            int k = hub->node_id;
            int i = node->node_id;
            varname = "z_" + std::to_string(i) + "_" + std::to_string(k);
            SCIP_CALL(SCIPcreateVarBasic(scip_facility, &var, varname.c_str(), 0.0, 1.0, probdata->assignment_costs[i][k], SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip_facility, var));
            z_vars[{node, hub}] = var;

            // add variable to assignment constraint
            SCIP_CALL(SCIPaddCoefLinear(scip_facility, assignment_cons[i], var, 1.0));
        }
    }

    // add and release assignment constraints
    for (auto cons : assignment_cons)
    {
        SCIP_CALL(SCIPaddCons(scip_facility, cons));
        SCIP_CALL(SCIPreleaseCons(scip_facility, &cons));
    }

    // p-median constraint
    if (probdata->with_p_constraint)
    {
        consname = "p_median";
        SCIP_CALL(SCIPcreateConsBasicLinear(scip_facility, &cons, consname.c_str(), 0, NULL, NULL, 0.0, probdata->p));
        for (auto hub : support)
        {
            SCIP_CALL(SCIPaddCoefLinear(scip_facility, cons, z_vars[{hub, hub}], 1.0));
        }
        SCIP_CALL(SCIPaddCons(scip_facility, cons));
        SCIP_CALL(SCIPreleaseCons(scip_facility, &cons));
    }

    // linking constraints
    for (auto [ik, var] : z_vars)
    {
        auto [i, k] = ik;
        consname = "linking_" + std::to_string(i->node_id) + "_" + std::to_string(k->node_id);
        SCIP_CALL(SCIPcreateConsBasicLinear(scip_facility, &cons, consname.c_str(), 0, NULL, NULL, -SCIPinfinity(scip_facility), 0.0));
        SCIP_CALL(SCIPaddCoefLinear(scip_facility, cons, var, 1.0));
        SCIP_CALL(SCIPaddCoefLinear(scip_facility, cons, z_vars[{k, k}], -1.0));
        SCIP_CALL(SCIPaddCons(scip_facility, cons));
        SCIP_CALL(SCIPreleaseCons(scip_facility, &cons));
    }

    // capacity constraints
    if (probdata->with_capacities)
    {
        for (auto k : support)
        {
            consname = "capacity_" + std::to_string(k->node_id);
            SCIP_CALL(SCIPcreateConsBasicKnapsack(scip_facility, &cons, consname.c_str(), 0, NULL, NULL, k->capacity));
            for (auto i : k->still_allowed_nodes)
            {
                SCIP_CALL(SCIPaddCoefKnapsack(scip_facility, cons, z_vars[{i, k}], i->in_demand));
            }
            SCIP_CALL(SCIPaddCons(scip_facility, cons));
            SCIP_CALL(SCIPreleaseCons(scip_facility, &cons));
        }
    }

    // open n of subset constraints
    for (auto [hub_subset, n_open] : probdata->open_n_of_subset_constraints)
    {
        consname = "open_" + std::to_string(n_open) + "_of_subset";

        SCIP_CALL(SCIPcreateConsBasicLinear(scip_facility, &cons, consname.c_str(), 0, NULL, NULL, n_open, n_open));
        for (auto k : hub_subset)
        {
            // check if variable is even in support of current solution
            if (z_vars.find({k, k}) != z_vars.end())
            {
                SCIP_CALL(SCIPaddCoefLinear(scip_facility, cons, z_vars[{k, k}], 1));
            }
        }
        SCIP_CALL(SCIPaddCons(scip_facility, cons));
        SCIP_CALL(SCIPreleaseCons(scip_facility, &cons));
    }

    // solve and write solution to assignment vector
    SCIP_CALL(SCIPsolve(scip_facility));
    assert(SCIPgetStatus(scip_facility) == SCIP_STATUS_OPTIMAL);

    SCIP_SOL *sol = SCIPgetBestSol(scip_facility);

    // write solution and free data
    for (auto [ik, var] : z_vars)
    {
        if (SCIPgetSolVal(scip_facility, sol, var) > 0.5)
        {
            auto [i, k] = ik;
            assign[i->node_id] = k->node_id;
        }
        SCIP_CALL(SCIPreleaseVar(scip_facility, &var));
    }
    SCIP_CALL(SCIPfree(&scip_facility));

    return SCIP_OKAY;
}

void Matheuristic::localSearch()
{
    bool improved = false;
    double current_cost;

    current_cost = calculateAssignmentCost(assign);
    updateRemainingCapacities();

    SCIPdebugMsg(scip_, "cost before local search: %f\n", current_cost);

    // search neighborhoods in the following order:
    // 1. swap nodes
    // 2. shift nodes to another hub
    // 3. close one and open another hub
    // the other two neighborhoods are not considered here since for now we only look at problems with a p-median constraint
    // if any search finds an improvement, we start again from the top
    do
    {
        improved = swapNodes(current_cost);
        if (!improved)
        {
            improved = shiftNodes(current_cost);
            if (!improved)
            {
                improved = switchHubs(current_cost);
            }
        }
    } while (improved);

    SCIPdebugMsg(scip_, "cost after local search: %f\n", current_cost);
}

bool Matheuristic::swapNodes(double &current_cost)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    vector<vector<bool>> &allowed = probdata->still_possible_assignments;
    bool improved = false;

    // try switching every i->k, j->l with i!=j, k!=l
    for (auto node : probdata->nodes)
    {
        int i = node->node_id;
        int k = assign[i];
        for (auto node2 : probdata->nodes)
        {
            int j = node2->node_id;
            int l = assign[j];

            // only consider the switch if it is allowed
            if ((i != j) && (k != l) && allowed[i][l] && allowed[j][k] && (remaining_capacity[k] > node2->in_demand - node->in_demand) && (remaining_capacity[l] > node->in_demand - node2->in_demand))
            {
                double delta = deltaSwap(i, j, k, l);
                if (delta > SCIPdualfeastol(scip_))
                {
                    // switch nodes
                    assign[i] = l;
                    assign[j] = k;
                    current_cost -= delta;
                    remaining_capacity[k] += node->in_demand - node2->in_demand;
                    remaining_capacity[l] += node2->in_demand - node->in_demand;

#ifdef LOCAL_SEARCH_DEBUG
                    SCIPdebugMsg(scip_, "swapped, now %i -> %i and %i -> %i (delta: %f)\n", i, l, j, k, -delta);
#endif
                    // update current assignment of i to continue with the loop
                    k = l;

                    // return true;
                    improved = true;
                }
            }
        }
    }
    return improved;
}

double Matheuristic::deltaSwap(int i, int j, int k, int l)
{
    // assignment costs for i, j
    double delta = assignment_cost[i][k] + assignment_cost[j][l] - assignment_cost[i][l] - assignment_cost[j][k];

    // hub transfers for i, j
    delta += (transfer_cost[k][l] - transfer_cost[l][k]) * (demand[i][j] - demand[j][i]);

    // hub transfers for all other nodes
    for (int m = 0; m < (int)assignment_cost.size(); m++)
    {
        if (m != i && m != j)
        {
            int h = assign[m]; // hub that m is assigned to
            // transfers from i, j to m
            delta += (transfer_cost[k][h] - transfer_cost[l][h]) * (demand[i][m] - demand[j][m]);
            // transfers from m to i, j
            delta += (transfer_cost[h][k] - transfer_cost[h][l]) * (demand[m][i] - demand[m][j]);
        }
    }
    return delta;
}

bool Matheuristic::shiftNodes(double &current_cost)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    bool improved = false;

    for (auto node : probdata->nodes)
    {
        int i = node->node_id;
        int k = assign[i]; // currently assigned hub

        for (auto hub : node->still_allowed_hubs)
        {
            int l = hub->node_id;
            bool is_open = (assign[l] == l);
            if (is_open && (k != l) && (remaining_capacity[l] > node->in_demand))
            {
                double delta = deltaShift(i, k, l);
                if (delta > SCIPdualfeastol(scip_))
                {
                    // shift node
                    assign[i] = l;
                    remaining_capacity[k] += node->in_demand;
                    remaining_capacity[l] -= node->in_demand;
                    current_cost -= delta;
#ifdef LOCAL_SEARCH_DEBUG
                    SCIPdebugMsg(scip_, "shifted, now %i -> %i (was %i, delta: %f)\n", i, l, k, -delta);
#endif

                    // update assignment of current node to continue with loop
                    k = l;
                    improved = true;
                }
            }
        }
    }
    return improved;
}

double Matheuristic::deltaShift(int i, int k, int l)
{
    // assignment costs for i
    double delta = assignment_cost[i][k] - assignment_cost[i][l];

    // self-transfers for i
    delta += (transfer_cost[k][k] - transfer_cost[l][l]) * (demand[i][i]);

    // transfer costs between i and all j != i
    for (int j = 0; j < (int)assignment_cost.size(); j++)
    {
        if (j != i)
        {
            int h = assign[j]; // hub that j is assigned to
            // transfers from i to j
            delta += (transfer_cost[k][h] - transfer_cost[l][h]) * (demand[i][j]);
            // transfers from j to i
            delta += (transfer_cost[h][k] - transfer_cost[h][l]) * (demand[j][i]);
        }
    }
    return delta;
}

bool Matheuristic::switchHubs(double &current_cost)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    vector<int> new_assign; // new assignments after the currently explored switch

    // if constraints are included that specific subsets of hubs be partially opened, we skip the hub switching
    if (!probdata->open_n_of_subset_constraints.empty())
    {
        return false;
    }

    for (auto hub : probdata->still_possible_hubs)
    {
        int k = hub->node_id;
        // look at currently open hubs that could be closed (reassigned to another hub)
        if (assign[k] == k && hub->potential_hubs.size() > 1)
        {
            for (auto hub2 : probdata->still_possible_hubs)
            {
                int l = hub2->node_id;
                // look at currently closed hubs
                if (assign[l] != l)
                {
                    double new_cost = costAfterSwitch(k, l, new_assign);
                    if (current_cost - new_cost > SCIPdualfeastol(scip_))
                    {
#ifdef LOCAL_SEARCH_DEBUG
                        SCIPdebugMsg(scip_, "closed %i and opened %i (delta: %f)\n", k, l, new_cost - current_cost);
#endif
                        assign = new_assign;
                        current_cost = new_cost;
                        updateRemainingCapacities();

                        return true;
                    }
                }
            }
        }
    }
    return false;
}

double Matheuristic::costAfterSwitch(int k, int l, vector<int> &new_assign)
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);

    // since we determine all new assignments, start from scratch
    new_assign = vector<int>(assign.size(), -1);
    vector<double> new_remaining_capacity = vector<double>(remaining_capacity.size(), 0.0);

    // determine set of hubs that would be open in this solution
    // and set their self-assignment as well as their remaining capacity
    vector<int> open_hubs;
    for (int i = 0; i < (int)assign.size(); i++)
    {
        if (assign[i] == i)
        {
            if (i == k)
            {
                open_hubs.push_back(l);
                new_assign[l] = l;

                if (probdata->with_capacities)
                {
                    auto hub = dynamic_cast<Hub *>(probdata->nodes[l]);
                    new_remaining_capacity[l] = hub->capacity - hub->in_demand;
                }
                else
                {
                    new_remaining_capacity[l] = SCIPinfinity(scip_);
                }
            }
            else
            {
                open_hubs.push_back(i);
                new_assign[i] = i;

                if (probdata->with_capacities)
                {
                    auto hub = dynamic_cast<Hub *>(probdata->nodes[i]);
                    new_remaining_capacity[i] = hub->capacity - hub->in_demand;
                }
                else
                {
                    new_remaining_capacity[i] = SCIPinfinity(scip_);
                }
            }
        }
    }

    // determine assignment of every non-hub node -  try to assign to cheapest possible hub
    for (int i = 0; i < (int)assign.size(); i++)
    {
        if (new_assign[i] == -1) // not yet assigned
        {
            int min_hub = -1;
            double min_cost = SCIPinfinity(scip_);
            for (auto hub : open_hubs)
            {
                double cost = assignment_cost[i][hub];
                if (cost < min_cost && probdata->still_possible_assignments[i][hub] && (new_remaining_capacity[hub] > probdata->nodes[i]->in_demand))
                {
                    min_cost = cost;
                    min_hub = hub;
                }
            }
            // after this switch, the customer could not be assigned to any hub
            // should only happen if the hubs are capacitated i think
            if (min_hub == -1)
            {
                return SCIPinfinity(scip_);
            }

            new_assign[i] = min_hub;
            new_remaining_capacity[min_hub] -= probdata->nodes[i]->in_demand;
        }
    }

    // determine difference between new and old cost
    double new_cost = calculateAssignmentCost(new_assign);

    return new_cost;
}

double Matheuristic::calculateAssignmentCost(vector<int> &assign)
{
    double cost = 0.0;
    for (int i = 0; i < (int)assign.size(); i++)
    {
        int k = assign[i];
        cost += assignment_cost[i][k]; // cost from assignment of node
        for (int j = 0; j < (int)assign.size(); j++)
        {
            int l = assign[j];
            cost += transfer_cost[k][l] * demand[i][j]; // cost from transfers of shipments originating from this node
        }
    }
    return cost;
}

void Matheuristic::updateRemainingCapacities()
{
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);

    remaining_capacity = vector<double>(probdata->nodes.size(), 0.0);
    if (probdata->with_capacities)
    {
        for (auto hub : probdata->still_possible_hubs)
        {
            int k = hub->node_id;
            if (assign[k] == k)
            {
                remaining_capacity[k] = hub->capacity;

                // decrease for all assigned
                for (auto node : hub->still_allowed_nodes)
                {
                    int i = node->node_id;
                    if (assign[i] == k)
                    {
                        remaining_capacity[k] -= node->in_demand;
                    }
                }
                assert(remaining_capacity[k] >= -SCIPfeastol(scip_));
            }
            else
            {
                remaining_capacity[k] = 0;
            }
        }
    }
    else
    {
        for (auto hub : probdata->still_possible_hubs)
        {
            int k = hub->node_id;
            remaining_capacity[k] = SCIPinfinity(scip_);
        }
    }
}

SCIP_RETCODE submitSolution(SCIP *scip, SCIP_HEUR *heur, vector<int> &assign)
{

    SCIP_Bool stored;
    SCIP_SOL *sol;
    vector<SCIP_VAR *> vars;
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip);
    vector<double> vals = vector<double>(probdata->nodes.size(), 1.0);

    // set assignment variables
#ifdef MORE_DEBUG
    SCIPdebugMsg(scip_, "heuristic solution:\n");
#endif
    double total_assignment_cost = 0;
    for (size_t i = 0; i < probdata->nodes.size(); ++i)
    {
        auto node = probdata->nodes[i];
        Hub *hub = dynamic_cast<Hub *>(probdata->nodes[assign[i]]);
        SCIP_VAR *var = probdata->z_vars[{node, hub}];
        assert(var != NULL);
        vars.push_back(var);
#ifdef MORE_DEBUG
        SCIPdebugMsg(scip_, "z_%d_%d = %f\n", (int)node->node_id, (int)hub->node_id, vals[i]);
#endif
        total_assignment_cost += SCIPvarGetObj(var);
    }

    // set auxiliary variables
    BendersSAHLP *benders = dynamic_cast<BendersSAHLP *>(SCIPfindObjConshdlr(scip, "BendersSAHLP"));
    double total_aux_val = 0;
    for (int idx = 0; idx < (int)benders->n_subproblems; ++idx)
    {
        vars.push_back(benders->auxvars[idx]);
        // obtain value from assignments
        double val = 0.0;
        for (auto [node_i, node_j] : benders->node_pairs[idx])
        {
            int i = node_i->node_id;
            int j = node_j->node_id;
            int k = assign[i];
            int l = assign[j];
            val += probdata->demands[i][j] * probdata->transfer_costs[k][l] + probdata->demands[j][i] * probdata->transfer_costs[l][k];
        }
        vals.push_back(val / benders->auxobj); // scale with objective value
        total_aux_val += val;
    }
    SCIPdebugMsg(scip, "heuristic sol val %f + %f = %f\n", total_assignment_cost, total_aux_val, total_assignment_cost + total_aux_val);

    SCIP_CALL(SCIPcreateSol(scip, &sol, heur));
    SCIP_CALL(SCIPsetSolVals(scip, sol, vars.size(), vars.data(), vals.data()));

    // we assume that we properly constructed this solution and do not need to check it with our benders cuts
    benders->registerCheckedSolution(sol);

    SCIP_CALL(SCIPtrySol(scip, sol, TRUE, FALSE, FALSE, FALSE, TRUE, &stored));
    assert(stored);

    SCIP_CALL(SCIPfreeSol(scip, &sol));

    return SCIP_OKAY;
}

SCIP_DECL_HEURINIT(Matheuristic::scip_init)
{
    // set up data for fast access during heuristic execution
    ProbDataSAHLP *probdata = SCIPgetObjProbDataSAHLP(scip_);
    int n_nodes = probdata->nodes.size();
    assignment_cost = probdata->assignment_costs;
    transfer_cost = probdata->transfer_costs;
    demand = probdata->demands;

    // initialize assignment vector
    assign = vector<int>(n_nodes, 0);
    return SCIP_OKAY;
}