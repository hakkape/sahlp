#pragma once
#include "objscip/objscip.h"
#include "solver_sahlp.hpp"
#include <boost/unordered/unordered_map.hpp>
#include <tuple>
#include <vector>

class Hub; // forward declaration, since Node and Hub depend on each other

/**
 * @brief basic node class, used for customers and hubs
 *
 */
class Node
{
public:
    size_t node_id; // nodes are enumerated from 0 to |I|-1
    std::vector<Hub *> potential_hubs;
    std::vector<Hub *> still_allowed_hubs; // hubs k where the assignment variable z_ik has not been fixed to zero yet
    double in_demand;                      // sum of demands arriving at this node
    double out_demand;                     // sum of demands starting at this node
    double total_demand;                   // sum of ingoing and outgoing demands
    virtual ~Node(){};
};

/**
 * @brief hub class derived from node class, additionally stores capacities and a list of nodes that can be assigned to this hub
 *
 */
class Hub : public Node
{
public:
    std::vector<Node *> potential_nodes;
    std::vector<Node *> still_allowed_nodes; // customers i where the assignment variable z_ik has not been fixed to zero yet
    double capacity;
};

/**
 * @brief problem data class stores all the data that describes the instance.
 * also provides the functionality to construct the MIP.
 *
 */
class ProbDataSAHLP : public scip::ObjProbData
{
public:
    /**
     * @brief construct a new problem data object for the single assignment hub location problem
     *
     * use this constructor to read in data after creation
     *
     * @param scip
     */
    ProbDataSAHLP(
        SCIP *scip) : scip(scip){};

    /**
     * @brief constructs a new problem data object for the single assignment hub location problem based on a given instance
     *
     * @param scip
     * @param ins
     */
    ProbDataSAHLP(SCIP *scip, const sahlp::Instance *ins);

    /**
     * @brief destroys the problem data object and frees all its memory
     *
     */
    virtual ~ProbDataSAHLP(){};

    /**
     * @brief callback that frees variables when original problem is deleted
     *
     * @param scip
     * @return SCIP_RETCODE
     */
    virtual SCIP_RETCODE scip_delorig(SCIP *scip);

    /**
     * @brief callback to create copy of user problem data for transformed problem
     *
     * @param scip
     * @param objprobdata
     * @param deleteobject
     * @return SCIP_RETCODE
     */
    virtual SCIP_RETCODE scip_trans(
        SCIP *scip,
        ObjProbData **objprobdata,
        SCIP_Bool *deleteobject);

    /**
     * @brief callback to delete problem data in transformed problem
     *
     * @param scip
     * @return SCIP_RETCODE
     */
    virtual SCIP_RETCODE scip_deltrans(SCIP *scip);

    // data structures
    SCIP *scip;
    std::vector<Node *> nodes;                                                    // nodes in ascending order of their id, from 0 to |I|-1
    std::vector<Hub *> hubs;                                                      // hubs in ascending order of their node ids
    std::vector<std::vector<double>> demands;                                     // demand of each customer pair
    std::vector<std::vector<double>> transfer_costs;                              // per volume transfer cost between two hubs k and l
    std::vector<std::vector<double>> assignment_costs;                            // cost of assigning a customer to a hub
    int p;                                                                        // number of hubs to open at most
    std::vector<std::pair<std::vector<Hub *>, int>> open_n_of_subset_constraints; // constraints forcing a specific number of hubs from a given subset to be opened

    // problem type parameters
    bool with_p_constraint;
    bool with_capacities;

    // model variables
    boost::unordered_map<std::tuple<Node *, Hub *>, SCIP_VAR *> z_vars; // z_{i,k} = 1 iff node i is assigned to hub k
    std::vector<std::vector<SCIP_VAR *>> z_vars_vector;                 // same variables as in the map but for faster access based on indices (signficant speed-up in construction of Benders' subproblems)
    boost::unordered_map<Hub *, SCIP_VAR *> open_hub_vars;              // z_{kk}

    // solving process data
    int n_fixed = 0;                                           // how many variables have been fixed globally
    int n_updates = 0;                                         // how often the list of allowed assignments has been updated
    std::vector<std::vector<bool>> still_possible_assignments; // still_possible_assignments[i][k] = true if node i can still be assigned to hub k
    std::vector<Hub *> still_possible_hubs;                    // list of hubs that can still be opened

    // functions
    /**
     * @brief construct the Benders' master problem
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE buildMasterProblem();

    /**
     * @brief add the z_ik assignment variables to the master problem
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addAssignmentVariables();

    /**
     * @brief  add constraint that each node is assigned to exactly one hub
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addAssignmentConstraints();

    /**
     * @brief add linking constraints, i.e., if node i is assigned to hub k, then hub k must be open
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addLinkingConstraints();

    /**
     * @brief add constraint that at most p many hubs shall be opened
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addPConstraint();

    /**
     * @brief add constraints limiting the capacity of each hub
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addCapacityConstraint();

    /**
     * @brief adds constraints forcing a specific number of hubs from a given subset to be opened
     * allows scenarios like `close one of the existing hubs and open one other hub`
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE addOpenNofSubsetConstraints();

    /**
     * @brief releases variables
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE freeVariables();

    /**
     * @brief get the solution object after solving
     *
     * @return Solution*
     */
    sahlp::Solution *getSolution();

    /**
     * @brief updates the list of hubs that a node can be assigned to at the current branch and bound node
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE updateAllowedHubs();

    /**
     * @brief check if assignment variables have been fixed since the last time this function was called
     *
     * @return true
     * @return false
     */
    bool updateNecessary();
};

/**
 * @brief convenience function to get the problem data object from SCIP
 *
 * @param scip
 * @return ProbDataSAHLP*
 */
ProbDataSAHLP *SCIPgetObjProbDataSAHLP(SCIP *scip);