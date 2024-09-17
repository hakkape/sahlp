#pragma once
#include "objscip/objscip.h"
#include "probdata_sahlp.hpp"
#include "scip/scip_timing.h"
#include <utility>
#include <vector>

// constraint handler for benders cuts at integer solutions
// to properly time the separation of cuts from fractional and integer solutions, we need to constraint handlers (just like in the SCIP default Benders implementation)

/**
 * @brief shared data between the two benders constraint handlers
 *
 */
struct BendersSharedData
{
    int n_separation_rounds = 0;
    int n_cuts_generated = 0;
    std::vector<std::vector<double>> corepoint = {{}};
};

/**
 * @brief constraint handler for optimality cuts from integer solutions
 *
 */
class BendersSAHLP : public scip::ObjConshdlr
{
public:
    /**
     * @brief construct a new benders decomposition constraint handler for the single assignment hub location problem
     * handles the generation of optimality cuts from integer solutions
     *
     * @param scip
     */
    BendersSAHLP(
        SCIP *scip);

    /**
     * @brief constructor for child class that overwrites some of the default values
     *
     * @param scip
     * @param name
     * @param desc
     * @param enfopriority
     * @param checkpriority
     */
    BendersSAHLP(
        SCIP *scip,
        const char *name,
        const char *desc,
        const int enfopriority,
        const int checkpriority);

    /**
     * @brief LP enforcement callback. separates cuts from LP solutions.
     *
     */
    virtual SCIP_DECL_CONSENFOLP(scip_enfolp);

    /**
     * @brief pseudo solution enforcement callback. separates cuts from pseudo solutions.
     *
     */
    virtual SCIP_DECL_CONSENFOPS(scip_enfops);

    /**
     * @brief solution checking callback. checks if primal solutions are feasible with respect to this constraint handler
     * If not, a cut is generated and a feasible solution is created and added to the solution pool
     *
     */
    virtual SCIP_DECL_CONSCHECK(scip_check);

    /**
     * @brief variable lock callback. specifies if constraints might become infeasible if variables are rounded up / down
     *
     */
    virtual SCIP_DECL_CONSLOCK(scip_lock);

    /**
     * @brief initialization callback. creates clocks for timekeeping
     *
     */
    virtual SCIP_DECL_CONSINIT(scip_init);

    /**
     * @brief deinitialization callback. frees clocks
     *
     */
    virtual SCIP_DECL_CONSEXIT(scip_exit);

    /**
     * @brief transformation callback. transforms and captures auxiliary variables
     *
     */
    virtual SCIP_DECL_CONSTRANS(scip_trans);

    /**
     * @brief returns the time in seconds that has been spent solving the subproblems
     *
     * @return double
     */
    double getSubproblemSolvingTime();

    /**
     * @brief returns the time in seconds that has been spent setting up the subproblems
     *
     * @return double
     */
    double getSubproblemSetupTime();

    /**
     * @brief returns the time in seconds that has been spent in the callbacks that separate cuts
     *
     * @return double
     */
    double getCallbackTime();

    /**
     * @brief prints the total time spend and time spent on various sub tasks
     *
     */
    void printTimeSpent();

    /**
     * @brief add externally generated solution to list of solutions that do not need to be checked by this constraint handler
     *
     * @param sol
     */
    void registerCheckedSolution(SCIP_SOL *sol);

    size_t n_subproblems;                                           // number of subproblems
    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs; // list of node pairs for each subproblem
    std::vector<SCIP_VAR *> auxvars;                                // auxiliary variables for each subproblem
    std::vector<double> subproblemobjvals;                          // objective values of the subproblems
    BendersSharedData *bendersdata;                                 // shared data between the two benders constraint handlers
    double auxobj;                                                  // sum of objective values of the auxiliary variables

protected:
    /**
     * @brief Given a  solution that is feasible w.r.t. all but this constraint and for which the auxiliary variables are violated, construct a feasible solution by setting the auxiliary variables to the proper values.
     *
     * Note: only call this after cuts have been separated, since this updates the values the auxiliary variables need to be set to
     *
     * @param sol solution to adjust
     * @param type type of the enforcement method that called this function
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE constructValidSolution(
        SCIP_SOL *sol,
        SCIP_BENDERSENFOTYPE type);

    /**
     * @brief Tries to separate stabilized benders cuts for the given solution. If the auxiliary variables are violated, a cut is generated.
     *
     * @param sol solution to check or NULL to check the current LP / pseudo solution
     * @param auxviolated stores whether the auxiliary variables were violated in the solution (values were less than subproblem solution values)
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE separateBendersCuts(
        SCIP_SOL *sol,
        bool *auxviolated);

    /**
     * @brief solves subproblem for given customer pair and stores coefficients in the passed map
     *
     * @param sol solution to check or NULL if LP solution should be checked
     * @param i origin customer
     * @param j destination customer
     * @param coefficients map to store cut coefficients in
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE solveSubproblem(
        SCIP_SOL *sol,
        Node *i,
        Node *j,
        boost::unordered_map<SCIP_VAR *, SCIP_Real> &coefficients);

    /**
     * @brief solves the benders subproblem using the network simplex algorithm of CPLEX
     *
     * @param n_nodes number of nodes in the subproblem
     * @param n_arcs number of arcs in the subproblem
     * @param supply supply of each node
     * @param tail tail node of each arc
     * @param head head node of each arc
     * @param lb lower bound of each arc
     * @param ub upper bound of each arc
     * @param obj objective value of each arc
     * @param dual_sol pointer to store the dual solution in
     * @param numerical_problems whether numerical problems occured during the optimization
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE solveSubproblemCPLEX(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems);

    /**
     * @brief solves the benders subproblem using SCIP
     *
     * @param n_nodes number of nodes in the subproblem
     * @param n_arcs number of arcs in the subproblem
     * @param supply supply of each node
     * @param tail tail node of each arc
     * @param head head node of each arc
     * @param lb lower bound of each arc
     * @param ub upper bound of each arc
     * @param obj objective value of each arc
     * @param dual_sol pointer to store the dual solution in
     * @param numerical_problems whether numerical problems occured during the optimization
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE solveSubproblemSCIP(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems);

    /**
     * @brief solves the benders subproblem using the open source network simplex from https://github.com/frangio68/Min-Cost-Flow-Class
     *
     * @param n_nodes number of nodes in the subproblem
     * @param n_arcs number of arcs in the subproblem
     * @param supply supply of each node
     * @param tail tail node of each arc
     * @param head head node of each arc
     * @param lb lower bound of each arc
     * @param ub upper bound of each arc
     * @param obj objective value of each arc
     * @param dual_sol pointer to store the dual solution in
     * @param numerical_problems whether numerical problems occured during the optimization
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE solveSubproblemMCF(int n_nodes, int n_arcs, double *supply, int *tail, int *head, double *lb, double *ub, double *obj, double *dual_sol, bool& numerical_problems);
    /**
     * @brief enforcement method that solves benders subproblems, adds cuts and if possible also a feasible solution
     *
     * @param sol solution to check or NULL if LP solution should be checked
     * @param result pointer to store the result of the enforcement call
     * @param intsol whether the passed solution is an integer solution
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE enforceSolution(
        SCIP_SOL *sol,
        SCIP_RESULT *result,
        SCIP_Bool intsol,
        SCIP_BENDERSENFOTYPE type);

    /**
     * @brief constructs benders cut for given variable coefficients
     *
     * @param cons pointer to store the generated constraint in
     * @param coefficients coefficients of the assignment variables in the cut
     * @param auxvar auxiliary variable that should be included in the cut
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE constructBendersCut(
        SCIP_CONS **cons,
        boost::unordered_map<SCIP_VAR *, SCIP_Real> &coefficients,
        SCIP_VAR *auxvar);

    /**
     * @brief creates core point based on currently allowed assignments
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE initializeCorePoint();

    /**
     * @brief updates the core point as a convex combination of the current solution and the previous core point
     *
     * @param sol solution to update the core point with
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE updateCorePoint(SCIP_SOL *sol);

    /**
     * @brief returns true if the given solution has been checked by this constraint handler before
     *
     * @param sol solution to check
     * @return true
     * @return false
     */
    bool solutionCheckedBefore(SCIP_SOL *sol);

    std::vector<int> checked_solution_ids; // ids of solutions that have been checked by this constraint handler
    SCIP_CLOCK *subproblem_solving_clock;  // clock for measuring the time spent solving subproblems
    SCIP_CLOCK *subproblem_setup_clock;    // clock for measuring the time spent solving subproblems
    SCIP_CLOCK *callback_clock;            // clock for measuring the time spent overall in the callback
    int n_subproblems_numerical_problems = 0;
};

/**
 * @brief interface method to include this constraint handler in a SCIP instance
 *
 * @param scip
 * @return SCIP_RETCODE
 */
SCIP_RETCODE SCIPincludeConshdlrBendersSAHLP(SCIP *scip);

/**
 * @brief setup method to prepare the necessary data structures for the benders constraint handler.
 *
 * @param scip
 * @param node_pairs list of node pairs for each subproblem
 * @return SCIP_RETCODE
 */
SCIP_RETCODE SCIPsetupBenders(
    SCIP *scip,
    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs);