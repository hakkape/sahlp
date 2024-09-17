#pragma once
#include "cons_benders_sahlp.hpp"
#include "objscip/objscip.h"
#include <vector>


/**
 * @brief constraint handler for optimality cuts from fractional solutions
 * 
 */
class BenderslpSAHLP : public BendersSAHLP
{
public:
    /**
     * @brief Construct a new benders decomposition constraint handler for the single assignment hub location problem
     * handles the generation of optimality cuts
     *
     * @param scip
     */
    BenderslpSAHLP(
        SCIP *scip);

    /**
     * @brief enforce LP solution callback. separates cuts from LP solutions.
     *
     */
    virtual SCIP_DECL_CONSENFOLP(scip_enfolp);

    /**
     * @brief enforce pseudo solution callback. separates cuts from pseudo solutions.
     *
     */
    virtual SCIP_DECL_CONSENFOPS(scip_enfops);

    /**
     * @brief solution checking callback. does nothing as we check integral solutions in the integral benders constraint handler
     *
     */
    virtual SCIP_DECL_CONSCHECK(scip_check);

    /**
     * @brief variable lock callback. does nothing as integral constraint handler takes care of variable locks
     *
     */
    virtual SCIP_DECL_CONSLOCK(scip_lock);

    /**
     * @brief presolving initialization method. gets the auxiliary variables from the integral benders constraint handler
     *
     */
    virtual SCIP_DECL_CONSINITPRE(scip_initpre);

    int lpfreq; // frequency of enforcing LP solutions (0: only at root node, 1: at every node (default))
};

/**
 * @brief interface method to include the benderslp constraint handler
 *
 * @param scip
 * @return SCIP_RETCODE
 */
SCIP_RETCODE SCIPincludeConshdlrBenderslpSAHLP(SCIP *scip);

/**
 * @brief setup method to prepare the necessary data structures for the benders constraint handler.
 *
 * @param scip
 * @param node_pairs list of node pairs for each subproblem
 * @return SCIP_RETCODE
 */
SCIP_RETCODE SCIPsetupBenderslp(
    SCIP *scip,
    std::vector<std::vector<std::pair<Node *, Node *>>> node_pairs);