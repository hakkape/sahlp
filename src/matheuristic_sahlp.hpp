#pragma once
#include "objscip/objscip.h"
#include "scip/scip.h"
#include "probdata_sahlp.hpp"
#include <vector>

/**
 * @brief two-stage Matheuristic from the paper by Zetina et al.
 * 
 */
class Matheuristic : public scip::ObjHeur
{
public:
    /**
     * @brief Constructs a new Matheuristic object.
     * 
     * @param scip 
     */
    Matheuristic(SCIP *scip) : scip::ObjHeur(scip, "matheuristic", "Matheuristic from Zetina et al.", 'c', 100000000, 0, 0, 0, SCIP_HEURTIMING_DURINGLPLOOP, TRUE){};
    virtual ~Matheuristic(){};

    /**
     * @brief heuristic execution callback of Matheuristic.
     * 
     */
    virtual SCIP_DECL_HEUREXEC(scip_exec);
    
    /**
     * @brief initialization callback of Matheuristic. 
     * 
     */
    virtual SCIP_DECL_HEURINIT(scip_init);

private:
    std::vector<std::vector<double>> assignment_cost;
    std::vector<std::vector<double>> transfer_cost;
    std::vector<std::vector<double>> demand;
    std::vector<double> remaining_capacity; // remaining capacity of each hub
    std::vector<int> assign; // current assignment in heuristic solution

    /**
     * @brief compares if the support of the hub opening variables has changed since the last time the heuristic was executed
     *
     * @return true
     * @return false
     */
    bool supportChanged();

    /**
     * @brief solves the facility location problem on the reduced instance and initializes the assignments
     *
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE solveFacilityModel();

    /**
     * @brief performs local search on the current solution
     *
     */
    void localSearch();

    /**
     * @brief tries swapping pairs of nodes
     *
     * @param current_cost cost of the current solution, may be adjusted if a better solution is found
     * @return true improved solution
     * @return false did not improve solution
     */
    bool swapNodes(double &current_cost);

    /**
     * @brief tries re-assigning single nodes
     *
     * @param current_cost cost of the current solution, may be adjusted if a better solution is found
     * @return true improved solution
     * @return false did not improve solution
     */
    bool shiftNodes(double &current_cost);

    /**
     * @brief closes one and opens another hub
     *
     * @param current_cost cost of the current solution, may be adjusted if a better solution is found
     * @return true improved solution
     * @return false did not improve solution
     */
    bool switchHubs(double &current_cost);

    /**
     * @brief cost difference of swapping i -> k, j-> l to i -> l, j -> k
     *
     * @param i
     * @param j
     * @param k
     * @param l
     * @return double positive value if swap is beneficial
     */
    double deltaSwap(int i, int j, int k, int l);

    /**
     * @brief cost difference for changing assignment of i from k to l
     *
     * @param i
     * @param k
     * @param l
     * @return double positive value if swap is beneficial
     */
    double deltaShift(int i, int k, int l);

    /**
     * @brief cost after closing k, opening l and assigning all customers to their closest hub
     *
     * @param k
     * @param l
     * @param new_assign returns the assignments resulting from the switch
     * @return double the new cost of the solution after switching
     */
    double costAfterSwitch(int k, int l, std::vector<int> &new_assign);

    /**
     * @brief calculates the cost of the passed assignments (assignment costs + transfer costs)
     *
     * @param assign
     * @return double
     */
    double calculateAssignmentCost(std::vector<int> &assign);
    
    /**
     * @brief updates the remaining capacity of each hub (0 if hub currently closed)
     * in case the instance has unlimited capacities, sets the capacity value to some very large value
     * 
     */
    void updateRemainingCapacities();

    std::vector<Hub *> support; // hubs currently in the support
};

/**
 * @brief stores solution given as an assignment vector
 *
 * @param scip pointer to scip instance
 * @param heur pointer to heuristic that created the solution
 * @param assign assignment vector, such that assign[i] = k
 * @return SCIP_RETCODE
 */
SCIP_RETCODE submitSolution(SCIP* scip, SCIP_HEUR *heur, std::vector<int> &assign);