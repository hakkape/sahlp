#pragma once
#include "scip/scip.h"
#include <vector>

// c interface for python API

/**
 * @brief a pure C struct that represents a solution to the single assignment hub location problem
 *
 */
struct CSolution
{
    int *assigned_hubs; // for each node its assigned hub, e.g., if node 0 is assigned to hub 2, then assigned_hubs[0] == 2
    int n_nodes;
    double solution_value;
    double cpu_time;
};

/**
 * @brief a pure C struct that represents an instance of the single assignment hub location problem
 *
 */
struct CInstance
{
    int n_nodes;
    int n_hubs;
    int *hubs;
    int p;
    double **demands;               // n_nodes x n_nodes demand matrix
    double **transfer_costs;        // n_hubs x n_hubs transfer cost matrix
    int *n_allowed_hubs;            // how many hubs are allowed for each node
    int **allowed_hubs;             // which hubs are allowed for each node
    double **assignment_costs;      // for every allowed assignment the cost
    unsigned int with_capacities;   // whether capacities are given (1 = yes, 0 = no)
    unsigned int with_p_constraint; // whether the p constraint should be used (1 = yes, 0 = no)
    double *capacities;             // hub capacities

    // "OPEN N out of M hubs"
    int n_open_hubs_constraints; // how many of these constraints we have
    int *n_open_hubs;            // how many hubs we want to open
    int *n_candidate_hubs;       // out of how many?
    int **candidate_hubs;        // which hubs are candidates?
};
extern "C"
{

    /**
     * @brief solves the passed instance (in the form of .hlp and .hlps file) and returns the solution as a C struct
     *
     * @param hlp_filename
     * @param hlps_filename
     * @param scip_setting_filename
     * @return CSolution*
     */
    CSolution *solveHLPS_C(const char *hlp_filename, const char *hlps_filename, const char *scip_setting_filename);

    /**
     * @brief solves the passed instance (in the form of a .sahlp file) and returns the solution as a C struct
     *
     * @param sahlp_filename
     * @param scip_setting_filename
     * @return CSolution*
     */
    CSolution *solveSAHLP_C(const char *sahlp_filename, const char *scip_setting_filename);

    /**
     * @brief solves the passed instance (in the form of a C struct) and returns the solution as a C struct
     *
     * @param cins
     * @param scip_setting_filename
     * @return CSolution*
     */
    CSolution *solveSAHLPInstance_C(CInstance *cins, const char *scip_setting_filename);

    /**
     * @brief frees the memory of a C solution
     *
     * @param sol
     */
    void freeCSolution(CSolution *sol);

    /**
     * @brief frees the memory of a C solution
     *
     */
    void freeCInstance(CInstance *ins);
}

// Cpp interface
namespace sahlp
{

    /**
     * @brief a c++ struct that represents an instance of the single assignment hub location problem
     *
     */
    struct Instance
    {
        int n_nodes;
        std::vector<int> hubs;
        std::vector<double> capacities;
        std::vector<std::vector<double>> demands;
        std::vector<std::vector<double>> transfer_costs;
        std::vector<std::vector<std::tuple<int, double>>> allowed_assignments_and_costs; // for every node the allowed hubs and the corresponding assignment costs
        int p;
        bool with_p_constraint;
        bool with_capacities;

        // "OPEN N out of M hubs"
        std::vector<std::tuple<int, std::vector<int>>> open_hubs_constraints;
    };

    /**
     * @brief a C++ struct that represents a solution to the single assignment hub location problem
     *
     */
    struct Solution
    {
        double solution_value;
        double cpu_time;
        SCIP_STATUS status;

        std::vector<int> hubs;          // list of opened hubs
        std::vector<int> assigned_hubs; // for each node its assigned hub
        CSolution *toCSolution();
    };

    /**
     * @brief builds scip instance and includes plugins
     *
     * @param scip
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE buildSCIPSAHLP(SCIP **scip);

    /**
     * @brief solves the passed instance (in the form of .hlp and .hlps file) and returns the solution
     *
     * @param hlp_filename
     * @param hlps_filename
     * @param scip_setting_filename
     * @return Solution*
     */
    Solution *solveHLPS(const char *hlp_filename, const char *hlps_filename, const char *scip_setting_filename);

    /**
     * @brief solves the passed instance (in the form of a .sahlp) and returns the solution
     *
     * @param sahlp_filename
     * @param scip_setting_filename
     * @return Solution*
     */
    Solution *solveSAHLP(const char *sahlp_filename, const char *scip_setting_filename);

    /**
     * @brief solves the instance
     *
     * @param ins
     * @param scip_setting_filename
     * @param quiet
     * @param timelimit in seconds
     * @return Solution*
     */
    Solution *solveSAHLPInstance(const Instance *ins, const char *scip_setting_filename, bool quiet, double timelimit);

    /**
     * @brief main method called when the executable is run, starts interaction with the SCIP terminal
     *
     * @param argc
     * @param argv
     */
    void runSAHLPShell(int argc, char **argv);

    /**
     * @brief converts a c instance to a c++ instance
     *
     * @param cins
     * @return Instance
     */
    Instance *toInstance(const CInstance *cins);

}
