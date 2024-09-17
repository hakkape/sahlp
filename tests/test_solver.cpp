#include "test_solver.hpp"
#include "probdata_sahlp.hpp"
#include "scip/scip.h"
#include "solver_sahlp.hpp"
#include "gtest/gtest.h"

// macro to catch SCIP return codes and make sure everything worked
#define SCIP_TEST(func)                     \
    do                                      \
    {                                       \
        SCIP_RETCODE temp_retcode = (func); \
        ASSERT_EQ(temp_retcode, SCIP_OKAY); \
    } while (0)

ProbDataSAHLP *createTinyInstance(SCIP *scip, bool with_capacities)
{
    ProbDataSAHLP *probdata = new ProbDataSAHLP(scip);
    // set up problem data
    for (int i = 0; i < 4; i++)
    {
        Hub *hub = new Hub();
        hub->node_id = i;
        if (with_capacities)
        {
            if (i == 1)
            {
                hub->capacity = 3;
            }
            else
            {
                hub->capacity = 10000;
            }
        }
        hub->in_demand = 3;
        hub->out_demand = 3;
        hub->total_demand = 6;
        probdata->nodes.push_back(hub);
        probdata->hubs.push_back(hub);
    }
    // all assignments allowed
    for (auto node : probdata->nodes)
    {
        for (auto hub : probdata->hubs)
        {
            node->potential_hubs.push_back(hub);
            hub->potential_nodes.push_back(node);
        }
    }
    std::vector<std::vector<double>> distances = {
        {0, 1, 2, 3},
        {1, 0, 1, 2},
        {2, 1, 0, 1},
        {3, 2, 1, 0}};

    probdata->transfer_costs = std::vector<std::vector<double>>(4, std::vector<double>(4, 0));
    probdata->assignment_costs = std::vector<std::vector<double>>(4, std::vector<double>(4, 0));

    for (auto k : probdata->hubs)
    {
        int k_idx = k->node_id;
        for (auto l : probdata->hubs)
        {
            int l_idx = l->node_id;
            probdata->transfer_costs[k_idx][l_idx] = distances[k_idx][l_idx];
            // pick up and deliviers for 3 demand, collection cost factor 3 and delivery cost factor 2
            probdata->assignment_costs[k_idx][l_idx] = distances[k_idx][l_idx] * 3 * (3 + 2);
        }
    }

    std::vector<std::vector<double>> demands = {
        {0, 1, 1, 1},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {1, 1, 1, 0}};

    probdata->demands = demands;
    probdata->with_p_constraint = true;
    probdata->with_capacities = with_capacities;
    probdata->p = 2;
    return probdata;
}

TEST(ProbdataTest, TestTinyInstanceSolving)
{
    SCIP *scip;
    SCIP_TEST(sahlp::buildSCIPSAHLP(&scip));
    ProbDataSAHLP *probdata = createTinyInstance(scip, false);

    // solve
    SCIP_TEST(SCIPcreateObjProb(scip, "SAHLP", probdata, TRUE));
    SCIP_TEST(probdata->buildMasterProblem());

    SCIP_TEST(SCIPsolve(scip));
    SCIP_TEST(SCIPprintSol(scip, SCIPgetBestSol(scip), NULL, FALSE));
    EXPECT_EQ(SCIPgetStatus(scip), SCIP_STATUS_OPTIMAL);
    EXPECT_NEAR(SCIPgetPrimalbound(scip), 38, 0.01);
    SCIP_TEST(SCIPfree(&scip));
}

TEST(ProbdataTest, TestTinyInstanceSolvingCapacity)
{
    SCIP *scip;
    SCIP_TEST(sahlp::buildSCIPSAHLP(&scip));
    ProbDataSAHLP *probdata = createTinyInstance(scip, true);

    // solve
    SCIP_TEST(SCIPcreateObjProb(scip, "SAHLP", probdata, TRUE));
    SCIP_TEST(probdata->buildMasterProblem());

    SCIP_TEST(SCIPsolve(scip));
    SCIP_TEST(SCIPprintSol(scip, SCIPgetBestSol(scip), NULL, FALSE));
    EXPECT_EQ(SCIPgetStatus(scip), SCIP_STATUS_OPTIMAL);
    EXPECT_NEAR(SCIPgetPrimalbound(scip), 42, 0.01);
    SCIP_TEST(SCIPfree(&scip));
}

// test reading and solving instances
SolverHLPSTest::SolverHLPSTest()
{
    // init scip
    scip = NULL;
    SCIP_CALL_ABORT(sahlp::buildSCIPSAHLP(&scip));
}

SolverHLPSTest::~SolverHLPSTest()
{
    SCIPfree(&scip);
}
SolverSAHLPTest::SolverSAHLPTest()
{
    // init scip
    scip = NULL;
    SCIP_CALL_ABORT(sahlp::buildSCIPSAHLP(&scip));
}

SolverSAHLPTest::~SolverSAHLPTest()
{
    SCIPfree(&scip);
}

hlpinstance tiny = {"test_data/4.hlp", "test_data/tiny.hlps", 38};
hlpinstance ap10 = {"test_data/ap10.hlp", "test_data/p4.hlps", 47030713.5708};
hlpinstance ap20 = {"test_data/ap20.hlp", "test_data/p4.hlps", 59358110.061};
hlpinstance cab30 = {"test_data/cab30.hlp", "test_data/p4.hlps", 707643659};
hlpinstance tr40 = {"test_data/tr40.hlp", "test_data/p4.hlps", 16901536458};
hlpinstance ap50 = {"test_data/ap50.hlp", "test_data/p4.hlps", 63412465.9580266};
hlpinstance cab60 = {"test_data/cab60.hlp", "test_data/p8.hlps", 2926588872};
hlpinstance ll100 = {"test_data/100ll.hlp", "test_data/zetina_p5.hlps", 136929.444191};
hlpinstance ap10r = {"test_data/ap10r.hlp", "test_data/p2.hlps", 141673.32499};
hlpinstance cab50r = {"test_data/cab50r.hlp", "test_data/p4.hlps", 476871.90099818150};
hlpinstance ap10cap = {"test_data/ap10_capacities.hlp", "test_data/p4.hlps", 52579310.5777788};
hlpinstance ap10subset = {"test_data/ap10_open3of4.hlp", "test_data/p4.hlps", 48067897.86107};
hlpinstance ap10manysubset = {"test_data/ap10_manyopencons.hlp", "test_data/p4.hlps", 56917719.9808};
hlpinstance cab60subset = {"test_data/cab60_open5of6.hlp", "test_data/p8.hlps", 3178465814};

sahlpinstance sa_tiny = {"test_data/4p2_312.sahlp", 38};

TEST_P(SolverHLPSTest, testHLPSInstances)
{
    auto [hlp_file, hlps_file, objval] = GetParam();
    sahlp::Solution *sol = sahlp::solveHLPS(hlp_file, hlps_file, NULL);

    EXPECT_NEAR(sol->solution_value, objval, 0.01);
    EXPECT_EQ(sol->status, SCIP_STATUS_OPTIMAL);
}

TEST_P(SolverSAHLPTest, testSAHLPInstances)
{
    auto [sahlp_file, objval] = GetParam();
    sahlp::Solution *sol = sahlp::solveSAHLP(sahlp_file, NULL);

    EXPECT_NEAR(sol->solution_value, objval, 0.01);
    EXPECT_EQ(sol->status, SCIP_STATUS_OPTIMAL);
}

INSTANTIATE_TEST_SUITE_P(testHLPSInstances, SolverHLPSTest, ::testing::Values(tiny, ap10, ap20, cab30, tr40, ap50, cab60, ll100, ap10r, cab50r, ap10cap, ap10subset, ap10manysubset, cab60subset));

INSTANTIATE_TEST_SUITE_P(testSAHLPInstances, SolverSAHLPTest, ::testing::Values(sa_tiny));
