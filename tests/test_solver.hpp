#pragma once
#include "gtest/gtest.h"
#include "scip/scip.h"

// tuple of instance filename and double of solution value
typedef std::tuple<const char *, const char *, double> hlpinstance;
typedef std::tuple<const char *, double> sahlpinstance;

class SolverHLPSTest : public ::testing::TestWithParam<hlpinstance>
{
protected:
    SolverHLPSTest();

    virtual ~SolverHLPSTest();

    SCIP *scip;
};
class SolverSAHLPTest : public ::testing::TestWithParam<sahlpinstance>
{
protected:
    SolverSAHLPTest();

    virtual ~SolverSAHLPTest();

    SCIP *scip;
};