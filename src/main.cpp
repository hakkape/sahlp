#include "scip/scip.h"
#include "solver_sahlp.hpp"

int main(int argc, char **argv)
{
    sahlp::runSAHLPShell(argc, argv);
    return 0;
}