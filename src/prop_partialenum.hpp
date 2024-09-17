#pragma once
#include "scip/scip.h"
#include "objscip/objscip.h"

/**
 * @brief partial enumeration propagator
 * enumerates hub opening and closing decisions, trying to fix them to zero or one
 * 
 */
class PropPartialEnum : public scip::ObjProp
{
public:
    /**
     * @brief constructs a partial enumeration propagator
     *
     * @param scip
     */
    PropPartialEnum(SCIP *scip);
    ~PropPartialEnum(){};

    /**
     * @brief execution callback for propagation
     *
     */
    virtual SCIP_DECL_PROPEXEC(scip_exec);

private:
    /**
     * @brief tries fixing eligible variables to zero or one
     *
     * @param to_zero true if variables should be fixed to zero, false if they should be fixed to one
     * @param result
     * @return SCIP_RETCODE
     */
    SCIP_RETCODE enumerateHubs(bool to_zero, SCIP_RESULT *result);

    double last_gap = 10e10; // optimality gap the last time the propagator was called
};

/**
 * @brief interface method for including the partial enumeration propagator
 *
 * @param scip
 * @return SCIP_RETCODE
 */
SCIP_RETCODE SCIPincludeObjPropPartialEnum(SCIP *scip);