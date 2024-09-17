#pragma once
#include "scip/scip.h"
#include "objscip/objscip.h"
#include <vector>

/**
 * @brief SCIP reader plugin to read HLP files (define allowed allowed assignments, distances and demands)
 *
 */
class ReaderHLP : public scip::ObjReader
{
public:
    /**
     * @brief Construct a new reader for .hlp files
     *
     * @param scip to which to add the reader
     */
    ReaderHLP(
        SCIP *scip) : scip::ObjReader(scip, "hlpreader", "file reader for `.hlp` files", "hlp"){};

    virtual ~ReaderHLP(){};

    /**
     * @brief callback to read a partial problem description from a .hlp file (afterwards still need to read .hlps file)
     *
     */
    virtual SCIP_DECL_READERREAD(scip_read);

    /**
     * @brief Writing callback (not needed)
     *
     */
    virtual SCIP_DECL_READERWRITE(scip_write)
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    };

    std::vector<std::vector<double>> distances; // temporary storage of distances matrix until .hlps file is read
};

/**
 * @brief SCIP reader plugin to read HLPs files (define number of hubs to open and cost parameters)
 *
 */
class ReaderHLPS : public scip::ObjReader
{
public:
    /**
     * @brief Construct a new reader for .hlps files
     *
     * @param scip to which to add the reader
     */
    ReaderHLPS(
        SCIP *scip) : scip::ObjReader(scip, "hlpsreader", "file reader for `.hlps` files", "hlps"){};
    virtual ~ReaderHLPS(){};

    /**
     * @brief callback to read a partial problem description from a .hlps file (need to read a .hlp file first)
     *
     */
    virtual SCIP_DECL_READERREAD(scip_read);

    /**
     * @brief writing callback (not needed)
     *
     */
    virtual SCIP_DECL_READERWRITE(scip_write)
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    };
};

/**
 * @brief SCIP reader plugin for .sahlp files that fully define an instance
 *
 */
class ReaderSAHLP : public scip::ObjReader
{
public:
    /**
     * @brief Construct a new reader for .sahlp files
     *
     * @param scip to which to add the reader
     */
    ReaderSAHLP(
        SCIP *scip) : scip::ObjReader(scip, "sahlpreader", "file reader for `.sahlp` files", "sahlp"){};

    virtual ~ReaderSAHLP(){};

    /**
     * @brief callback to read a full problem description from a .sahlp file
     *
     */
    virtual SCIP_DECL_READERREAD(scip_read);

    /**
     * @brief writing callback (not needed)
     *
     */
    virtual SCIP_DECL_READERWRITE(scip_write)
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    };
};
