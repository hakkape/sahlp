// #define SCIP_DEBUG
#include <fstream>
#include <string>
#include <sstream>
#include "reader_sahlp.hpp"
#include "probdata_sahlp.hpp"

using std::getline;
using std::ifstream;
using std::string;
using std::stringstream;
using std::vector;

SCIP_DECL_READERREAD(ReaderHLP::scip_read)
{
    ifstream file;
    string line;
    stringstream line_stream;
    bool all_assignments_possible;
    int i;
    int n_nodes;
    int n_hubs;
    *result = SCIP_DIDNOTRUN;
    ProbDataSAHLP *probdata;

    /* open file */
    file.open(filename);
    if (!file.is_open())
    {
        SCIPinfoMessage(scip, NULL, "cannot open file <%s>\n", filename);
        return SCIP_READERROR;
    }

    /* read number of nodes and potential hubs */
    getline(file, line);
    n_nodes = stoi(line);
    getline(file, line);
    all_assignments_possible = (line == "ALL");
    if (all_assignments_possible)
    {
        n_hubs = n_nodes;
    }
    else
    {
        n_hubs = stoi(line);
    }

    // create probdata object
    probdata = new ProbDataSAHLP(scip);

    // insert hubs and assignments
    if (all_assignments_possible)
    {
        for (i = 0; i < n_nodes; i++)
        {
            Hub *hub = new Hub();
            hub->node_id = i;
            probdata->hubs.push_back(hub);
            probdata->nodes.push_back(hub);
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
    }
    else
    {
        // create list of nodes with nullpointers
        // insert hubs at appropriate positions
        // replace remaining nullpointers with customers
        probdata->nodes = vector<Node *>(n_nodes, NULL);

        // read possible hubs
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        for (int i = 0; i < n_hubs; i++)
        {
            int hub_id;
            line_stream >> hub_id;
            Hub *hub = new Hub();
            hub->node_id = hub_id;
            probdata->hubs.push_back(hub);
            probdata->nodes[hub_id] = hub;
        }

        // add missing customers nodes
        for (int i = 0; i < n_nodes; i++)
        {
            if (probdata->nodes[i] == NULL)
            {
                Node *node = new Node();
                node->node_id = i;
                probdata->nodes[i] = node;
            }
        }

        /* set allowed assignments */
        for (i = 0; i < n_nodes; i++)
        {
            getline(file, line);
            std::istringstream new_stream(line);
            Node *node = probdata->nodes[i];
            vector<int> allowed_hubs((std::istream_iterator<int>(new_stream)), std::istream_iterator<int>());
            for (auto k : allowed_hubs)
            {
                Hub *hub = dynamic_cast<Hub *>(probdata->nodes[k]);
                assert(hub != NULL);
                node->potential_hubs.push_back(hub);
                hub->potential_nodes.push_back(node);
            }
        }
    }

    /* read distances */
    // they are used in the HLPS reader to calculate the assignment costs and transfer costs
    distances = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));

    for (int i = 0; i < n_nodes; i++)
    {
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        for (int j = 0; j < n_nodes; j++)
        {
            line_stream >> distances[i][j];
        }
    }

    /* read demands */
    probdata->demands = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (int i = 0; i < n_nodes; i++)
    {
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        for (int j = 0; j < n_nodes; j++)
        {
            line_stream >> probdata->demands[i][j];
        }
    }

    /* try to read capacities for facilities or "open n out of subset"*/
    probdata->with_capacities = false;
    bool reached_file_end = false;
    while (!reached_file_end)
    {
        getline(file, line);
        if (file) // file end not yet reached
        {
            // are we reading capacities or OPEN constraints?
            if (line.rfind("OPEN", 0) == 0) // starts with open
            {
                line_stream.clear();
                line_stream << line;
                line_stream.ignore(4, ' '); // ignore OPEN
                int n_hubs_to_open;
                int n_hubs_in_subset;
                line_stream >> n_hubs_to_open;
                line_stream >> n_hubs_in_subset;
                vector<Hub *> subset;
                for (int i = 0; i < n_hubs_in_subset; i++)
                {
                    int hub_id;
                    line_stream >> hub_id;
                    Hub *hub = dynamic_cast<Hub *>(probdata->nodes[hub_id]);
                    assert(hub != NULL);
                    subset.push_back(hub);
                }
                probdata->open_n_of_subset_constraints.push_back({subset, n_hubs_to_open});
            }
            else
            {
                probdata->with_capacities = true;
                line_stream.clear();
                line_stream << line;
                for (int i = 0; i < n_hubs; i++)
                {
                    line_stream >> probdata->hubs[i]->capacity;
                }
            }
        }
        else
        {
            reached_file_end = true;
        }
    }

    file.close();

    SCIP_CALL(SCIPcreateObjProb(scip, "sahlp", probdata, TRUE));
    SCIPverbMessage(scip, SCIP_VERBLEVEL_DIALOG, NULL, "succesfully read HLP instance, still need .hlps file (%s)\n", filename);
    *result = SCIP_SUCCESS;

    return SCIP_OKAY;
}

SCIP_DECL_READERREAD(ReaderHLPS::scip_read)
{
    ifstream file;
    string line;
    ProbDataSAHLP *probdata = dynamic_cast<ProbDataSAHLP *>(SCIPgetObjProbData(scip));
    ReaderHLP *sahlpreader = dynamic_cast<ReaderHLP *>(SCIPfindObjReader(scip, "hlpreader"));
    assert(probdata != NULL);
    assert(sahlpreader != NULL);

    double c_collect, c_transfer, c_distribute;

    *result = SCIP_DIDNOTRUN;

    /* open file */
    file.open(filename);
    if (!file.is_open())
    {
        SCIPinfoMessage(scip, NULL, "cannot open file <%s>", filename);
        return SCIP_READERROR;
    }

    getline(file, line);

    probdata->p = stoi(line);
    probdata->with_p_constraint = true;

    getline(file, line);
    c_collect = stod(line);

    getline(file, line);
    c_transfer = stod(line);

    getline(file, line);
    c_distribute = stod(line);

    file.close();

    /* finish building the access and transfer networks */
    // calculate assignment cost
    vector<vector<double>> &distances = sahlpreader->distances;
    int n_nodes = probdata->nodes.size();
    probdata->assignment_costs = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (auto node_i : probdata->nodes)
    {
        int i = node_i->node_id;
        node_i->in_demand = 0;
        node_i->out_demand = 0;
        node_i->total_demand = 0;
        for (auto node_j : probdata->nodes)
        {
            int j = node_j->node_id;
            node_i->in_demand += probdata->demands[j][i];
            node_i->out_demand += probdata->demands[i][j];
        }
        node_i->total_demand = node_i->in_demand + node_i->out_demand;

        for (auto hub_k : node_i->potential_hubs)
        {
            int k = hub_k->node_id;
            probdata->assignment_costs[i][k] = node_i->in_demand * c_distribute * distances[k][i] + node_i->out_demand * c_collect * distances[i][k];
        }
    }
    // calculate transfer costs
    probdata->transfer_costs = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (auto hub_k : probdata->hubs)
    {
        int k = hub_k->node_id;
        for (auto hub_l : probdata->hubs)
        {
            int l = hub_l->node_id;
            probdata->transfer_costs[k][l] = c_transfer * distances[k][l];
        }
    }

    *result = SCIP_SUCCESS;
    SCIPverbMessage(scip, SCIP_VERBLEVEL_DIALOG, NULL, "succesfully read HLPS file (%s)\n", filename);
    SCIP_CALL(probdata->buildMasterProblem());

    return SCIP_OKAY;
}

/**
 * @brief reading method for SAHLP files. Sets the result pointer to SCIP_SUCESS it file was read and SCIP_READERROR if not.
 *
 * @return SCIP_OKAY
 *
 */
SCIP_DECL_READERREAD(ReaderSAHLP::scip_read)
{
    ifstream file;
    string line;
    stringstream line_stream;
    int n_nodes;
    int n_hubs;
    *result = SCIP_DIDNOTRUN;
    ProbDataSAHLP *probdata = new ProbDataSAHLP(scip);

    /* open file */
    file.open(filename);
    if (!file.is_open())
    {
        SCIPinfoMessage(scip, NULL, "cannot open file <%s>\n", filename);
        return SCIP_READERROR;
    }

    /* read number of nodes and potential hubs */
    getline(file, line);
    line_stream << line;
    line_stream >> n_nodes;
    line_stream >> n_hubs;
    line_stream >> probdata->p;
    probdata->with_p_constraint = true;

    // insert hubs and assignments
    // - create list of nodes with nullpointers
    // - insert hubs at appropriate positions
    // - replace remaining nullpointers with customers
    probdata->nodes = vector<Node *>(n_nodes, NULL);

    // read possible hubs
    getline(file, line);
    line_stream.clear();
    line_stream << line;
    for (int i = 0; i < n_hubs; i++)
    {
        int hub_id;
        line_stream >> hub_id;
        Hub *hub = new Hub();
        hub->node_id = hub_id;
        probdata->hubs.push_back(hub);
        probdata->nodes[hub_id] = hub;
    }

    // add missing customers nodes
    for (int i = 0; i < n_nodes; i++)
    {
        if (probdata->nodes[i] == NULL)
        {
            Node *node = new Node();
            node->node_id = i;
            probdata->nodes[i] = node;
        }
    }

    // read demand matrix
    probdata->demands = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (int i = 0; i < n_nodes; i++)
    {
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        Node *node = probdata->nodes[i];
        node->in_demand = 0;
        node->out_demand = 0;

        for (int j = 0; j < n_nodes; j++)
        {
            line_stream >> probdata->demands[i][j];
            node->in_demand += probdata->demands[j][i];
            node->out_demand += probdata->demands[i][j];
        }
    }

    // read hub to hub transfer cost matrix
    probdata->transfer_costs = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (int i = 0; i < n_hubs; i++)
    {
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        int k = probdata->hubs[i]->node_id;
        for (int j = 0; j < n_hubs; j++)
        {
            int l = probdata->hubs[j]->node_id;
            line_stream >> probdata->transfer_costs[k][l];
        }
    }

    // read allowed assignments and cost
    // - lines have format N_ALLOWED HUB_1 COST_1 HUB_2 COST_2...
    // - first the number of hubs that current customer can be assigned to
    // - then the allowed hub and respective cost etc.
    int n_allowed;
    int hub_id;
    double cost;
    probdata->assignment_costs = vector<vector<double>>(n_nodes, vector<double>(n_nodes, 0));
    for (int i = 0; i < n_nodes; i++)
    {
        getline(file, line);
        line_stream.clear();
        line_stream << line;
        line_stream >> n_allowed;
        for (int j = 0; j < n_allowed; j++)
        {
            line_stream >> hub_id;
            line_stream >> cost;
            Hub *hub = dynamic_cast<Hub *>(probdata->nodes[hub_id]);
            assert(hub != NULL);
            probdata->nodes[i]->potential_hubs.push_back(hub);
            hub->potential_nodes.push_back(probdata->nodes[i]);
            probdata->assignment_costs[i][hub_id] = cost;
        }
    }

    /* try to read capacities for facilities or "open n out of subset"*/
    probdata->with_capacities = false;
    bool reached_file_end = false;
    while (!reached_file_end)
    {
        getline(file, line);
        if (file) // file end not yet reached
        {
            // are we reading capacities or OPEN constraints?
            if (line.rfind("OPEN", 0) == 0) // starts with open
            {
                line_stream.clear();
                line_stream << line;
                line_stream.ignore(4, ' '); // ignore OPEN
                int n_hubs_to_open;
                int n_hubs_in_subset;
                line_stream >> n_hubs_to_open;
                line_stream >> n_hubs_in_subset;
                vector<Hub *> subset;
                for (int i = 0; i < n_hubs_in_subset; i++)
                {
                    int hub_id;
                    line_stream >> hub_id;
                    Hub *hub = dynamic_cast<Hub *>(probdata->nodes[hub_id]);
                    assert(hub != NULL);
                    subset.push_back(hub);
                }
                probdata->open_n_of_subset_constraints.push_back({subset, n_hubs_to_open});
            }
            else
            {
                probdata->with_capacities = true;
                line_stream.clear();
                line_stream << line;
                for (int i = 0; i < n_hubs; i++)
                {
                    line_stream >> probdata->hubs[i]->capacity;
                }
            }
        }
        else
        {
            reached_file_end = true;
        }
    }

    file.close();

    SCIP_CALL(SCIPcreateObjProb(scip, "sahlp", probdata, TRUE));
    SCIP_CALL(probdata->buildMasterProblem());
    SCIPverbMessage(scip, SCIP_VERBLEVEL_DIALOG, NULL, "succesfully read SAHLP file (%s), ready to solve!\n", filename);
    *result = SCIP_SUCCESS;

    return SCIP_OKAY;
}
