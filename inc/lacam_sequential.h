#pragma once
#include "MAPFSolver.h"

class LaCAMSequential:
    public MAPFSolver
{
public:
    LaCAMSequential(BasicGraph& G, SingleAgentSolver& path_planner);
    ~LaCAMSequential();

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State>& starts,
             const vector< vector<pair<int, int> > >& goal_locations,
             int time_limit);

    string get_name() const {return "LaCAMSequential"; }

    void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}
	void clear();
private:

};
