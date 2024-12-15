#include "lacam_sequential.h"
#include <lacam.hpp>
#include <algorithm>

LaCAMSequential::LaCAMSequential(BasicGraph& G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner) {
    lacam::Planner::FLG_STAR = false;
    lacam::Planner::PIBT_NUM = 1;
    // lacam::Planner::FLG_MULTI_THREAD = false;
    lacam::Planner::FLG_REFINER = false;
    lacam::Planner::FLG_SCATTER = false;
    // lacam::Planner::RANDOM_INSERT_PROB1 = 0.0;
    // lacam::Planner::RANDOM_INSERT_PROB2 = 0.0;
    // lacam::Planner::FLG_RANDOM_INSERT_INIT_NODE = false;
    lacam::Planner::FLG_ALLOW_FOLLOWING = false;
}

bool LaCAMSequential::run(const vector<State>& starts,
                const vector< vector<pair<int, int> > >& goal_locations,
                int time_limit)
{
    clock_t start = std::clock();

    const auto N = starts.size();
    std::vector<int> start_indexes;
    for (const auto& start : starts)
    {
        start_indexes.push_back(start.location);
    }

    std::vector<int> goal_indexes;
    for (const auto& goal_sequence : goal_locations)
    {
        const auto goal = goal_sequence[0];
        goal_indexes.push_back(goal.first);
    }

    auto ins = lacam::Instance(G.map_name + ".map", start_indexes, goal_indexes);
    assert(ins.is_valid(1));
    const auto verbosity = 10;
    const std::optional<int> threshold = std::nullopt; 
    auto deadline = lacam::Deadline(5 * 60 * 1000);  // 5min
    auto lacam_soln = lacam::solve(ins, threshold, verbosity, &deadline, 0);
    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    if (!lacam::is_feasible_solution(ins, lacam_soln, threshold, verbosity)) {
        solution_cost = -1;
        solution_found = false;
        return false;
    }

    solution = std::vector<Path>(N);
    for (size_t t = 0; t < lacam_soln.size(); t++)
    {
        for (size_t i = 0; i < N; i++)
        {
            auto loc = lacam_soln[t][i];
            solution[i].push_back(State(loc->index));
        }
    }
    solution_found = true;
    return true;
}

void LaCAMSequential::save_results(const std::string &fileName, const std::string &instanceName) const
{
//     std::ofstream stats;
//     stats.open(fileName, std::ios::app);
//     stats << runtime << "," <<
//              solution_cost << "," << min_sum_of_costs << "," <<
//              avg_path_length << "," << "0" << "," <<
//              instanceName << std::endl;
//     stats.close();
}

void LaCAMSequential::clear()
{
	runtime = 0;
	solution_found = false;
	solution_cost = -2;
	avg_path_length = -1;
	solution.clear();
	initial_constraints.clear();
	initial_rt.clear();
}