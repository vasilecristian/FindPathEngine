// ConsoleApplication1.cpp : Defines the entry point for the console application.
//



#include "FindPathEngine/FindPathEngine.h"
#include <vector>
#include <memory>
#include <array>
#include <map>



FindPathEngine::FindPathEngine(std::shared_ptr<NavMeshBase> navMesh)
	: m_navMesh(navMesh)
{

}

FindPathEngine::Ticket::Ticket(int startIndex, int goalIndex)
	: m_startIndex(startIndex)
	, m_goalIndex(goalIndex)
	, m_current(nullptr)
	, m_state(State::WAITING)
	, m_steps(0)
{
}

/** Add a new request to determine a path */
void FindPathEngine::AddTicket(std::shared_ptr<Ticket> ticket)
{
	m_tickets.push_back(ticket);
}

/** This function will run and process every Ticket.
* If a ticket is solved (aka the path was found, or there is not solution) just remove the
* ticket from the pending list.
* @return true if pending list with tickets is empty.*/
bool FindPathEngine::Update()
{
	for (auto it = m_tickets.begin(); it != m_tickets.end();)
	{
		if (ProcessTicket((*it)))
			it = m_tickets.erase(it);
		else
			++it;
	}

	return (m_tickets.size() == 0);
}

bool FindPathEngine::ProcessTicket(std::shared_ptr<Ticket> ticket)
{
	ticket->m_steps++;

	/// Chekc if the m_start is the same with m_goalIndex
	if (ticket->m_startIndex == ticket->m_goalIndex)
	{
		ticket->m_pathFound.push_back(ticket->m_startIndex);

		/// Search is stopped because the goal si the same with start node
		ticket->m_state = Ticket::State::COMPLETED;
		return true;
	}

	/// This is the first step -> closed list is empty.
	/// Add the start node to the closed list.
	if (ticket->m_closedList.size() == 0)
	{
		std::shared_ptr<Node> start = std::make_shared<Node>(ticket->m_startIndex);

		/// In this case, we will add the start node.
		start->m_parent = nullptr;

		/// calculate the distance to target
		start->m_distToTarget = m_navMesh->ComputeGoalDistanceEstimate(ticket->m_goalIndex, ticket->m_startIndex);

		/// calculate the cost to travel from m_startIndex node to the neighbor note
		start->m_cost = 0;

		/// Calculate the "F" value
		start->m_f = start->m_distToTarget;

		/// Add the neighbor node to the open list
		ticket->m_closedList[ticket->m_startIndex] = start;

		/// Set the current node to be the start node
		ticket->m_current = start;
	}


	/// If the start node doe not have valid neighbors, try to GetNeighbors and add them to the open list
	if (ticket->m_current->m_neighbors.size() == 0)
	{
		ticket->m_current->m_neighbors = m_navMesh->GetNeighbors(ticket->m_current->m_index);// ticket->m_current->GetNeighbors();
	}

	for (auto& neighbor : ticket->m_current->m_neighbors)
	{
		/// Check if the goal is one of the neighbors
		if (ticket->m_goalIndex == neighbor)
		{
			ticket->m_pathFound.push_back(ticket->m_goalIndex);

			std::shared_ptr<Node> node = ticket->m_current;
			do
			{
				ticket->m_pathFound.push_back(node->m_index);
				//std::cout << "result " << node << " " << node->m_index << " " << node->m_f << std::endl;
				node = node->m_parent;
			} 
			while (node != nullptr);

			ticket->m_state = Ticket::State::COMPLETED;
			return true;
		}

		bool addedAlready = false;
		std::shared_ptr<Node> neigh = nullptr;

		/// check if is in open list
		if (ticket->m_openList.find(neighbor) != ticket->m_openList.end())
		{
			addedAlready = true;
			neigh = ticket->m_openList[neighbor];
		}
		else if (ticket->m_closedList.find(neighbor) != ticket->m_closedList.end())
		{
			addedAlready = true;
			neigh = ticket->m_closedList[neighbor];
		}

		if (!addedAlready)
		{
			neigh = std::make_shared<Node>(neighbor);
			neigh->m_parent = ticket->m_current;
		}



		/// calculate the distance to target
		neigh->m_distToTarget = m_navMesh->ComputeGoalDistanceEstimate(ticket->m_goalIndex, neighbor);

		/// calculate the cost to travel from m_startIndex node to the neighbor note
		neigh->m_cost = m_navMesh->ComputeCost(ticket->m_current->m_index, neighbor);

		/// Calculate the "F" value
		int f = neigh->m_distToTarget + neigh->m_cost;

		/// If the "F" have the default value it means that this node does not have added 
		/// to the open list before.
		if (!addedAlready)
		{
			neigh->m_f = f;

			/// Add the neighbor node to the open list
			ticket->m_openList[neighbor] = neigh;
		}
		else if (neigh->m_f > f)
		{
			neigh->m_f = f;
		}
	}


	/// Chekc if there are some nodes in Open list
	if (ticket->m_openList.size() == 0)
	{
		ticket->m_pathFound.push_back(ticket->m_goalIndex);
		std::shared_ptr<Node> node = ticket->m_current;
		do
		{
			ticket->m_pathFound.push_back(node->m_index);
			//std::cout << "result " << node << " " << node->m_index << " " << node->m_f << std::endl;
			node = node->m_parent;
		} while (node != nullptr);

		/// If no nodes -> search is stopped due to no path to goal.
		ticket->m_state = Ticket::State::STOPPED;
		return true;
	}

	/// Get the object with the 
	int f = INT_MAX;
	for (auto& on : ticket->m_openList)
	{
		if (on.second->m_f < f)
			ticket->m_current = on.second;
	}

	ticket->m_openList.erase(ticket->m_current->m_index);

	ticket->m_closedList[ticket->m_current->m_index] = ticket->m_current;

	//int i = 0;
	//for (auto& on : ticket->m_openList)
	//{
	//	std::cout << i << " " << on.second << " " << on.first << " " << on.second->m_f << std::endl;
	//	i++;
	//}


	return false;
}