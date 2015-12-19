// ConsoleApplication1.cpp : Defines the entry point for the console application.
//


#include "FindPathEngine/FindPathEngine.h"



class Tile;

class NavMesh : public NavMeshBase
{
public:
	NavMesh();
	static const int k_w = 8;
	static const int k_h = 8;
	static const int k_meshSize = k_w * k_h;


	static const int k_collisions[];

	static int GetIndex(int x, int y){ return y * k_w + x; }

	int ComputeGoalDistanceEstimate(NavMeshBase* navMesh, int goalNode, int node) override;
	int ComputeCost(NavMeshBase* navMesh, int node, int neighbor) override;
	std::vector<int> GetNeighbors(NavMeshBase* navMesh, int node) override;
};



const int NavMesh::k_collisions[NavMesh::k_meshSize] = {
/*    0  1  2  3  4  5  6  7  */
/*0*/ 1, 1, 1, 1, 1, 1, 1, 1,
/*1*/ 1, 0, 1, 0, 0, 0, 0, 1,
/*2*/ 1, 0, 1, 0, 1, 1, 0, 1,
/*3*/ 1, 0, 1, 0, 1, 0, 1, 1,
/*4*/ 1, 0, 1, 0, 1, 0, 0, 1,
/*5*/ 1, 0, 1, 0, 1, 1, 0, 1,
/*6*/ 1, 0, 0, 0, 1, 1, 0, 1,
/*7*/ 1, 1, 1, 1, 1, 1, 1, 1
};

NavMesh::NavMesh()
{
	
}

int NavMesh::ComputeGoalDistanceEstimate(NavMeshBase* navMesh, int goalNode, int node)
{
	int goalNodeX = goalNode % NavMesh::k_w;
	int goalNodeY = goalNode / NavMesh::k_w;

	int nodeX = node % NavMesh::k_w;
	int nodeY = node / NavMesh::k_w;

	return std::abs(nodeX - goalNodeX) + std::abs(nodeY - goalNodeY);
}

int NavMesh::ComputeCost(NavMeshBase* navMesh, int node, int neighbor)
{
	int neighborX = neighbor % NavMesh::k_w;
	int neighborY = neighbor / NavMesh::k_w;

	int nodeX = node % NavMesh::k_w;
	int nodeY = node / NavMesh::k_w;

	int x = std::abs(neighborX - nodeX);
	int y = std::abs(neighborY - nodeY);

	if ((x + y) >= 2)
		return 14;

	if ((x + y) == 1)
		return 10;

	/// in case the x = 0, y = 0 -> neighbor = this
	/// so, the cost is 0
	return 0;
};

std::vector<int> NavMesh::GetNeighbors(NavMeshBase* navMesh, int node)
{
	std::vector<int> neighbors;

	int nodeX = node % NavMesh::k_w;
	int nodeY = node / NavMesh::k_w;

	for (int y = nodeY - 1; y <= nodeY + 1; y++)
	{
		for (int x = nodeX - 1; x <= nodeX + 1; x++)
		{
			/// if is outside the mesh do not add it
			if ((y < 0) || (y >= NavMesh::k_h))
				continue;

			/// if is outside the mesh do not add it
			if ((x < 0) || (x >= NavMesh::k_w))
				continue;

			/// if is the current node do not add it
			if ((x == nodeX) && (y == nodeY))
				continue;

			int neighbor = y * NavMesh::k_w + x;

			/// If the tile have collision on it, do not add it
			if (NavMesh::k_collisions[neighbor] == 1)
				continue;


			neighbors.push_back(neighbor);
		}
	}

	return neighbors;
};




int main(int argc, char* argv[])
{
	NavMesh navmesh;

	FindPathEngine engine(&navmesh);

	FindPathEngine::Ticket ticket(NavMesh::GetIndex(1, 1), NavMesh::GetIndex(6, 6));

	engine.AddTicket(&ticket);

	while (!engine.Update())
	{

	}

	for (auto& nodeIndex : ticket.m_pathFound)
	{
		std::cout << "result " << nodeIndex << " " << (nodeIndex % NavMesh::k_w) << "x" << (nodeIndex / NavMesh::k_w) << std::endl;
	}
	

	return 0;
}

