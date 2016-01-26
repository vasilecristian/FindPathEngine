// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "MemoryLeaksTracker/MemoryLT.h"
#include "FindPathEngine/FindPathEngine.h"



class NavMesh : public fpe::NavMeshBase
{
public:
	NavMesh();
	static const unsigned int k_w = 8;
	static const unsigned int k_h = 8;
	static const unsigned int k_meshSize = k_w * k_h;


	static const int k_collisions[];

	static unsigned int GetIndex(unsigned int x, unsigned int y){ return y * k_w + x; }

	int ComputeGoalDistanceEstimate(unsigned int goalIndex, unsigned int nodeIndex) override;
	int ComputeCost(unsigned int nodeIndex, unsigned int neighborIndex) override;
	std::vector<unsigned int> GetNeighbors(unsigned int nodeIndex) override;
};



const int NavMesh::k_collisions[NavMesh::k_meshSize] = {
/*    0  1  2  3  4  5  6  7  */
/*0*/ 1, 1, 1, 1, 1, 1, 1, 1,
/*1*/ 1, 0, 1, 0, 0, 0, 0, 1,
/*2*/ 1, 0, 1, 0, 1, 1, 0, 1,
/*3*/ 1, 0, 1, 0, 1, 0, 1, 1,
/*4*/ 1, 0, 1, 0, 1, 1, 0, 1,
/*5*/ 1, 0, 1, 0, 1, 0, 1, 1,
/*6*/ 1, 0, 0, 1, 1, 1, 0, 1,
/*7*/ 1, 1, 1, 1, 1, 1, 1, 1
};

NavMesh::NavMesh()
{
	
}

int NavMesh::ComputeGoalDistanceEstimate(unsigned int goalIndex, unsigned int nodeIndex)
{
	int goalX = goalIndex % NavMesh::k_w;
	int goalY = goalIndex / NavMesh::k_w;

	int nodeX = nodeIndex % NavMesh::k_w;
	int nodeY = nodeIndex / NavMesh::k_w;

	int dx = std::abs(nodeX - goalX);
	int dy = std::abs(nodeY - goalY);
	float dist = std::sqrtf(float(dx*dx + dy*dy)) * 1000;

	return (int)dist;
}

int NavMesh::ComputeCost(unsigned int nodeIndex, unsigned int neighborIndex)
{
	int neighborX = neighborIndex % NavMesh::k_w;
	int neighborY = neighborIndex / NavMesh::k_w;

	int nodeX = nodeIndex % NavMesh::k_w;
	int nodeY = nodeIndex / NavMesh::k_w;

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

std::vector<unsigned int> NavMesh::GetNeighbors(unsigned int nodeIndex)
{
	std::vector<unsigned int> neighbors;

	int nodeX = nodeIndex % NavMesh::k_w;
	int nodeY = nodeIndex / NavMesh::k_w;

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
	std::shared_ptr<NavMesh> navmesh = std::shared_ptr<NavMesh>(NEW NavMesh());

	std::shared_ptr<fpe::FindPathEngine> engine = std::make_shared<fpe::FindPathEngine>(navmesh, 2);
    std::shared_ptr<fpe::Ticket> ticket = nullptr;
    for (int i = 0; i < 1000; i++)
    {
        ticket = std::make_shared<fpe::Ticket>(NavMesh::GetIndex(1, 1), NavMesh::GetIndex(6, 6), true);

        engine->AddTicket(ticket);
    }

	while (!engine->Update())
	{

	}

	for (auto& nodeIndex : ticket->GetFoundPath())
	{
		std::cout << "result " << nodeIndex << " " << (nodeIndex % NavMesh::k_w) << "x" << (nodeIndex / NavMesh::k_w) << std::endl;
	}
	

	return 0;
}

