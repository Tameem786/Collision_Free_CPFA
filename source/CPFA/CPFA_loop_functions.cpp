#include "CPFA_loop_functions.h"

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
        SimTime(0),
	//MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
    MaxSimTime(0),//qilu 02/05/2021
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FoodItemCount(256),
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
    RedCircleRadius(2.0),
	NestElevation(0.01),
	RedCirclePosition(0.0, 0.0),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0),
	maxPathQueueSize(50),
	RedCircleRadiusMultiplier(20.0),
	lastWaitingQueueIndexForNest(-1),
	waitingQueueForNest(20, false)
{}

void CPFA_loop_functions::AddRobotToPathQueueToNest(int nestIndex, const std::string& robotId) {
    if (nestIndex >= 0 && nestIndex < 4) {
        pathAvailableToNest[nestIndex] = false;
    }
}

void CPFA_loop_functions::RemoveRobotFromPathQueueToNest(int nestIndex, const std::string& robotId) {
    if (nestIndex >= 0 && nestIndex < 4) {        
        pathAvailableToNest[nestIndex] = true;
    }
}

bool CPFA_loop_functions::IsPathAvailableToNest(int nestIndex) {
    return (nestIndex >= 0 && nestIndex < 4) ? pathAvailableToNest[nestIndex] : false;
}

int CPFA_loop_functions::GetAvailablePathToNest() {
    
    for (int i = 0; i < 4; i++) {
        if (pathAvailableToNest[i]) {
            return i;
        }
    }
    return -1;  // No path available
}

void CPFA_loop_functions::AddRobotToPathQueue(const std::string& robotId) {
    // Check if robot is already in queue
    auto it = std::find(pathQueue.begin(), pathQueue.end(), robotId);
    if (it == pathQueue.end()) {
        pathQueue.push_back(robotId);
        // argos::LOG << "Robot " << robotId << " added to path queue. Queue size: " 
        //            << pathQueue.size() << std::endl;
    } else {
        argos::LOG << "Robot " << robotId << " already in path queue." << std::endl;
    }
}

void CPFA_loop_functions::RemoveRobotFromPathQueue(const std::string& robotId) {
    auto it = std::find(pathQueue.begin(), pathQueue.end(), robotId);
    if (it != pathQueue.end()) {
        pathQueue.erase(it);
        // argos::LOG << "Robot " << robotId << " removed from path queue. Queue size: " 
        //            << pathQueue.size() << std::endl;
    } else {
        argos::LOG << "Warning: Tried to remove robot " << robotId 
                   << " from path queue, but it wasn't found." << std::endl;
    }
}

int CPFA_loop_functions::GetPathQueueSize() {
    return pathQueue.size();
}

int CPFA_loop_functions::GetMaxPathQueueSize() {
    return maxPathQueueSize;
}

void CPFA_loop_functions::SetMaxPathQueueSize(int maxSize) {
    if (maxSize > 0) {
        maxPathQueueSize = maxSize;
        argos::LOG << "Max path queue size set to: " << maxPathQueueSize << std::endl;
    } else {
        argos::LOG << "Warning: Invalid max queue size. Must be > 0." << std::endl;
    }
}

bool CPFA_loop_functions::IsRobotInPathQueue(const std::string& robotId) {
    auto it = std::find(pathQueue.begin(), pathQueue.end(), robotId);
    return (it != pathQueue.end());
}

void CPFA_loop_functions::ClearPathQueue() {
    pathQueue.clear();
    argos::LOG << "Path queue cleared." << std::endl;
}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error

	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
    // argos::GetNodeAttribute(settings_node, "NestPosition", NestPosition);

	std::string nest_positions_str;
	argos::GetNodeAttribute(settings_node, "NestPosition", nest_positions_str);

	// Parse the positions into the NestPositions vector
	std::stringstream ss(nest_positions_str);
	std::string position;
	while (std::getline(ss, position, ';')) {
	    std::stringstream pos_stream(position);
	    argos::Real x, y;
	    char comma;
	    pos_stream >> x >> comma >> y;
	    NestPositions.emplace_back(x, y);
		// argos::LOG << "Nest Position: " << x << ", " << y << std::endl;
	}
	//print content of nest_positions
	for (int i=0; i<NestPositions.size(); i++) {
		argos::LOG << "Nest Position[" << i << "]: " << NestPositions[i].GetX() << ", " << NestPositions[i].GetY() << std::endl;
	}
    FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }
    

	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

        ArenaWidth = ArenaSize[0];
        
        // if(abs(NestPosition.GetX()) < -1) //quad arena
        // {
        //     NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
        // }
        // else
        // {
        //     NestRadius *= sqrt(log(ArenaWidth)/log(2));
        // }
        // argos::LOG<<"NestRadius="<<NestRadius<<endl;
	   // Send a pointer to this loop functions object to each controller.
	   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        c2.SetLoopFunctions(this);
	    }
     
     
   	NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
	
	ForageList.clear(); 
	last_time_in_minutes=0;

	if(SpiralPathCoordinates.empty()) {
		SetSpiralPathCoordinates();
	}

	// argos::LOG << "NestRadius: " << NestRadius << std::endl;
	// argos::LOG << "NestRadiusSquared: " << NestRadiusSquared << std::endl;

	pathAvailableToNest.resize(4, true);
 
}

bool CPFA_loop_functions::IsFirstInQueue(const std::string& robotId) {
    if(robotQueue.empty()) {
        return false;
    }
    return robotQueue.front().robotId == robotId;
}

int CPFA_loop_functions::AddToQueue(const std::string& robotId) {
    // Check if robot is already in queue
    int existingIndex = FindRobotInQueue(robotId);
    if(existingIndex != -1) {
        argos::LOG << "Robot " << robotId << " is already in queue" << std::endl;
        return robotQueue[existingIndex].positionIndex;
    }
    
    // Find next available queue position
    int availablePosition = GetNextAvailableQueuePosition();
    if(availablePosition == -1) {
        argos::LOG << "Error: No available queue positions for robot " << robotId << std::endl;
        return -1;
    }
    
    // Mark position as occupied
    waitingQueuePositionsAvailability[availablePosition] = false;
    
    // Add robot to queue
    QueueEntry newEntry(robotId, availablePosition);
    robotQueue.push_back(newEntry);
    
    // argos::LOG << "Robot " << robotId << " added to queue at position index " << availablePosition 
    //            << " (x=" << waitingQueuePositions[availablePosition].GetX() 
    //            << ", y=" << waitingQueuePositions[availablePosition].GetY() << ")" << std::endl;
    
    return availablePosition;
}

void CPFA_loop_functions::RemoveFromQueue(const std::string& robotId) {
    // Find robot in queue
    auto it = std::find_if(robotQueue.begin(), robotQueue.end(),
        [&robotId](const QueueEntry& entry) {
            return entry.robotId == robotId;
        });
    
    if(it != robotQueue.end()) {
        int positionIndex = it->positionIndex;
        
        argos::LOG << "Robot " << robotId << " removed from queue position " << positionIndex << std::endl;
        
        // Mark position as available
        waitingQueuePositionsAvailability[positionIndex] = true;
        
        // Remove from queue
        robotQueue.erase(it);
        
        // Update positions for remaining robots (move them forward)
        UpdateQueuePositions();
    } else {
        argos::LOG << "Warning: Robot " << robotId << " not found in queue for removal" << std::endl;
    }
}

int CPFA_loop_functions::GetCurrentQueueLength() {
    return robotQueue.size();
}

argos::CVector2 CPFA_loop_functions::GetUpdatedQueuePosition(const std::string& robotId) {
    int robotIndex = FindRobotInQueue(robotId);
    if(robotIndex == -1) {
        argos::LOG << "Warning: Robot " << robotId << " not found in queue" << std::endl;
        return argos::CVector2(0.0, 0.0);
    }
    
    int positionIndex = robotQueue[robotIndex].positionIndex;
    return waitingQueuePositions[positionIndex];
}

void CPFA_loop_functions::UpdateQueuePositions() {
    if(robotQueue.empty()) return;
    
    // Sort robots by their current position index to maintain order
    std::sort(robotQueue.begin(), robotQueue.end(),
        [](const QueueEntry& a, const QueueEntry& b) {
            return a.positionIndex < b.positionIndex;
        });
    
    // Try to move robots forward to earlier positions if available
    for(size_t i = 0; i < robotQueue.size(); ++i) {
        int currentPos = robotQueue[i].positionIndex;
        
        // Look for an earlier available position
        for(int j = 0; j < currentPos; ++j) {
            if(waitingQueuePositionsAvailability[j]) {
                // Move robot to earlier position
                waitingQueuePositionsAvailability[currentPos] = true;  // Free old position
                waitingQueuePositionsAvailability[j] = false;          // Occupy new position
                robotQueue[i].positionIndex = j;
                
                argos::LOG << "Robot " << robotQueue[i].robotId << " moved forward from position " 
                           << currentPos << " to position " << j << std::endl;
                break;
            }
        }
    }
    
    // Sort again after updates
    std::sort(robotQueue.begin(), robotQueue.end(),
        [](const QueueEntry& a, const QueueEntry& b) {
            return a.positionIndex < b.positionIndex;
        });
}

int CPFA_loop_functions::FindRobotInQueue(const std::string& robotId) {
    for(size_t i = 0; i < robotQueue.size(); ++i) {
        if(robotQueue[i].robotId == robotId) {
            return i;
        }
    }
    return -1;
}

int CPFA_loop_functions::GetNextAvailableQueuePosition() {
    // Find the first available position in the queue
    for(size_t i = 0; i < waitingQueuePositionsAvailability.size(); ++i) {
        if(waitingQueuePositionsAvailability[i]) {
            return i;
        }
    }
    return -1; // No available positions
}

void CPFA_loop_functions::PrintQueueStatus() {
    argos::LOG << "=== Queue Status ===" << std::endl;
    argos::LOG << "Queue length: " << robotQueue.size() << std::endl;
    argos::LOG << "Available positions: ";
    
    int availableCount = 0;
    for(size_t i = 0; i < waitingQueuePositionsAvailability.size(); ++i) {
        if(waitingQueuePositionsAvailability[i]) {
            availableCount++;
        }
    }
    argos::LOG << availableCount << "/" << waitingQueuePositionsAvailability.size() << std::endl;
    
    for(size_t i = 0; i < robotQueue.size(); ++i) {
        int posIndex = robotQueue[i].positionIndex;
        argos::LOG << "Robot " << robotQueue[i].robotId << " at queue position " << posIndex
                   << " (x=" << waitingQueuePositions[posIndex].GetX() 
                   << ", y=" << waitingQueuePositions[posIndex].GetY() << ")" << std::endl;
    }
    argos::LOG << "===================" << std::endl;
}

// Additional helper methods

bool CPFA_loop_functions::IsQueueFull() {
    return GetNextAvailableQueuePosition() == -1;
}

bool CPFA_loop_functions::IsQueueEmpty() {
    return robotQueue.empty();
}

std::vector<std::string> CPFA_loop_functions::GetQueuedRobotIds() {
    std::vector<std::string> robotIds;
    for(const auto& entry : robotQueue) {
        robotIds.push_back(entry.robotId);
    }
    return robotIds;
}

// Method to check if a robot should move forward in queue
bool CPFA_loop_functions::ShouldRobotMoveForward(const std::string& robotId, const argos::CVector2& currentPos) {
    int robotIndex = FindRobotInQueue(robotId);
    if(robotIndex == -1) return false;
    
    int positionIndex = robotQueue[robotIndex].positionIndex;
    argos::CVector2 targetPos = waitingQueuePositions[positionIndex];
    argos::Real distance = (targetPos - currentPos).Length();
    
    // Robot should move if it's more than a small threshold away from target position
    return distance > 0.1;
}

// Initialize/Reset queue system
void CPFA_loop_functions::ResetQueue() {
    // Clear the robot queue
    robotQueue.clear();
    
    // Mark all queue positions as available
    std::fill(waitingQueuePositionsAvailability.begin(), 
              waitingQueuePositionsAvailability.end(), true);
    
    argos::LOG << "Queue system reset - " << waitingQueuePositions.size() 
               << " positions available" << std::endl;
}

CircleEntry CPFA_loop_functions::IsNearFirstInnerCircle(const argos::CVector2& position) {
    for(int i = 0; i < FirstInnerCircleCoordinates.size(); i++) {
        if((FirstInnerCircleCoordinates[i] - position).Length() < 0.15) {
            for(int j = 0; j < SpiralPathCoordinatesForController.size(); j++) {
                if(SpiralPathCoordinatesForController[j] == FirstInnerCircleCoordinates[i]) {
                    return CircleEntry(true, j);
                }
            }
        }
    }
    return CircleEntry(false, -1);
}

CircleEntry CPFA_loop_functions::IsNearSecondInnerCircle(const argos::CVector2& position) {
    for(int i = 0; i < SecondInnerCircleCoordinates.size(); i++) {
        if((SecondInnerCircleCoordinates[i] - position).Length() < 0.15) {
            for(int j = 0; j < SpiralPathCoordinatesForController.size(); j++) {
                if(SpiralPathCoordinatesForController[j] == SecondInnerCircleCoordinates[i]) {
                    return CircleEntry(true, j);
                }
            }
        }
    }
    return CircleEntry(false, -1);
}

CircleEntry CPFA_loop_functions::IsNearThirdInnerCircle(const argos::CVector2& position) {
    
    for(int i = 0; i < ThirdInnerCircleCoordinates.size(); i++) {
        
        if((ThirdInnerCircleCoordinates[i] - position).Length() < 0.15) {
            for(int j = 0; j < SpiralPathCoordinatesForController.size(); j++) {
                if(SpiralPathCoordinatesForController[j] == ThirdInnerCircleCoordinates[i]) {
                    return CircleEntry(true, j);
                }
            }
        }
    }
    return CircleEntry(false, -1);
}

CircleEntry CPFA_loop_functions::IsNearFourthInnerCircle(const argos::CVector2& position) {
    
    for(int i = 0; i < FourthInnerCircleCoordinates.size(); i++) {
        
        if((FourthInnerCircleCoordinates[i] - position).Length() < 0.15) {
            for(int j = 0; j < SpiralPathCoordinatesForController.size(); j++) {
                if(SpiralPathCoordinatesForController[j] == FourthInnerCircleCoordinates[i]) {
                    return CircleEntry(true, j);
                }
            }
        }
    }
    return CircleEntry(false, -1);
}

CircleEntry CPFA_loop_functions::IsNearFifthInnerCircle(const argos::CVector2& position) {
    
    for(int i = 0; i < FifthInnerCircleCoordinates.size(); i++) {
        
        if((FifthInnerCircleCoordinates[i] - position).Length() < 0.15) {
            for(int j = 0; j < SpiralPathCoordinatesForController.size(); j++) {
                if(SpiralPathCoordinatesForController[j] == FifthInnerCircleCoordinates[i]) {
                    return CircleEntry(true, j);
                }
            }
        }
    }
    return CircleEntry(false, -1);
}

void CPFA_loop_functions::SetNestsPredefinedEntryPathCoordinates() {
    nest1EntryPoints.clear();
	nest2EntryPoints.clear();
	nest3EntryPoints.clear();
	nest4EntryPoints.clear();

    CVector3 lastPoint1 = SpiralPathCoordinates.back();
    CVector3 lastPoint2 = SpiralPathCoordinates.back();
    CVector3 lastPoint3 = SpiralPathCoordinates.back();
    CVector3 lastPoint4 = SpiralPathCoordinates.back();

    nest1EntryPoints.push_back(lastPoint1);
    nest2EntryPoints.push_back(lastPoint2);
    nest3EntryPoints.push_back(lastPoint3);
    nest4EntryPoints.push_back(lastPoint4);

    for(int i = 0; i < 6; i++) {
        lastPoint1.Set(lastPoint1.GetX(), lastPoint1.GetY() + 0.05f, 0.0f);
        nest1EntryPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 8; i++) {
        lastPoint1.Set(lastPoint1.GetX()+ 0.05f, lastPoint1.GetY(), 0.0f);
        nest1EntryPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 3; i++) {
        lastPoint2.Set(lastPoint2.GetX() - 0.05f, lastPoint2.GetY() + 0.05f, 0.0f);
        nest2EntryPoints.push_back(lastPoint2);
    }

    for(int i = 0; i < 9; i++) {
        lastPoint2.Set(lastPoint2.GetX(), lastPoint2.GetY() + 0.05f, 0.0f);
        nest2EntryPoints.push_back(lastPoint2);
    }

    for(int i = 0; i < 5; i++) {
        lastPoint2.Set(lastPoint2.GetX() + 0.05f, lastPoint2.GetY(), 0.0f);
        nest2EntryPoints.push_back(lastPoint2);
    }

    for(int i = 0; i < 6; i++) {
        lastPoint3.Set(lastPoint3.GetX() - 0.05f, lastPoint3.GetY() + 0.05f, 0.0f);
        nest3EntryPoints.push_back(lastPoint3);
    }

    for(int i = 0; i < 12; i++) {
        lastPoint3.Set(lastPoint3.GetX(), lastPoint3.GetY() + 0.05f, 0.0f);
        nest3EntryPoints.push_back(lastPoint3);
    }

    for(int i = 0; i < 14; i++) {
        lastPoint3.Set(lastPoint3.GetX() + 0.05f, lastPoint3.GetY(), 0.0f);
        nest3EntryPoints.push_back(lastPoint3);
    }

    for(int i = 0; i < 2; i++) {
        lastPoint4.Set(lastPoint4.GetX(), lastPoint4.GetY() + 0.05f, 0.0f);
        nest4EntryPoints.push_back(lastPoint4);
    }

    for(int i = 0; i < 16; i++) {
        lastPoint4.Set(lastPoint4.GetX()+ 0.05f, lastPoint4.GetY(), 0.0f);
        nest4EntryPoints.push_back(lastPoint4);
    }

    for(int i = 0; i < 8; i++) {
        lastPoint4.Set(lastPoint4.GetX(), lastPoint4.GetY() + 0.05f, 0.0f);
        nest4EntryPoints.push_back(lastPoint4);
    }

}

void CPFA_loop_functions::SetNestsPredefinedExitPathCoordinates() {
    nest1ExitPoints.clear();
	nest2ExitPoints.clear();
	nest3ExitPoints.clear();
	nest4ExitPoints.clear();

    CVector3 lastPoint1 = CVector3(NestPositions[3].GetX(), NestPositions[3].GetY(), 0.0f);
    CVector3 lastPoint2 = CVector3(NestPositions[2].GetX(), NestPositions[2].GetY(), 0.0f);
    CVector3 lastPoint3 = CVector3(NestPositions[1].GetX(), NestPositions[1].GetY(), 0.0f);
    CVector3 lastPoint4 = CVector3(NestPositions[0].GetX(), NestPositions[0].GetY(), 0.0f);

    nest1ExitPoints.push_back(lastPoint1);
    nest2ExitPoints.push_back(lastPoint2);
    nest3ExitPoints.push_back(lastPoint3);
    nest4ExitPoints.push_back(lastPoint4);

    for(int i = 0; i < 6; i++) {
        lastPoint1.Set(lastPoint1.GetX()+ 0.05f, lastPoint1.GetY(), 0.0f);
        nest1ExitPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 30; i++) {
        lastPoint1.Set(lastPoint1.GetX() + 0.05f, lastPoint1.GetY() + 0.05f, 0.0f);
        nest1ExitPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 6; i++) {
        lastPoint2.Set(lastPoint2.GetX(), lastPoint2.GetY() + 0.05f, 0.0f);
        nest2ExitPoints.push_back(lastPoint2);
    }

    for(int i = 0; i < 30; i++) {
        lastPoint2.Set(lastPoint2.GetX() + 0.05f, lastPoint2.GetY() + 0.05f, 0.0f);
        nest2ExitPoints.push_back(lastPoint2);
    }

    

    for(int i = 0; i < 6; i++) {
        lastPoint3.Set(lastPoint3.GetX(), lastPoint3.GetY() + 0.05f, 0.0f);
        nest3ExitPoints.push_back(lastPoint3);
    }

    for(int i = 0; i < 25; i++) {
        lastPoint3.Set(lastPoint3.GetX() + 0.05f, lastPoint3.GetY() + 0.05f, 0.0f);
        nest3ExitPoints.push_back(lastPoint3);
    }

    for(int i = 0; i < 6; i++) {
        lastPoint4.Set(lastPoint4.GetX() + 0.05f, lastPoint4.GetY(), 0.0f);
        nest4ExitPoints.push_back(lastPoint4);
    }
     // Move Left
    for(int i = 0; i < 20; i++) {
        lastPoint4.Set(lastPoint4.GetX() + 0.05f, lastPoint4.GetY() + 0.05f, 0.0f);
        nest4ExitPoints.push_back(lastPoint4);
    }
}

void CPFA_loop_functions::SetSpiralPathCoordinates() {
	SpiralPathCoordinates.clear();
	SpiralPathCoordinatesForController.clear();
    
    FirstInnerCircleCoordinates.clear();
    SecondInnerCircleCoordinates.clear();
    ThirdInnerCircleCoordinates.clear();
    FourthInnerCircleCoordinates.clear();
    FifthInnerCircleCoordinates.clear();
    SixthInnerCircleCoordinates.clear();

    const Real fRedCircleRadius = RedCircleRadius;
    const CVector2 center = RedCirclePosition;

    std::vector<std::vector<CVector2>*> arcLayers = {
        &FirstInnerCircleCoordinates,
        &SecondInnerCircleCoordinates,
        &ThirdInnerCircleCoordinates,
        &FourthInnerCircleCoordinates,
        &FifthInnerCircleCoordinates,
        &SixthInnerCircleCoordinates
    };

    const UInt32 numArcs = 6;
    const UInt32 numPoints = 250;

    const Real startAngleDeg = 55.0f;
    const Real arcSpanDeg = 340.0f;
    const Real stepRatio = 0.1f;                     // Inward step fraction of red circle radius
    const Real baseRadius = fRedCircleRadius * 0.9f; // Start radius
    Real radius = baseRadius;

    // ====== Initial Starting Point ======
    Real angleRad = startAngleDeg * ARGOS_PI / 180.0f;
    CVector2 currentPoint = CVector2(
        center.GetX() + radius * Cos(CRadians(angleRad)),
        center.GetY() + radius * Sin(CRadians(angleRad))
    );
    SpiralPathCoordinates.push_back(CVector3(currentPoint.GetX(), currentPoint.GetY(), 0.1f));
    (*arcLayers[0]).push_back(currentPoint);

    for (UInt32 layer = 0; layer < numArcs; ++layer) {
        bool reverse = (layer % 2 == 1); // odd layers go in reverse

        Real arcStart = reverse ? startAngleDeg + arcSpanDeg : startAngleDeg;
        Real arcSpan = (layer == 5) ? 165.0f : arcSpanDeg;
        Real arcDir = reverse ? -arcSpan : arcSpan;

        // === Generate arc ===
        for (UInt32 i = 0; i <= numPoints; ++i) {
            Real angleDeg = arcStart + (arcDir * i) / numPoints;
            Real angleRad = angleDeg * ARGOS_PI / 180.0f;

            Real x = center.GetX() + radius * Cos(CRadians(angleRad));
            Real y = center.GetY() + radius * Sin(CRadians(angleRad));

            CVector3 pt3D(x, y, 0.1f);
            SpiralPathCoordinates.push_back(pt3D);
            (*arcLayers[layer]).push_back(CVector2(x, y));
        }

        // === Step inward toward center ===
        if (layer < numArcs - 1) { // skip final step after last arc
            CVector3 last3D = SpiralPathCoordinates.back();
            CVector2 last2D(last3D.GetX(), last3D.GetY());
            CVector2 target;
            if(layer % 2 != 0) {
                target = CVector2(0.0, 0.2);
            } else {
                target = CVector2(0.2, 0.0);
            }
            CVector2 dir = target - last2D;
            dir.Normalize();

            CVector2 nextPoint = last2D + fRedCircleRadius * stepRatio * dir;
            SpiralPathCoordinates.push_back(CVector3(nextPoint.GetX(), nextPoint.GetY(), 0.1f));
            (*arcLayers[layer + 1]).push_back(nextPoint);

            // Update radius based on new point
            radius = (nextPoint - center).Length();
        }
    }

    
    // const UInt32 unNumPoints = 270;
    // Real arc_degree = 306.0f;
   
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.9f) * Sin(CRadians(-radian)),
    //                    (fRedCircleRadius*0.9f) * Cos(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     FirstInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }

    // float t = 0.1f; // 20% of the distance to the center
    // CVector3 lastPoint = SpiralPathCoordinates.back();
    // CVector3 anotherPoint(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY()- lastPoint.GetY()),
    //     0.1f
    // );

    // SpiralPathCoordinates.push_back(anotherPoint);
    // SecondInnerCircleCoordinates.push_back(CVector2(anotherPoint.GetX(), anotherPoint.GetY()));

    // // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.8f) * Cos(CRadians(-radian)),
    //                    (fRedCircleRadius*0.8f) * Sin(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     SecondInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }

    // lastPoint = SpiralPathCoordinates.back();
    
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);
    // ThirdInnerCircleCoordinates.push_back(CVector2(anotherPoint.GetX(), anotherPoint.GetY()));

    // // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    // arc_degree = 304;
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.7f) * Sin(CRadians(-radian)),
    //                    (fRedCircleRadius*0.7f) * Cos(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     ThirdInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }

    // lastPoint = SpiralPathCoordinates.back();
    // t = 0.15f;
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);
    // FourthInnerCircleCoordinates.push_back(CVector2(anotherPoint.GetX(), anotherPoint.GetY()));

    // // arc_degree = 125.0f;
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.6f) * Cos(CRadians(-radian)),
    //                    (fRedCircleRadius*0.6f) * Sin(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     FourthInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }


    // lastPoint = SpiralPathCoordinates.back();
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);
    // FifthInnerCircleCoordinates.push_back(CVector2(anotherPoint.GetX(), anotherPoint.GetY()));

    // arc_degree = 302;
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.5f) * Sin(CRadians(-radian)),
    //                    (fRedCircleRadius*0.5f) * Cos(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     FifthInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }

    // lastPoint = SpiralPathCoordinates.back();
    // t = 0.2f;
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);
    // SixthInnerCircleCoordinates.push_back(CVector2(anotherPoint.GetX(), anotherPoint.GetY()));

    // arc_degree = 125.0f;
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.4f) * Cos(CRadians(-radian)),
    //                    (fRedCircleRadius*0.4f) * Sin(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    //     SixthInnerCircleCoordinates.push_back(CVector2(point.GetX(), point.GetY()));
    // }

	for(UInt32 i = 0; i <= SpiralPathCoordinates.size(); ++i) {
		SpiralPathCoordinatesForController.push_back(
			CVector2(SpiralPathCoordinates[i].GetX(), SpiralPathCoordinates[i].GetY()));
	}

    SetNestsPredefinedEntryPathCoordinates();
    SetNestsPredefinedExitPathCoordinates();

    // const Real fRedCircleRadius = NestRadius * RedCircleRadiusMultiplier;
    const UInt32 unNumSegments = 30;
    const Real arcStart = M_PI / 6.0;      // 30 degrees in radians
    const Real arcEnd = M_PI / 2.0;        // 90 degrees in radians
    const Real fDeltaAngle = (arcEnd - arcStart) / unNumSegments;

    for(UInt32 i = 0; i <= unNumSegments; ++i) { // <= to include the last point
        Real fAngle = arcStart + fDeltaAngle * i;
        Real x = (fRedCircleRadius + 0.3) * cos(fAngle);
        Real y = (fRedCircleRadius + 0.3)* sin(fAngle);
        forbiddenAreaCoordinates.push_back(CVector2(x, y));
    }

    // LOG << "forbiddenAreaCoordinates size: " << forbiddenAreaCoordinates.size() << std::endl;

    // LOG << "Waiting queue first position: " << waitingQueuePositionsForNest[0].GetX() << ", " << waitingQueuePositionsForNest[0].GetY() << std::endl;
    // LOG << "SpiralPathCoordinatesForController last position: " << SpiralPathCoordinatesForController[SpiralPathCoordinatesForController.size()-2].GetX() << ", " << SpiralPathCoordinatesForController[SpiralPathCoordinatesForController.size()-2].GetY() << std::endl;

}

bool CPFA_loop_functions::IsNearExitPoint(const argos::CVector2& position) {
    CVector2 exitpoint(nest1ExitPoints.back().GetX(), nest1ExitPoints.back().GetY());
    Real distance = (position - exitpoint).Length();

    if(distance < 0.2f) return true;

    return false;
}

CircleEntry CPFA_loop_functions::IsNearForbiddenArea(const argos::CVector2& position) {
    for(int i; i < forbiddenAreaCoordinates.size(); i++) {
        argos::Real distance = (position - forbiddenAreaCoordinates[i]).Length();
        if(distance < 0.1) {
            return CircleEntry(true, i);
        }
    }
    return CircleEntry(false, -1);
}


argos::CVector2 CPFA_loop_functions::GetClosestNest(argos::CVector2& robotPosition) {
	argos::CVector2 closestPos;
	argos::Real minDistance = std::numeric_limits<argos::Real>::max();

	// Find the closest nest
	for(const auto& nestPos : NestPositions) {
		argos::Real distance = (nestPos - robotPosition).Length();
		if(distance < minDistance) {
			minDistance = distance;
			closestPos = nestPos;
		}
	}
	return closestPos;
}


void CPFA_loop_functions::Reset() {
	   if(VariableFoodPlacement == 0) {
		      RNG->Reset();
	   }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();
    FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();
    
    SetFoodDistribution();
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    // if(SimTime % 19200 == 0) { // 19200 == 10 (curr_time_in_mins)
    //     printf("%f, %f, %d\n", score, curr_time_in_minutes, SimTime);
    // }

    if(curr_time_in_minutes - last_time_in_minutes==1){
		      
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }


	   UpdatePheromoneList();

	   if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
        for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = argos::CColor::BLACK;
        }
	   }
 
    if(FoodList.size() == 0) {
	FidelityList.clear();
	PheromoneList.clear();
        TargetRayList.clear();
    }
}

void CPFA_loop_functions::PostStep() {
	// nothing... yet...
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0) {
		isFinished = true;
	}
    //set to collected 88% food and then stop
    if(score >= NumDistributedFood){
		isFinished = true;
	}
    if(curr_time_in_minutes >= 20.0) {
        isFinished = true;

    }
    
	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;
        argos::LOG<< "time out..."<<endl; 
		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
    printf("Resource Collected: %f, Time Took: %f minutes, Random Seed: %lu\n", score, getSimTimeInSeconds()/60.0, RandomSeed);
       
                  
    if (PrintFinalScore == 1) {
        string type="";
        if (FoodDistribution == 0) type = "random";
        else if (FoodDistribution == 1) type = "cluster";
        else type = "powerlaw";
            
        ostringstream num_tag;
        num_tag << FoodItemCount; 
              
        ostringstream num_robots;
        num_robots <<  Num_robots;
   
        ostringstream arena_width;
        arena_width << ArenaWidth;
        
        // ostringstream quardArena;
        // if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
        //      quardArena << 1;
        //  }
        //  else{
        //      quardArena << 0;
        // }
        
        string header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_";
       
        //unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("Default").GetInverseSimulationClockTick();
        unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
        /* Real total_travel_time=0;
        Real total_search_time=0;
        ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
        */
        
        
        argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
        for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
            argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
            BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
            CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
            CollisionTime += c2.GetCollisionTime();
            
            /*if(c2.GetStatus() == "SEARCHING"){
                total_search_time += SimTime-c2.GetTravelingTime();
                total_travel_time += c2.GetTravelingTime();
	    }
            else {
		total_search_time += c2.GetSearchingTime();
		total_travel_time += SimTime-c2.GetSearchingTime();
            } */        
        }
        //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
        //travelSearchTimeDataOutput.close();   
             
        ofstream dataOutput( (header+ "iAntTagData.txt").c_str(), ios::app);
        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
        }
    
        //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
        //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput.close();
    
        ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
        if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
        for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
        forageDataOutput<<"\n";
        forageDataOutput.close();
        
      }  

}


argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();
	    for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
      }
     	PheromoneList = new_p_list;
	new_p_list.clear();
}
void CPFA_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void CPFA_loop_functions::RandomFoodDistribution() {
	FoodList.clear();
        FoodColoringList.clear();
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}


void CPFA_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
    FoodColoringList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	FoodItemCount = foodPlaced;
}
 
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
    argos::CVector2 placementPosition = p;

    argos::Real foodOffset   = 3.0 * FoodRadius;
    argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
    argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

    argos::Real x_min = p.GetX() - FoodRadius;
    argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

    argos::Real y_min = p.GetY() - FoodRadius;
    argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

    if((x_min < (ForageRangeX.GetMin() + FoodRadius))
            || (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
            (y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
            (y_max > (ForageRangeY.GetMax() - FoodRadius)))
    {
        return true;
    }

    for(size_t j = 0; j < length; j++) {
        for(size_t k = 0; k < width; k++) {
            if(IsCollidingWithFood(placementPosition)) return true;
            if(IsCollidingWithNest(placementPosition)) return true;
            
            // Check if position is inside any red circle
			argos::Real redCircleRadius =  RedCircleRadius + 0.5;
			argos::Real distanceToNest = (placementPosition - RedCirclePosition).Length();
			if (distanceToNest < redCircleRadius) {
				return true;
			}
            
            placementPosition.SetX(placementPosition.GetX() + foodOffset);
        }

        placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
        placementPosition.SetY(placementPosition.GetY() + foodOffset);
    }

    return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    for (const auto& nest_position : NestPositions) {
        if ((p - nest_position).SquareLength() < NRPB_squared) {
            return true;
        }
    }
    return false;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
	if (score >= NumDistributedFood) {
		PostExperiment();
	}
}

double CPFA_loop_functions::Score() {	
	return score;
}

void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

bool CPFA_loop_functions::IsNearRedCircle(const argos::CVector2& p) {
    // The red circle radius is 5 times the nest radius
    argos::Real redCircleRadius = RedCircleRadius;
    argos::Real distance = (p - RedCirclePosition).Length();
    
    // Check if the position is near the red circle (within a small tolerance)
    argos::Real tolerance = 0.15; // 15cm tolerance
    // return std::abs(distance - redCircleRadius) < tolerance;
	return (distance >= (redCircleRadius - tolerance) && 
            distance <= (redCircleRadius + tolerance));
}

// Check if robot is inside the red circle
bool CPFA_loop_functions::IsInsideRedCircle(const argos::CVector2& position) {
    argos::Real redCircleRadius = RedCircleRadius;
    argos::Real distanceToCenter = (position - RedCirclePosition).Length();
    return (distanceToCenter < redCircleRadius);
}

// Get the closest entry point on the red circle
argos::CVector2 CPFA_loop_functions::GetClosestExitPoint(const argos::CVector2& robotPosition) {
    // For your diagram, the entry point appears to be at the bottom (south)
    argos::Real redCircleRadius =  RedCircleRadius;
    argos::CVector2 exitPoint(RedCirclePosition.GetX()+redCircleRadius, RedCirclePosition.GetY()+redCircleRadius-0.7);
    return exitPoint;
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
