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
    RedCircleRadius(2.0),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	RedCirclePosition(0.0, 0.0),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0),
	maxPathQueueSize(50),
	RedCircleRadiusMultiplier(20.0)
{}

void CPFA_loop_functions::AddRobotToPathQueueToNest(int nestIndex, const std::string& robotId) {
    if (nestIndex >= 0 && nestIndex < 4) {
        auto& queue = (nestIndex == 0) ? pathQueueToNest1 : 
                     (nestIndex == 1) ? pathQueueToNest2 :
                     (nestIndex == 2) ? pathQueueToNest3 : pathQueueToNest4;
        
        auto it = std::find(queue.begin(), queue.end(), robotId);
        if (it == queue.end()) {
            queue.push_back(robotId);
            pathAvailableToNest[nestIndex] = false;  // Path is now occupied
            // LOG << "Robot " << robotId << " added to path queue " << nestIndex << std::endl;
        }
    }
}

void CPFA_loop_functions::RemoveRobotFromPathQueueToNest(int nestIndex, const std::string& robotId) {
    if (nestIndex >= 0 && nestIndex < 4) {
        auto& queue = (nestIndex == 0) ? pathQueueToNest1 : 
                     (nestIndex == 1) ? pathQueueToNest2 :
                     (nestIndex == 2) ? pathQueueToNest3 : pathQueueToNest4;
        
        auto it = std::find(queue.begin(), queue.end(), robotId);
        if (it != queue.end()) {
            queue.erase(it);
            if (queue.empty()) {
                pathAvailableToNest[nestIndex] = true;  // Path is now free
            }
            // LOG << "Robot " << robotId << " removed from path queue " << nestIndex << std::endl;
        }
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
    } else {
        argos::LOG << "Robot " << robotId << " already in path queue." << std::endl;
    }
}

void CPFA_loop_functions::RemoveRobotFromPathQueue(const std::string& robotId) {
    auto it = std::find(pathQueue.begin(), pathQueue.end(), robotId);
    if (it != pathQueue.end()) {
        pathQueue.erase(it);
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
	for (const auto& pos : NestPositions) {
		argos::LOG << "Nest Position: " << pos.GetX() << ", " << pos.GetY() << std::endl;
	}
    FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }

    argos::LOG << "Need to collect: " << NumDistributedFood * 0.5 << "/" << NumDistributedFood << std::endl;
    

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

    for(int i = 0; i < 7; i++) {
        lastPoint1.Set(lastPoint1.GetX(), lastPoint1.GetY() + 0.05f, 0.0f);
        nest1EntryPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 7; i++) {
        lastPoint1.Set(lastPoint1.GetX()+ 0.05f, lastPoint1.GetY(), 0.0f);
        nest1EntryPoints.push_back(lastPoint1);
    }

    for(int i = 0; i < 4; i++) {
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

    for(int i = 0; i < 7; i++) {
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

    for(int i = 0; i < 15; i++) {
        lastPoint4.Set(lastPoint4.GetX()+ 0.05f, lastPoint4.GetY(), 0.0f);
        nest4EntryPoints.push_back(lastPoint4);
    }

    for(int i = 0; i < 9; i++) {
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

    // Nest 1 EXIT PATH
    for(int i = 0; i < 6; i++) {
        lastPoint1.Set(lastPoint1.GetX()+ 0.05f, lastPoint1.GetY(), 0.0f);
        nest1ExitPoints.push_back(lastPoint1);
    }
    for(int i = 0; i < 30; i++) {
        lastPoint1.Set(lastPoint1.GetX() + 0.05f, lastPoint1.GetY() + 0.05f, 0.0f);
        nest1ExitPoints.push_back(lastPoint1);
    }

    // Nest 2 EXIT PATH
    for(int i = 0; i < 6; i++) {
        lastPoint2.Set(lastPoint2.GetX(), lastPoint2.GetY() + 0.05f, 0.0f);
        nest2ExitPoints.push_back(lastPoint2);
    }
    for(int i = 0; i < 30; i++) {
        lastPoint2.Set(lastPoint2.GetX() + 0.05f, lastPoint2.GetY() + 0.05f, 0.0f);
        nest2ExitPoints.push_back(lastPoint2);
    }

    
    // Nest 3 EXIT PATH
    for(int i = 0; i < 6; i++) {
        lastPoint3.Set(lastPoint3.GetX(), lastPoint3.GetY() + 0.05f, 0.0f);
        nest3ExitPoints.push_back(lastPoint3);
    }
    for(int i = 0; i < 25; i++) {
        lastPoint3.Set(lastPoint3.GetX() + 0.05f, lastPoint3.GetY() + 0.05f, 0.0f);
        nest3ExitPoints.push_back(lastPoint3);
    }

    // Nest 4 EXIT PATH
    for(int i = 0; i < 6; i++) {
        lastPoint4.Set(lastPoint4.GetX() + 0.05f, lastPoint4.GetY(), 0.0f);
        nest4ExitPoints.push_back(lastPoint4);
    }
    for(int i = 0; i < 25; i++) {
        lastPoint4.Set(lastPoint4.GetX() + 0.05f, lastPoint4.GetY() + 0.05f, 0.0f);
        nest4ExitPoints.push_back(lastPoint4);
    }
   
}

void CPFA_loop_functions::SetSpiralPathCoordinates() {
	SpiralPathCoordinates.clear();
	SpiralPathCoordinatesForController.clear();
	
    const Real fRedCircleRadius = RedCircleRadius;
    const CVector2 center = RedCirclePosition;

    CVector3 entryPoint(center.GetX(), 
                    center.GetY() + fRedCircleRadius,
                    0.1f);
                       
    SpiralPathCoordinates.push_back(entryPoint);
    
    // Specific point inside the red circle (you can adjust this)
    CVector3 startPoint(center.GetX(),  // 30% radius to the right
                       center.GetY() + (fRedCircleRadius * 0.9f),   // 40% radius down
                       0.1f);
    
    SpiralPathCoordinates.push_back(startPoint);
    
    const UInt32 unNumPoints = 270;
    Real arc_degree = 306.0f;
   
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.9f) * Sin(CRadians(-radian)),
                       (fRedCircleRadius*0.9f) * Cos(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }

    float t = 0.1f; // 20% of the distance to the center
    CVector3 lastPoint = SpiralPathCoordinates.back();
    CVector3 anotherPoint(
        lastPoint.GetX() + t * (0.3 - lastPoint.GetX()),
        lastPoint.GetY() + t * (0.0 - lastPoint.GetY()),
        0.1f
    );

    SpiralPathCoordinates.push_back(anotherPoint);

    // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.8f) * Cos(CRadians(-radian)),
                       (fRedCircleRadius*0.8f) * Sin(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }

    lastPoint = SpiralPathCoordinates.back();
    
    anotherPoint.Set(
        lastPoint.GetX() + t * (0.0 - lastPoint.GetX()),
        lastPoint.GetY() + t * (0.3 - lastPoint.GetY()),
        0.1f
    );
    SpiralPathCoordinates.push_back(anotherPoint);

    // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    arc_degree = 304;
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.7f) * Sin(CRadians(-radian)),
                       (fRedCircleRadius*0.7f) * Cos(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }

    lastPoint = SpiralPathCoordinates.back();
    t = 0.15f;
    anotherPoint.Set(
        lastPoint.GetX() + t * (0.3 - lastPoint.GetX()),
        lastPoint.GetY() + t * (0.0 - lastPoint.GetY()),
        0.1f
    );
    SpiralPathCoordinates.push_back(anotherPoint);

    // arc_degree = 125.0f;
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.6f) * Cos(CRadians(-radian)),
                       (fRedCircleRadius*0.6f) * Sin(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }


    lastPoint = SpiralPathCoordinates.back();
    anotherPoint.Set(
        lastPoint.GetX() + t * (0.0 - lastPoint.GetX()),
        lastPoint.GetY() + t * (0.3 - lastPoint.GetY()),
        0.1f
    );
    SpiralPathCoordinates.push_back(anotherPoint);

    arc_degree = 302;
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.5f) * Sin(CRadians(-radian)),
                       (fRedCircleRadius*0.5f) * Cos(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }

    lastPoint = SpiralPathCoordinates.back();
    t = 0.2f;
    anotherPoint.Set(
        lastPoint.GetX() + t * (0.3 - lastPoint.GetX()),
        lastPoint.GetY() + t * (0.0 - lastPoint.GetY()),
        0.1f
    );
    SpiralPathCoordinates.push_back(anotherPoint);

    arc_degree = 125.0f;
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        // Real radian = i * (M_PI / 180.0f);
        Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
        CVector3 point((fRedCircleRadius*0.4f) * Cos(CRadians(-radian)),
                       (fRedCircleRadius*0.4f) * Sin(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        SpiralPathCoordinates.push_back(point);
    }
    
    // const Real fRedCircleRadius = NestRadius * RedCircleRadiusMultiplier;
    // const CVector2 center = RedCirclePosition;

    // // Entry point (bottom of the outer circle)
    // CVector3 entryPoint(center.GetX(), 
    //                    center.GetY() + fRedCircleRadius,
    //                    0.1f);
                       
    // LOG << "entryPoint: " << entryPoint.GetX() << ", " << entryPoint.GetY() << std::endl;
    // SpiralPathCoordinates.push_back(entryPoint);
    
    // // Specific point inside the red circle (you can adjust this)
    // CVector3 startPoint(center.GetX(),  // 30% radius to the right
    //                    center.GetY() + (fRedCircleRadius * 0.9f),   // 40% radius down
    //                    0.1f);

    // LOG << "startPoint: " << startPoint.GetX() << ", " << startPoint.GetY() << std::endl;
    
    // SpiralPathCoordinates.push_back(startPoint);
    
    // const UInt32 unNumPoints = 270;
    // Real arc_degree = 302.0f;
   
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.9f) * Sin(CRadians(-radian)),
    //                    (fRedCircleRadius*0.9f) * Cos(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    // }

    // float t = 0.15f; // 20% of the distance to the center
    // CVector3 lastPoint = SpiralPathCoordinates.back();
    // CVector3 anotherPoint(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );

    // SpiralPathCoordinates.push_back(anotherPoint);

    // // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.75f) * Cos(CRadians(-radian)),
    //                    (fRedCircleRadius*0.75f) * Sin(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    // }

    // lastPoint = SpiralPathCoordinates.back();
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);

    // // // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    // arc_degree -= 3;

    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.6f) * Sin(CRadians(-radian)),
    //                    (fRedCircleRadius*0.6f) * Cos(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    // }

    // lastPoint = SpiralPathCoordinates.back();
    // anotherPoint.Set(
    //     lastPoint.GetX() + t * (center.GetX() - lastPoint.GetX()),
    //     lastPoint.GetY() + t * (center.GetY() - lastPoint.GetY()),
    //     0.1f
    // );
    // SpiralPathCoordinates.push_back(anotherPoint);

    // arc_degree = 125.0f;
    // for(UInt32 i = 0; i <= unNumPoints; ++i) {
    //     // Real radian = i * (M_PI / 180.0f);
    //     Real radian = i * (arc_degree * M_PI / (180.0f *unNumPoints));
    //     CVector3 point((fRedCircleRadius*0.45f) * Cos(CRadians(-radian)),
    //                    (fRedCircleRadius*0.45f) * Sin(CRadians(-radian)),
    //                    0.1f);
    //     // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
    //     SpiralPathCoordinates.push_back(point);
    // }

    SetNestsPredefinedEntryPathCoordinates();
    SetNestsPredefinedExitPathCoordinates();

	for(UInt32 i = 0; i <= SpiralPathCoordinates.size(); ++i) {
		SpiralPathCoordinatesForController.push_back(
			CVector2(SpiralPathCoordinates[i].GetX(), SpiralPathCoordinates[i].GetY()));
	}

	LOG << "SpiralPathCoordinatesForController size: " << SpiralPathCoordinatesForController.size() << std::endl;
}

bool CPFA_loop_functions::IsNearExitPoint(const argos::CVector2 &position) {
    
    CVector2 exitpoint(nest1ExitPoints.back().GetX(), nest1ExitPoints.back().GetY());
    Real distance = (position - exitpoint).Length();

    if(distance < 0.2f) return true;

    return false;
}

// Method to get a safe target that avoids exit point during boundary following
argos::CVector2 CPFA_loop_functions::GetExitPointAvoidanceTarget(const argos::CVector2& currentPos, argos::Real currentAngle, argos::Real targetAngle) {
    argos::Real safeDistance = RedCircleRadius + 0.05;
    argos::Real angularStep = 0.1;
    
    // Calculate next position normally
    argos::Real nextAngle;
    argos::Real angleDiff = targetAngle - currentAngle;
    while(angleDiff > M_PI) angleDiff -= 2.0 * M_PI;
    while(angleDiff < -M_PI) angleDiff += 2.0 * M_PI;
    
    if(angleDiff > 0) {
        nextAngle = currentAngle + angularStep;
    } else {
        nextAngle = currentAngle - angularStep;
    }
    
    // Calculate the normal next position
    argos::CVector2 nextExternalPos(
        RedCirclePosition.GetX() + safeDistance * cos(nextAngle),
        RedCirclePosition.GetY() + safeDistance * sin(nextAngle)
    );
    
    // Check if this position is near exit point
    if(IsNearExitPoint(nextExternalPos)) {
        // If near exit point, take a detour by moving further out from the circle
        argos::Real detourDistance = safeDistance + 0.5; // Move further out to avoid exit point
        nextExternalPos.Set(
            RedCirclePosition.GetX() + detourDistance * cos(nextAngle),
            RedCirclePosition.GetY() + detourDistance * sin(nextAngle)
        );
        
        // If still near exit point, try a different angle (skip ahead)
        if(IsNearExitPoint(nextExternalPos)) {
            argos::Real skipAngle = nextAngle + (angleDiff > 0 ? angularStep * 2 : -angularStep * 2);
            nextExternalPos.Set(
                RedCirclePosition.GetX() + safeDistance * cos(skipAngle),
                RedCirclePosition.GetY() + safeDistance * sin(skipAngle)
            );
        }
    }
    
    return nextExternalPos;
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

	// if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
	// 	isFinished = true;
	// }
    if(FoodList.size() == 0) {
		isFinished = true;
	}
    //set to collected 88% food and then stop
    if(score >= (NumDistributedFood * 0.9)){
        printf("90% Resources Are Collected! Took %f minutes.\n", curr_time_in_minutes/60.0);
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
	  
     printf("Resource Collected: %f, Time Took: %f minutes, Random Seed: %lu\n", score, getSimTimeInSeconds()/3600, RandomSeed);
       
                  
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
			argos::Real redCircleRadius = RedCircleRadius + 0.5;
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
    argos::Real redCircleRadius = RedCircleRadius;
    argos::CVector2 exitPoint(RedCirclePosition.GetX()+redCircleRadius, RedCirclePosition.GetY()+redCircleRadius-0.7);
    return exitPoint;
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
