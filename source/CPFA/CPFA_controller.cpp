#include "CPFA_controller.h"
#include <unistd.h>

CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	ResourceDensity(0),
	MaxTrailSize(50),
	SearchTime(0),
	CPFA_state(DEPARTING),
	LoopFunctions(NULL),
	survey_count(0),
	isUsingPheromone(0),
    SiteFidelityPosition(1000, 1000), 
        searchingTime(0),
        travelingTime(0),
        startTime(0),
    m_pcLEDs(NULL),
        updateFidelity(false),
		logReport(true),
		hasSetEntryPoint(false),
		isHeadingToEntryPoint(false),
		isFollowingCircleBoundary(false),
		entryPoint(0.0, 0.0),
		redCircleRadius(0.0),
		hasStartedBoundaryFollow(false),
		isPausingOnBoundary(false),
		isBouncing(false),
		nextSearchType(RANDOM_SEARCH)
{
	// redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
	argos::TConfigurationNode settings = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
	argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestAngleTolerance",    NestAngleTolerance);
	argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
	argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
	argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
	argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
	argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
	argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
	argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);

	argos::CVector2 p(GetPosition());
	SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
	
	FoodDistanceTolerance *= FoodDistanceTolerance;
	SetIsHeadingToNest(true);
	//qilu 10/21/2016 Let robots start to search immediately
	SetTarget(p);
        controllerID= GetId();
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
		m_pcLEDs->SetAllColors(CColor::GREEN);

	// if(!LoopFunctions->SpiralPathCoordinates.empty()) {
	// 	LOG << "SpiralPathCoordinates size: " << LoopFunctions->SpiralPathCoordinates.size() << endl;
	// } else {
	// 	LOG << "SpiralPathCoordinates is empty" << endl;
	// }
}

void CPFA_controller::ControlStep() {
	/*
	ofstream log_output_stream;
	log_output_stream.open("cpfa_log.txt", ios::app);

	// depart from nest after food drop off or simulation start
	if (isHoldingFood) log_output_stream << "(Carrying) ";
	
	switch(CPFA_state)  {
		case DEPARTING:
			if (isUsingSiteFidelity) {
				log_output_stream << "DEPARTING (Fidelity): "
					<< GetTarget().GetX() << ", " << GetTarget().GetY()
					<< endl;
			} else if (isInformed) {
				log_output_stream << "DEPARTING (Waypoint): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			} else {
				log_output_stream << "DEPARTING (Searching): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			}
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			if (isInformed) log_output_stream << "SEARCHING: Informed" << endl;     
			else log_output_stream << "SEARCHING: UnInformed" << endl;
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			log_output_stream << "RETURNING" << endl;
			break;
		case SURVEYING:
			log_output_stream << "SURVEYING" << endl;
			break;
		default:
			log_output_stream << "Unknown state" << endl;
	}
	*/

	// Add line so we can draw the trail

	CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
	CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
	CRay3 targetRay(target3d, position3d);
	myTrail.push_back(targetRay);
	LoopFunctions->TargetRayList.push_back(targetRay);
	LoopFunctions->TargetRayColorList.push_back(TrailColor);

	previous_position = GetPosition();

	//UpdateTargetRayList();
	CPFA();
	Move();
}

void CPFA_controller::Reset() {
 num_targets_collected =0;
 isHoldingFood   = false;
    isInformed      = false;
    SearchTime      = 0;
    ResourceDensity = 0;
    collisionDelay = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
    
    
    /* Set LED color */
    /* m_pcLEDs->SetAllColors(CColor::BLACK); //qilu 09/04 */
	// Now we will set the target to a fixed nest position (0, 0.3)
    // SetTarget(); //qilu 09/08
	argos::CVector2 currentPos = GetPosition();

	// Calculate direction to closest nest
	argos::CVector2 directionToNest = (LoopFunctions->RedCirclePosition - currentPos).Normalize();

	// Set target to a point slightly outside the red circle (safe distance)
	argos::Real redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
	argos::Real safeDistance = redCircleRadius + 0.1;
	argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToNest * safeDistance;
	
	SetTarget(targetOnCircle);
	SetIsHeadingToNest(false);

    updateFidelity = false;
    TrailToShare.clear();
    TrailToFollow.clear();
    	MyTrail.clear();

	myTrail.clear();

	isInformed = false;
	isHoldingFood = false;
	isUsingSiteFidelity = false;
	isGivingUpSearch = false;
}

bool CPFA_controller::IsHoldingFood() {
		return isHoldingFood;
}

bool CPFA_controller::IsUsingSiteFidelity() {
		return isUsingSiteFidelity;
}

void CPFA_controller::CPFA() {
	
	switch(CPFA_state) {
		// depart from nest after food drop off or simulation start
		case DEPARTING:
			// argos::LOG << "DEPARTING" << std::endl;
			//SetIsHeadingToNest(false);
			Departing();
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			// argos::LOG << "SEARCHING" << std::endl;
			//SetIsHeadingToNest(false);
			if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
				Searching();
			}
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			// argos::LOG << "RETURNING" << std::endl;
			//SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			// argos::LOG << "SURVEYING" << std::endl;
			//SetIsHeadingToNest(false);
			Surveying();
			break;
		case STOPPING:
			// argos::LOG << "STOPPING" << std::endl;
			Stopping();
			break;
	}
}

bool CPFA_controller::IsInTheNest() {

	for(const auto& nestPos : LoopFunctions->NestPositions) {
		if((GetPosition() - nestPos).SquareLength() < LoopFunctions->NestRadiusSquared) {
			return true;
		}
	}
	return false;
}

void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;

	// Initialize the SiteFidelityPosition

	// Create the output file here because it needs LoopFunctions
		
	// Name the results file with the current time and date
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	stringstream ss;

	char hostname[1024];                                                   
	hostname[1023] = '\0';                    
	gethostname(hostname, 1023);  

	/* ss << "CPFA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
		<< hostname << '-'
		<< getpid() << '-'
		<< (now->tm_year) << '-'
		<< (now->tm_mon + 1) << '-'
		<<  now->tm_mday << '-'
		<<  now->tm_hour << '-'
		<<  now->tm_min << '-'
		<<  now->tm_sec << ".csv";

		string results_file_name = ss.str();
		results_full_path = results_path+"/"+results_file_name;
         */
         
	// Only the first robot should do this:	 
	if (GetId().compare("CPFA_0") == 0) {
		/*
		ofstream results_output_stream;
		results_output_stream.open(results_full_path, ios::app);
		results_output_stream << "NumberOfRobots, "
			<< "TargetDistanceTolerance, "
			<< "TargetAngleTolerance, "
			<< "FoodDistanceTolerance, "
			<< "RobotForwardSpeed, "
			<< "RobotRotationSpeed, "
			<< "RandomSeed, "
			<< "ProbabilityOfSwitchingToSearching, "
			<< "ProbabilityOfReturningToNest, "
			<< "UninformedSearchVariation, "   
			<< "RateOfInformedSearchDecay, "   
			<< "RateOfSiteFidelity, "          
			<< "RateOfLayingPheromone, "       
			<< "RateOfPheromoneDecay" << endl
			<< LoopFunctions->getNumberOfRobots() << ", "
			<< CSimulator::GetInstance().GetRandomSeed() << ", "  
			<< TargetDistanceTolerance << ", "
			<< TargetAngleTolerance << ", "
			<< FoodDistanceTolerance << ", "
			<< RobotForwardSpeed << ", "
			<< RobotRotationSpeed << ", "
			<< LoopFunctions->getProbabilityOfSwitchingToSearching() << ", "
			<< LoopFunctions->getProbabilityOfReturningToNest() << ", "
			<< LoopFunctions->getUninformedSearchVariation() << ", "
			<< LoopFunctions->getRateOfInformedSearchDecay() << ", "
			<< LoopFunctions->getRateOfSiteFidelity() << ", "
			<< LoopFunctions->getRateOfLayingPheromone() << ", "
			<< LoopFunctions->getRateOfPheromoneDecay()
			<< endl;
				
			results_output_stream.close();
		*/
	}

}

void CPFA_controller::Departing()
{
    argos::Real distanceToTarget = (GetPosition() - GetTarget()).Length();
    argos::Real randomNumber = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

    // Calculate red circle radius if not already set
    if(redCircleRadius == 0.0) {
        redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
    }

    // Check if current target is inside red circle (except when returning to nest)
    // if(!isGivingUpSearch && !isHoldingFood) {
    //     argos::Real distanceToCenter = (GetTarget() - argos::CVector2(0.0, 0.3)).Length();
    //     if(distanceToCenter <= redCircleRadius) {
    //         // Target is inside red circle, set a new random search location outside
    //         SetRandomSearchLocation();
    //         return;
    //     }
    // }

    /* When not informed, continue to travel until randomly switching to the searching state. */
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
        if(isInformed == false){
            if(SimulationTick()%(5*SimulationTicksPerSecond())==0 && randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching){
                Stop();
                SearchTime = 0;
                CPFA_state = SEARCHING;
                travelingTime+=SimulationTick()-startTime;
                startTime = SimulationTick();
            
                argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
                argos::Real rand = RNG->Gaussian(USV);
                argos::CRadians rotation(rand);
                argos::CRadians angle1(rotation.UnsignedNormalize());
                argos::CRadians angle2(GetHeading().UnsignedNormalize());
                argos::CRadians turn_angle(angle1 + angle2);
                argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                SetIsHeadingToNest(false);
                
                // Ensure new search target is outside red circle
                argos::CVector2 newTarget = turn_vector + GetPosition();
				argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
				if(distanceToCenter <= redCircleRadius) {
					// If new target is inside red circle, adjust it to be outside
					argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
					newTarget = LoopFunctions->RedCirclePosition + directionFromCenter * (redCircleRadius + 0.1);
				}
				SetTarget(newTarget);
                // argos::Real distanceToCenter = (newTarget - argos::CVector2(0.0, 0.3)).Length();
                // if(distanceToCenter <= redCircleRadius) {
                //     // If new target is inside red circle, adjust it to be outside
                //     argos::CVector2 directionFromCenter = (newTarget - argos::CVector2(0.0, 0.3)).Normalize();
                //     newTarget = argos::CVector2(0.0, 0.3) + directionFromCenter * (redCircleRadius + 0.1);
                // }
                // SetTarget(newTarget);
            }
            else if(distanceToTarget < TargetDistanceTolerance){
                SetRandomSearchLocation(false);
            }
        }
    }
	
    /* Are we informed? I.E. using site fidelity or pheromones. */	
    if(isInformed && distanceToTarget < TargetDistanceTolerance) {
        SearchTime = 0;
        CPFA_state = SEARCHING;
        travelingTime+=SimulationTick()-startTime;
        startTime = SimulationTick();

        if(isUsingSiteFidelity) {
            isUsingSiteFidelity = false;
            SetFidelityList();
        }
    }
}

void CPFA_controller::Searching() {
    // "scan" for food only every half of a second
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
        SetHoldingFood();
    }
    // When not carrying food, calculate movement.
    if(IsHoldingFood() == false) {
        argos::CVector2 distance = GetPosition() - GetTarget();
        argos::Real     random   = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
     
        // Calculate red circle radius if not already set
        if(redCircleRadius == 0.0) {
            redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
        }

        // Check if current target is inside red circle
		argos::Real distanceToCenter = (GetTarget() - LoopFunctions->RedCirclePosition).Length();
		if(distanceToCenter <= redCircleRadius) {
			// Target is inside red circle, set a new random search location outside
			SetRandomSearchLocation(false);
			return;
		}
     
        // If we reached our target search location, set a new one. The 
        // new search location calculation is different based on whether
        // we are currently using informed or uninformed search.
        if(distance.SquareLength() < TargetDistanceTolerance) {
            // randomly give up searching
            if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest) {
                argos::LOG << GetId() << " Given up search. Heading to red circle." << std::endl;
                SetFidelityList();
                TrailToShare.clear();
                argos::CVector2 currentPos = GetPosition();
                argos::CVector2 directionToCenter = (LoopFunctions->RedCirclePosition - currentPos).Normalize();
                
                // Set target to a point slightly outside the red circle (safe distance)
                argos::Real safeDistance = redCircleRadius + 0.1;
                argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToCenter * safeDistance;
                
                SetTarget(targetOnCircle);
                SetIsHeadingToNest(false);

                isGivingUpSearch = true;
                LoopFunctions->FidelityList.erase(controllerID);
                isUsingSiteFidelity = false; 
                updateFidelity = false; 
                CPFA_state = RETURNING;
                searchingTime+=SimulationTick()-startTime;
                startTime = SimulationTick();
                return; 
            }

            argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
            argos::Real rand = RNG->Gaussian(USCV);

            // uninformed search
            if(isInformed == false) {
                argos::CRadians rotation(rand);
                argos::CRadians angle1(rotation);
                argos::CRadians angle2(GetHeading());
                argos::CRadians turn_angle(angle1 + angle2);
                argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                SetIsHeadingToNest(false);
                
                // Ensure new search target is outside red circle
                argos::CVector2 newTarget = turn_vector + GetPosition();
                argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
                if(distanceToCenter <= redCircleRadius) {
                    // If new target is inside red circle, adjust it to be outside
                    argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
                    newTarget = LoopFunctions->NestPosition + directionFromCenter * (redCircleRadius + 0.1);
                }
                SetTarget(newTarget);
            }
            // informed search
            else {
                SetIsHeadingToNest(false);
                
                if(IsAtTarget()) {
                    size_t t = SearchTime++;
                    argos::Real twoPi = (argos::CRadians::TWO_PI).GetValue();
                    argos::Real pi = (argos::CRadians::PI).GetValue();
                    argos::Real isd = LoopFunctions->RateOfInformedSearchDecay;
                    Real correlation = GetExponentialDecay(rand, t, isd);
                    argos::CRadians rotation(GetBound(correlation, -pi, pi));
                    argos::CRadians angle1(rotation);
                    argos::CRadians angle2(GetHeading());
                    argos::CRadians turn_angle(angle2 + angle1);
                    argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                    
                    // Ensure new search target is outside red circle
                    argos::CVector2 newTarget = turn_vector + GetPosition();
                    argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
                    if(distanceToCenter <= redCircleRadius) {
                        // If new target is inside red circle, adjust it to be outside
                        argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
                        newTarget = LoopFunctions->RedCirclePosition + directionFromCenter * (redCircleRadius + 0.1);
                    }
                    SetTarget(newTarget);
                }
            }
        }
    }
}

// Cause the robot to rotate in place as if surveying the surrounding targets
// Turns 36 times by 10 degrees
void CPFA_controller::Surveying() {
 //LOG<<"Surveying..."<<endl;
	if (survey_count <= 4) { 
		CRadians rotation(survey_count*3.14/2); // divide by 10 so the vecot is small and the linear motion is minimized
		argos::CVector2 turn_vector(SearchStepSize, rotation.SignedNormalize());
			
		SetIsHeadingToNest(true); // Turn off error for this
		SetTarget(turn_vector + GetPosition());
		/*
		ofstream log_output_stream;
		log_output_stream.open("log.txt", ios::app);
		log_output_stream << (GetHeading() - rotation ).SignedNormalize() << ", "  << SearchStepSize << ", "<< rotation << ", " <<  turn_vector << ", " << GetHeading() << ", " << survey_count << endl;
		log_output_stream.close();
		*/
		
		if(fabs((GetHeading() - rotation).SignedNormalize().GetValue()) < TargetAngleTolerance.GetValue()) survey_count++;
			//else Keep trying to reach the turning angle
	}
	// Set the survey countdown
	else {
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->RedCirclePosition); 
		
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
                searchingTime+=SimulationTick()-startTime;//qilu 10/22
                startTime = SimulationTick();//qilu 10/22
            
	}
}


/*****
 * RETURNING: Stay in this state until the robot has returned to the nest.
 * This state is triggered when a robot has found food or when it has given
 * up on searching and is returning to the nest.
 *****/
void CPFA_controller::Returning() {
    //LOG<<"Returning..."<<endl;
    //SetHoldingFood();
	int selectedNestIndex;
	if(isGivingUpSearch) {
		// If the robot is near the red circle, bounce back after getting information.
		if(LoopFunctions->IsNearRedCircle(GetPosition())) {
			// argos::LOG << GetId() << " Reached red circle without food - bouncing back." << std::endl;
			
			// BOUNCE BACK LOGIC: Move away from the red circle
			argos::CVector2 currentPos = GetPosition();
			argos::CVector2 directionAwayFromCenter = (currentPos - LoopFunctions->RedCirclePosition).Normalize();
			
			// Calculate red circle radius if not already set
			redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
			
			// Set target to a position further away from the red circle
			argos::Real bounceDistance = redCircleRadius + 1.0; // Move 1 unit beyond the red circle
			argos::CVector2 bounceTarget = LoopFunctions->RedCirclePosition + directionAwayFromCenter * bounceDistance;
			
			SetTarget(bounceTarget);
			SetIsHeadingToNest(false);
			
			// Based on a Poisson CDF, the robot may or may not create a pheromone
			// located at the last place it picked up food.
			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
			argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
			if(poissonCDF_pLayRate > r1 && updateFidelity) {
				TrailToShare.push_back(argos::CVector2(0.0, 0.3));
				argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
				Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
				LoopFunctions->PheromoneList.push_back(sharedPheromone);
				sharedPheromone.Deactivate();
			}
			TrailToShare.clear();
			
			// Store the decision for next search behavior but DON'T execute it yet
			if(updateFidelity && poissonCDF_sFollowRate > r2) {
				// Will use site fidelity after bounce
				nextSearchType = SITE_FIDELITY;
			}
			else if(LoopFunctions->PheromoneList.size() > 0) {
				// Check if there are pheromones to follow (don't call SetTargetPheromone yet)
				nextSearchType = PHEROMONE_TRAIL;
			}
			else {
				// Will use random search after bounce
				nextSearchType = RANDOM_SEARCH;
			}
			
			// Set bounce state - don't reset other flags yet
			isBouncing = true;
			
			// argos::LOG << GetId() << " Bouncing to: " << bounceTarget.GetX() << ", " << bounceTarget.GetY() << std::endl;
			
		} else if(isBouncing) {
			// Check if we've moved far enough away from the red circle or reached bounce target
			argos::Real distanceFromCenter = (GetPosition() - LoopFunctions->RedCirclePosition).Length();
			argos::Real minBounceDistance = redCircleRadius + LoopFunctions->RedCircleRadiusMultiplier; // Minimum distance to be considered "bounced back"
			
			if(distanceFromCenter >= minBounceDistance || IsAtTarget()) {
				// argos::LOG << GetId() << " Bounce back complete. Starting next search phase." << std::endl;
				
				// Now execute the stored search decision
				if(nextSearchType == SITE_FIDELITY) {
					SetTarget(SiteFidelityPosition);
					SetIsHeadingToNest(false);
					isInformed = true;
					isUsingSiteFidelity = true;
					// argos::LOG << GetId() << " Using site fidelity for next search." << std::endl;
				}
				else if(nextSearchType == PHEROMONE_TRAIL && SetTargetPheromone()) {
					SetIsHeadingToNest(false);
					isInformed = true;
					isUsingSiteFidelity = false;
					// argos::LOG << GetId() << " Using pheromone trail for next search." << std::endl;
				}
				else {
					// Use random search (fallback)
					SetRandomSearchLocation(false);
					SetIsHeadingToNest(false);
					isInformed = false;
					isUsingSiteFidelity = false;
					// argos::LOG << GetId() << " Using random search." << std::endl;
				}
				
				// Reset all flags for next food collection cycle
				hasStartedBoundaryFollow = false;
				isFollowingCircleBoundary = false;
				isPausingOnBoundary = false;
				isHeadingToEntryPoint = false;
				isGivingUpSearch = false;
				isBouncing = false;
				nextSearchType = RANDOM_SEARCH; // Reset search type
				CPFA_state = DEPARTING;   
				travelingTime += SimulationTick() - startTime;
				startTime = SimulationTick();
			}
			// Continue moving to bounce target if not far enough yet
		}
	} else {

		// SIMPLIFIED LOGIC: When robot picks up food, set target to red circle, then start boundary following
		if(isHoldingFood && !hasStartedBoundaryFollow && !isFollowingCircleBoundary) {
			// Calculate red circle parameters
			redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
			
			// Check if robot is already near the red circle
			if(LoopFunctions->IsNearRedCircle(GetPosition())) {
				// Robot has reached the red circle, start boundary following immediately
				
				// Calculate entry point position (north point of red circle)
				entryPoint.Set(LoopFunctions->RedCirclePosition.GetX(), 
				              LoopFunctions->RedCirclePosition.GetY() + redCircleRadius);
				
				SetIsHeadingToNest(false);
				hasStartedBoundaryFollow = true;
				isFollowingCircleBoundary = true;
				
				// Start boundary following immediately by setting first target
				argos::CVector2 currentPos = GetPosition();
				argos::CVector2 relativePos = currentPos - LoopFunctions->RedCirclePosition;
				argos::Real currentAngle = atan2(relativePos.GetY(), relativePos.GetX());
				argos::Real safeDistance = redCircleRadius + 0.1;
				
				// Move slightly along the circle to start the boundary following
				argos::Real nextAngle = currentAngle + 0.1;
				argos::CVector2 nextExternalPos(0.0 + safeDistance * cos(nextAngle),
											0.0 + safeDistance * sin(nextAngle));
				SetTarget(nextExternalPos);
				
			} else {
				// Robot is not at red circle yet, head towards it
				argos::CVector2 currentPos = GetPosition();
				argos::CVector2 directionToCenter = (LoopFunctions->RedCirclePosition - currentPos).Normalize();
				
				// Set target to a point slightly outside the red circle (safe distance)
				argos::Real safeDistance = redCircleRadius + 0.1;
				argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToCenter * safeDistance;
				
				SetTarget(targetOnCircle);
				SetIsHeadingToNest(false);
			}
		}

		// Follow the red circle boundary FROM OUTSIDE until reaching entry point
		if(isFollowingCircleBoundary && isHoldingFood) {
			
			argos::CVector2 currentPos = GetPosition();
			
			// Calculate distance from center - robot should ALWAYS be outside the circle
			argos::Real currentDistanceFromCenter = (currentPos - LoopFunctions->RedCirclePosition).Length();
			
			// CRITICAL: Maintain safe distance outside the circle
			argos::Real safeDistance = redCircleRadius + 0.1; // Stay 0.1 units outside the red circle
			
			// Calculate current angle relative to circle center
			argos::CVector2 relativePos = currentPos - LoopFunctions->RedCirclePosition;
			argos::Real currentAngle = atan2(relativePos.GetY(), relativePos.GetX());
			
			// Target angle is north (π/2 radians = 90 degrees) - entry point
			argos::Real targetAngle = M_PI / 2.0;
			
			// Calculate angular difference (shortest path around the circle)
			argos::Real angleDiff = targetAngle - currentAngle;
			while(angleDiff > M_PI) angleDiff -= 2.0 * M_PI;
			while(angleDiff < -M_PI) angleDiff += 2.0 * M_PI;
			
			// Check if we're close enough to entry point
			argos::Real distanceToEntry = (currentPos - entryPoint).Length();
			
			if(distanceToEntry < 0.2) { // Reached entry point area
				argos::LOG << GetId() << " Reached entry point. Checking queue position." << std::endl;
				
				// Initialize predefined path if not already done
				// if(predefinedPath.empty()) {
				// 	GeneratePredefinedPath();
				// }
				
				// Check if we can enter the predefined path (queue management)
				if(CanEnterPredefinedPath()) {
					// Add this robot to the queue and start following predefined path
					LoopFunctions->AddRobotToPathQueue(GetId());
					isFollowingPredefinedPath = true;
					predefinedPathIndex = 0;
					isFollowingCircleBoundary = false;
					
					// Set first waypoint of predefined path
					SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
					// argos::LOG << GetId() << " Entering predefined path. First waypoint: " 
					// 		   << predefinedPath[predefinedPathIndex].GetX() << ", " 
					// 		   << predefinedPath[predefinedPathIndex].GetY() << std::endl;
				} else {
					// Wait in queue - stay at entry point
					SetTarget(entryPoint);
					argos::LOG << GetId() << " Waiting in queue at entry point." << std::endl;
				}
			} else {
				// Continue following boundary FROM OUTSIDE
				argos::Real angularStep = 0.1; // Step size for movement along circle
				
				// Move along the EXTERNAL path in the direction of shortest angular distance
				argos::Real nextAngle;
				if(angleDiff > 0) {
					nextAngle = currentAngle + angularStep;
				} else {
					nextAngle = currentAngle - angularStep;
				}
				
				// Calculate next position OUTSIDE the circle at safe distance
				argos::CVector2 nextExternalPos(LoopFunctions->RedCirclePosition.GetX() + safeDistance * cos(nextAngle),
											LoopFunctions->RedCirclePosition.GetY() + safeDistance * sin(nextAngle));
				
				SetTarget(nextExternalPos);
			}
		}

		// NEW: Follow predefined path from entry point to nest
		if(isFollowingPredefinedPath && isHoldingFood) {
			argos::CVector2 currentPos = GetPosition();
			argos::CVector2 currentTarget = LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex];
			argos::Real distanceToCurrentWaypoint = (currentPos - currentTarget).Length();
			
			// Check if we've reached the current waypoint
			if(distanceToCurrentWaypoint < 0.15) { // Waypoint reached threshold
				predefinedPathIndex++;

				// Check if we've completed the path (reached nest) - Works Fine Up to Here
				if(predefinedPathIndex >= LoopFunctions->SpiralPathCoordinatesForController.size()-1) {
					argos::LOG << GetId() << " Completed predefined path. Heading to nest." << std::endl;
					
					
					int availablePathToNest = LoopFunctions->GetAvailablePathToNest();

					if(availablePathToNest != -1) {
						selectedNestIndex = availablePathToNest;
						LoopFunctions->RemoveRobotFromPathQueue(GetId());
						LoopFunctions->AddRobotToPathQueueToNest(selectedNestIndex, GetId());

						isFollowingPredefinedPath = false;
						isHeadingToEntryPoint = true; // This will trigger nest arrival logic
						// Set a nest position as the target (among 4 nest positions)
						// LOG << GetId() << " Heading to nest(x,y): " 
						// 				<< LoopFunctions->NestPositions[availablePathToNest].GetX() 
						// 				<< ", "
						// 				<< LoopFunctions->NestPositions[availablePathToNest].GetY()
						// 				<< std::endl;
						SetTarget(LoopFunctions->NestPositions[availablePathToNest]); // temporary value, need to fix later.
						SetIsHeadingToNest(false);
						// for(int i = 0; i < 4; i++) {
						// 	argos::LOG << "Path available to nest " << i << ": " << LoopFunctions->IsPathAvailableToNest(i) << std::endl;
						// }
					} else {
						argos::LOG << GetId() << " No available path to nest. Waiting in queue." << std::endl;
					}
					
				} else {
					// Move to next waypoint
					SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
					// argos::LOG << GetId() << " Moving to waypoint " << predefinedPathIndex 
					// 		   << ": " << predefinedPath[predefinedPathIndex].GetX() 
					// 		   << ", " << predefinedPath[predefinedPathIndex].GetY() << std::endl;
				}
			}
			// Continue moving toward current waypoint if not reached yet
		}

		// Check if robot reached nest from predefined path
		if(isHeadingToEntryPoint && isHoldingFood) {
			if(IsInTheNest()) {

				argos::LOG << GetId() << " Reached nest. Dropping food and exiting." << std::endl;
				
				// Remove this robot from the path queue
				LoopFunctions->RemoveRobotFromPathQueueToNest(selectedNestIndex, GetId());
				
				// Based on a Poisson CDF, the robot may or may not create a pheromone
				// located at the last place it picked up food.
				argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
				argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
				argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
				argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
				
				// Drop off the food and display in the nest, now temp value added, later need to add four nest logics.
				// argos::CVector2 placementPosition;
				// placementPosition.Set(0.0f+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
				// 					0.3f+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
				
				// while((placementPosition-CVector2(0.0f, 0.3f)).SquareLength()>pow(LoopFunctions->NestRadius/2.0-LoopFunctions->FoodRadius, 2))
				// 	placementPosition.Set(0.0f+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
				// 						0.3f+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));

				// LoopFunctions->CollectedFoodList.push_back(placementPosition);
				
				// Update the location of the nest
				num_targets_collected++;
				LoopFunctions->currNumCollectedFood++;
				LoopFunctions->setScore(num_targets_collected);
				
				if(poissonCDF_pLayRate > r1 && updateFidelity) {
					TrailToShare.push_back(LoopFunctions->NestPositions[selectedNestIndex]);
					argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
					Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
					LoopFunctions->PheromoneList.push_back(sharedPheromone);
					sharedPheromone.Deactivate();
				}
				TrailToShare.clear();

				// Food is now dropped, update state
				isHoldingFood = false;
				
				// EXIT STRAIGHT: Set exit direction from nest outward (away from entry point)
				// argos::CVector2 exitDirection = (GetPosition() - entryPoint).Normalize();
				// argos::CVector2 exitTarget = LoopFunctions->RedCirclePosition + exitDirection * (redCircleRadius + 1.0);
				
				// SetTarget(exitTarget);
				// SetIsHeadingToNest(false);

				// Determine probabilistically whether to use site fidelity, pheromone trails, or random search
				if(updateFidelity && poissonCDF_sFollowRate > r2) {
					// Use site fidelity
					LOG << GetId() << " Using site fidelity." << std::endl;
					SetIsHeadingToNest(false);
					SetTarget(SiteFidelityPosition);
					isInformed = true;
				}
				else if(SetTargetPheromone()) {
					// Use pheromone waypoints
					LOG << GetId() << " Using pheromone waypoints." << std::endl;
					SetIsHeadingToNest(false);
					isInformed = true;
					isUsingSiteFidelity = false;
				}
				else {
					// Use random search
					SetRandomSearchLocation(false);
					SetIsHeadingToNest(false);
					isInformed = false;
					isUsingSiteFidelity = false;
				}

				// Reset all flags for next food collection cycle
				hasStartedBoundaryFollow = false;
				isFollowingCircleBoundary = false;
				isPausingOnBoundary = false;
				isHeadingToEntryPoint = false;
				isGivingUpSearch = false;
				isFollowingPredefinedPath = false;
				predefinedPathIndex = 0;
				CPFA_state = DEPARTING;   
				travelingTime += SimulationTick() - startTime;
				startTime = SimulationTick();
				
				argos::LOG << GetId() << " Exiting nest and ready for next foraging cycle." << std::endl;
			} else {
				// LOG << GetId() << " is still heading to nest..." << std::endl;
				// SetTarget(CVector2(0.0, 0.3));
			}
		}
	}
}

// NEW: Generate predefined spiral/curved path from entry point to nest
void CPFA_controller::GeneratePredefinedPath() {
	// redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
	predefinedPath.clear();
	
	// Parameters for spiral path
	// argos::Real numTurns = 2.0; // Number of spiral turns
	// int numWaypoints = 20; // Number of waypoints along the path
	// argos::Real startRadius = redCircleRadius * 0.8; // Start radius (near entry point)
	// argos::Real endRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier; // End radius (near nest center)
	
	for(int i = 0; i < LoopFunctions->SpiralPathCoordinates.size(); i++) {
		// argos::Real t = (argos::Real)i / (argos::Real)(numWaypoints - 1); // Parameter from 0 to 1
		
		// // Spiral inward: radius decreases as we progress
		// argos::Real currentRadius = startRadius - (startRadius - endRadius) * t;
		
		// // Angle: start from north (π/2) and spiral inward
		// argos::Real currentAngle = M_PI/2.0 - numTurns * 2.0 * M_PI * t;
		
		// Calculate waypoint position
		argos::CVector2 waypoint(
			LoopFunctions->SpiralPathCoordinates[i].GetX(),
			LoopFunctions->SpiralPathCoordinates[i].GetY()
		);
		
		predefinedPath.push_back(waypoint);
	}
	
	// Ensure the last waypoint is at the nest center
	// predefinedPath.push_back(argos::CVector2(0.0, 0.3)); // need to fix later.
	
	argos::LOG << "Generated predefined spiral path with " << predefinedPath.size() << " waypoints." << std::endl;
}

// NEW: Check if robot can enter the predefined path (queue management)
bool CPFA_controller::CanEnterPredefinedPath() {
	// Check if there's space in the path queue
	// This assumes LoopFunctions has methods to manage the queue
	return LoopFunctions->GetPathQueueSize() < LoopFunctions->GetMaxPathQueueSize();
}


// void CPFA_controller::Returning() {
//     //LOG<<"Returning..."<<endl;
//     //SetHoldingFood();

// 	if(isGivingUpSearch) {
// 		// If the robot is near the red circle, bounce back after getting information.
// 		if(LoopFunctions->IsNearRedCircle(GetPosition())) {
// 			argos::LOG << GetId() << " Reached red circle without food - bouncing back." << std::endl;
			
// 			// BOUNCE BACK LOGIC: Move away from the red circle
// 			argos::CVector2 currentPos = GetPosition();
// 			argos::CVector2 directionAwayFromCenter = (currentPos - LoopFunctions->NestPosition).Normalize();
			
// 			// Calculate red circle radius if not already set
// 			redCircleRadius = LoopFunctions->NestRadius * 5.0;
			
// 			// Set target to a position further away from the red circle
// 			argos::Real bounceDistance = redCircleRadius + 1.0; // Move 1 unit beyond the red circle
// 			argos::CVector2 bounceTarget = LoopFunctions->NestPosition + directionAwayFromCenter * bounceDistance;
			
// 			SetTarget(bounceTarget);
// 			SetIsHeadingToNest(false);
			
// 			// Based on a Poisson CDF, the robot may or may not create a pheromone
// 			// located at the last place it picked up food.
// 			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
// 			argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
// 			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
// 			argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
// 			if(poissonCDF_pLayRate > r1 && updateFidelity) {
// 				TrailToShare.push_back(LoopFunctions->NestPosition);
// 				argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
// 				Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
// 				LoopFunctions->PheromoneList.push_back(sharedPheromone);
// 				sharedPheromone.Deactivate();
// 			}
// 			TrailToShare.clear();
			
// 			// Store the decision for next search behavior but DON'T execute it yet
// 			if(updateFidelity && poissonCDF_sFollowRate > r2) {
// 				// Will use site fidelity after bounce
// 				nextSearchType = SITE_FIDELITY;
// 			}
// 			else if(LoopFunctions->PheromoneList.size() > 0) {
// 				// Check if there are pheromones to follow (don't call SetTargetPheromone yet)
// 				nextSearchType = PHEROMONE_TRAIL;
// 			}
// 			else {
// 				// Will use random search after bounce
// 				nextSearchType = RANDOM_SEARCH;
// 			}
			
// 			// Set bounce state - don't reset other flags yet
// 			isBouncing = true;
			
// 			argos::LOG << GetId() << " Bouncing to: " << bounceTarget.GetX() << ", " << bounceTarget.GetY() << std::endl;
			
// 		} else if(isBouncing) {
// 			// Check if we've moved far enough away from the red circle or reached bounce target
// 			argos::Real distanceFromCenter = (GetPosition() - LoopFunctions->NestPosition).Length();
// 			argos::Real minBounceDistance = redCircleRadius + 0.5; // Minimum distance to be considered "bounced back"
			
// 			if(distanceFromCenter >= minBounceDistance || IsAtTarget()) {
// 				argos::LOG << GetId() << " Bounce back complete. Starting next search phase." << std::endl;
				
// 				// Now execute the stored search decision
// 				if(nextSearchType == SITE_FIDELITY) {
// 					SetTarget(SiteFidelityPosition);
// 					SetIsHeadingToNest(false);
// 					isInformed = true;
// 					isUsingSiteFidelity = true;
// 					argos::LOG << GetId() << " Using site fidelity for next search." << std::endl;
// 				}
// 				else if(nextSearchType == PHEROMONE_TRAIL && SetTargetPheromone()) {
// 					SetIsHeadingToNest(false);
// 					isInformed = true;
// 					isUsingSiteFidelity = false;
// 					argos::LOG << GetId() << " Using pheromone trail for next search." << std::endl;
// 				}
// 				else {
// 					// Use random search (fallback)
// 					SetRandomSearchLocation();
// 					SetIsHeadingToNest(false);
// 					isInformed = false;
// 					isUsingSiteFidelity = false;
// 					argos::LOG << GetId() << " Using random search." << std::endl;
// 				}
				
// 				// Reset all flags for next food collection cycle
// 				hasStartedBoundaryFollow = false;
// 				isFollowingCircleBoundary = false;
// 				isPausingOnBoundary = false;
// 				isHeadingToEntryPoint = false;
// 				isGivingUpSearch = false;
// 				isBouncing = false;
// 				nextSearchType = RANDOM_SEARCH; // Reset search type
// 				CPFA_state = DEPARTING;   
// 				travelingTime += SimulationTick() - startTime;
// 				startTime = SimulationTick();
// 			}
// 			// Continue moving to bounce target if not far enough yet
// 		}
// 	} else {

// 		// SIMPLIFIED LOGIC: When robot picks up food, set target to red circle, then start boundary following
// 		if(isHoldingFood && !hasStartedBoundaryFollow && !isFollowingCircleBoundary) {
// 			// Calculate red circle parameters
// 			redCircleRadius = LoopFunctions->NestRadius * 5.0;
// 			// LoopFunctions->NestPosition = LoopFunctions->NestPosition;
			
// 			// Check if robot is already near the red circle
// 			if(LoopFunctions->IsNearRedCircle(GetPosition())) {
// 				// Robot has reached the red circle, start boundary following immediately
// 				// argos::LOG << GetId() << " At red circle with food. Starting boundary following." << std::endl;
				
// 				// Calculate entry point position (north point of red circle)
// 				entryPoint.Set(LoopFunctions->NestPosition.GetX(), 
// 							LoopFunctions->NestPosition.GetY() + redCircleRadius);
				
// 				SetIsHeadingToNest(false);
// 				hasStartedBoundaryFollow = true;
// 				isFollowingCircleBoundary = true;
				
// 				// Start boundary following immediately by setting first target
// 				argos::CVector2 currentPos = GetPosition();
// 				argos::CVector2 relativePos = currentPos - LoopFunctions->NestPosition;
// 				argos::Real currentAngle = atan2(relativePos.GetY(), relativePos.GetX());
// 				argos::Real safeDistance = redCircleRadius + 0.1;
				
// 				// Move slightly along the circle to start the boundary following
// 				argos::Real nextAngle = currentAngle + 0.1;
// 				argos::CVector2 nextExternalPos(LoopFunctions->NestPosition.GetX() + safeDistance * cos(nextAngle),
// 											LoopFunctions->NestPosition.GetY() + safeDistance * sin(nextAngle));
// 				SetTarget(nextExternalPos);
				
// 				// argos::LOG << GetId() << " Starting boundary follow at angle: " << currentAngle << std::endl;
// 			} else {
// 				// Robot is not at red circle yet, head towards it
// 				argos::CVector2 currentPos = GetPosition();
// 				argos::CVector2 directionToCenter = (LoopFunctions->NestPosition - currentPos).Normalize();
				
// 				// Set target to a point slightly outside the red circle (safe distance)
// 				argos::Real safeDistance = redCircleRadius + 0.1;
// 				argos::CVector2 targetOnCircle = LoopFunctions->NestPosition - directionToCenter * safeDistance;
				
// 				SetTarget(targetOnCircle);
// 				SetIsHeadingToNest(false);
				
// 				// argos::LOG << GetId() << " Holding food - heading to red circle at: " << targetOnCircle.GetX() << ", " << targetOnCircle.GetY() << std::endl;
// 			}
// 		}

// 		// Follow the red circle boundary FROM OUTSIDE until reaching entry point
// 		if(isFollowingCircleBoundary && isHoldingFood) {
// 			// argos::LOG << GetId() << " Following external boundary" << std::endl;
			
// 			argos::CVector2 currentPos = GetPosition();
			
// 			// Calculate distance from center - robot should ALWAYS be outside the circle
// 			argos::Real currentDistanceFromCenter = (currentPos - LoopFunctions->NestPosition).Length();
// 			// argos::LOG << GetId() << " Distance from center: " << currentDistanceFromCenter << " (radius: " << redCircleRadius << ")" << std::endl;
			
// 			// CRITICAL: Maintain safe distance outside the circle
// 			argos::Real safeDistance = redCircleRadius + 0.1; // Stay 0.15 units outside the red circle
			
// 			// Calculate current angle relative to circle center
// 			argos::CVector2 relativePos = currentPos - LoopFunctions->NestPosition;
// 			argos::Real currentAngle = atan2(relativePos.GetY(), relativePos.GetX());
			
// 			// Target angle is north (π/2 radians = 90 degrees) - entry point
// 			argos::Real targetAngle = M_PI / 2.0;
			
// 			// Calculate angular difference (shortest path around the circle)
// 			argos::Real angleDiff = targetAngle - currentAngle;
// 			while(angleDiff > M_PI) angleDiff -= 2.0 * M_PI;
// 			while(angleDiff < -M_PI) angleDiff += 2.0 * M_PI;
			
// 			// Check if we're close enough to entry point
// 			argos::Real distanceToEntry = (currentPos - entryPoint).Length();
// 			// argos::LOG << GetId() << " Distance to entry point: " << distanceToEntry << std::endl;
			
// 			if(distanceToEntry < 0.2) { // Reached entry point area
// 				// argos::LOG << GetId() << " Reached entry point area. Now going straight to nest." << std::endl;
// 				SetTarget(LoopFunctions->NestPosition);
// 				SetIsHeadingToNest(true);
// 				isFollowingCircleBoundary = false;
// 				isHeadingToEntryPoint = true;
// 			} else {
// 				// Continue following boundary FROM OUTSIDE
// 				argos::Real angularStep = 0.1; // Step size for movement along circle
				
// 				// Move along the EXTERNAL path in the direction of shortest angular distance
// 				argos::Real nextAngle;
// 				if(angleDiff > 0) {
// 					nextAngle = currentAngle + angularStep;
// 				} else {
// 					nextAngle = currentAngle - angularStep;
// 				}
				
// 				// Calculate next position OUTSIDE the circle at safe distance
// 				argos::CVector2 nextExternalPos(LoopFunctions->NestPosition.GetX() + safeDistance * cos(nextAngle),
// 											LoopFunctions->NestPosition.GetY() + safeDistance * sin(nextAngle));
				
// 				SetTarget(nextExternalPos);
				
// 				// argos::LOG << GetId() << " Next external position: " << nextExternalPos.GetX() << ", " << nextExternalPos.GetY() << std::endl;
// 				// argos::LOG << GetId() << " Current angle: " << currentAngle << ", Next angle: " << nextAngle << std::endl;
// 			}
// 		}

// 		// Check if robot reached nest from entry point
// 		if(isHeadingToEntryPoint && IsInTheNest() && isHoldingFood) {
// 			// argos::LOG << GetId() << " Reached nest from entry point. Dropping food." << std::endl;
			
// 			// Based on a Poisson CDF, the robot may or may not create a pheromone
// 			// located at the last place it picked up food.
// 			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
// 			argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
// 			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
// 			argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
// 			// Drop off the food and display in the nest 
// 			argos::CVector2 placementPosition;
// 			placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
// 								LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
			
// 			while((placementPosition-LoopFunctions->NestPosition).SquareLength()>pow(LoopFunctions->NestRadius/2.0-LoopFunctions->FoodRadius, 2))
// 				placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
// 									LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));

// 			LoopFunctions->CollectedFoodList.push_back(placementPosition);
			
// 			// Update the location of the nest
// 			num_targets_collected++;
// 			LoopFunctions->currNumCollectedFood++;
// 			LoopFunctions->setScore(num_targets_collected);
			
// 			if(poissonCDF_pLayRate > r1 && updateFidelity) {
// 				TrailToShare.push_back(LoopFunctions->NestPosition);
// 				argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
// 				Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
// 				LoopFunctions->PheromoneList.push_back(sharedPheromone);
// 				sharedPheromone.Deactivate();
// 			}
// 			TrailToShare.clear();

// 			// Food is now dropped, update state
// 			isHoldingFood = false;
			
// 			// Determine probabilistically whether to use site fidelity, pheromone trails, or random search
// 			if(updateFidelity && poissonCDF_sFollowRate > r2) {
// 				// Use site fidelity
// 				SetIsHeadingToNest(false);
// 				SetTarget(SiteFidelityPosition);
// 				isInformed = true;
// 				// argos::LOG << GetId() << " Using site fidelity for next search." << std::endl;
// 			}
// 			else if(SetTargetPheromone()) {
// 				// Use pheromone waypoints
// 				SetIsHeadingToNest(false);
// 				isInformed = true;
// 				isUsingSiteFidelity = false;
// 				// argos::LOG << GetId() << " Using pheromone trail for next search." << std::endl;
// 			}
// 			else {
// 				// Use random search
// 				SetRandomSearchLocation();
// 				SetIsHeadingToNest(false);
// 				isInformed = false;
// 				isUsingSiteFidelity = false;
// 				// argos::LOG << GetId() << " Using random search." << std::endl;
// 			}

// 			// Reset all flags for next food collection cycle
// 			hasStartedBoundaryFollow = false;
// 			isFollowingCircleBoundary = false;
// 			isPausingOnBoundary = false;
// 			isHeadingToEntryPoint = false;
// 			isGivingUpSearch = false;
// 			CPFA_state = DEPARTING;   
// 			travelingTime += SimulationTick() - startTime;
// 			startTime = SimulationTick();
			
// 			// argos::LOG << GetId() << " Ready for next foraging cycle." << std::endl;
// 		}
// 	}
	
// 	// // Main navigation logic - Working
// 	// if(LoopFunctions->IsNearRedCircle(GetPosition()) && isHoldingFood && !hasSetEntryPoint) {
// 	// 	if(logReport) {
// 	// 		argos::LOG << GetId() << " Reached red circle with food." << std::endl;
// 	// 	}
		
// 	// 	// Calculate entry point position (north point of red circle)
// 	// 	argos::CVector2 entryPoint(LoopFunctions->NestPosition.GetX(), 
// 	// 								LoopFunctions->NestPosition.GetY() + LoopFunctions->NestRadius * 5.0);
		
// 	// 	SetIsHeadingToNest(false);
// 	// 	SetTarget(entryPoint);
// 	// 	hasSetEntryPoint = true;
// 	// 	isHeadingToEntryPoint = true;
		
// 	// 	if(logReport) {
// 	// 		argos::LOG << "Target Point: " << GetTarget().GetX() << ", " << GetTarget().GetY() << std::endl;
// 	// 	}
// 	// 	logReport = false;
// 	// }

// 	// // Check if robot reached entry point
// 	// if(isHeadingToEntryPoint && IsAtTarget()) {
// 	// 	argos::LOG << GetId() << " Reached target point. Heading to nest." << std::endl;
// 	// 	SetIsHeadingToNest(true);
// 	// 	SetTarget(LoopFunctions->NestPosition);
// 	// 	isHeadingToEntryPoint = false;
// 	// }
	
// 	// // Check if robot reached nest and drop food
// 	// if(GetIsHeadingToNest() && IsInTheNest() && isHoldingFood) {
// 	// 	// argos::LOG << GetId() << " Reached nest." << std::endl;
// 	// 	argos::LOG << GetId() << " Reached nest." << std::endl;
// 	// 	// Based on a Poisson CDF, the robot may or may not create a pheromone
// 	// 	// located at the last place it picked up food.
// 	// 	argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
// 	// 	argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
// 	// 	argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
// 	// 	argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
		
// 	// 	// Drop off the food and display in the nest 
// 	// 	argos::CVector2 placementPosition;
// 	// 	placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
// 	// 						LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
		
// 	// 	while((placementPosition-LoopFunctions->NestPosition).SquareLength()>pow(LoopFunctions->NestRadius/2.0-LoopFunctions->FoodRadius, 2))
// 	// 		placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), 
// 	// 							LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));

// 	// 	LoopFunctions->CollectedFoodList.push_back(placementPosition);
		
// 	// 	// Update the location of the nest
// 	// 	num_targets_collected++;
// 	// 	LoopFunctions->currNumCollectedFood++;
// 	// 	LoopFunctions->setScore(num_targets_collected);
		
// 	// 	if(poissonCDF_pLayRate > r1 && updateFidelity) {
// 	// 		TrailToShare.push_back(LoopFunctions->NestPosition);
// 	// 		argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
// 	// 		Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
// 	// 		LoopFunctions->PheromoneList.push_back(sharedPheromone);
// 	// 		sharedPheromone.Deactivate();
// 	// 	}
// 	// 	TrailToShare.clear();

// 	// 	// Determine probabilistically whether to use site fidelity, pheromone trails, or random search
// 	// 	//ofstream log_output_stream;
// 	// 	//log_output_stream.open("cpfa_log.txt", ios::app);
// 	// 	//log_output_stream << "At the nest." << endl;	    
	 
// 	//     // use site fidelity
// 	// 	if(updateFidelity && poissonCDF_sFollowRate > r2) {
// 	// 		//log_output_stream << "Using site fidelity" << endl;
// 	// 			SetIsHeadingToNest(false);
// 	// 			SetTarget(SiteFidelityPosition);
// 	// 			isInformed = true;
// 	// 	}
//     //   // use pheromone waypoints
//     //   else if(SetTargetPheromone()) {
//     //       //log_output_stream << "Using site pheremone" << endl;
//     //       isInformed = true;
//     //       isUsingSiteFidelity = false;
//     //   }
//     //    // use random search
//     //   else {
//     //        //log_output_stream << "Using random search" << endl;
//     //         SetRandomSearchLocation();
//     //         isInformed = false;
//     //         isUsingSiteFidelity = false;
//     //   }

// 	// 	isGivingUpSearch = false;
// 	// 	CPFA_state = DEPARTING;   
//     //     isHoldingFood = false; 
//     //     travelingTime+=SimulationTick()-startTime;//qilu 10/22
//     //     startTime = SimulationTick();//qilu 10/22
                
//     // }
// 	// Take a small step towards the nest so we don't overshoot by too much is we miss it
//     // else 
//     // {
//     //     if(IsAtTarget())
//     //     {
//     //     // argos::LOG << GetId() << " Heading to nest." << std::endl;
//     //     //SetIsHeadingToNest(false); // Turn off error for this
//     //     //SetTarget(LoopFunctions->NestPosition);
//     //     //randomly search for the nest
//     //     argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
//     //     argos::Real rand = RNG->Gaussian(USCV);

//     //     argos::CRadians rotation(rand);
//     //     argos::CRadians angle1(rotation);
//     //     argos::CRadians angle2(GetHeading());
//     //     argos::CRadians turn_angle(angle1 + angle2);
//     //     argos::CVector2 turn_vector(SearchStepSize, turn_angle);
//     //     SetIsHeadingToNest(false);
//     //     SetTarget(turn_vector + GetPosition());
//     //     }
//     //     //detect other robots in its camera view
        
//     // }		   
// }

void CPFA_controller::Stopping() {
	argos::LOG << GetId() << " Reached red circle, stopping." << std::endl;
	CVector2 nextTarget(LoopFunctions->RedCirclePosition.GetX(), 
				LoopFunctions->RedCirclePosition.GetY() + LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier);
	SetIsHeadingToNest(false);
	SetTarget(nextTarget);
}

void CPFA_controller::SetRandomSearchLocation(bool isExit) {
    argos::Real random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
    argos::Real x = 0.0, y = 0.0;
    argos::CVector2 searchPosition;
    bool validPosition = false;

    // Calculate red circle radius if not already set
    if(redCircleRadius == 0.0) {
        redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
    }

	if(isExit) {
		if(random_wall < 0.5) {
			x = RNG->Uniform(ForageRangeX);
			y = ForageRangeY.GetMax();
		} else {
			x = RNG->Uniform(ForageRangeX);
			y = ForageRangeY.GetMin();
		}
	} else {

		// Keep trying until we find a valid position outside the red circle
		while(!validPosition) {
			/* north wall */
			if(random_wall < 0.25) {
				x = RNG->Uniform(ForageRangeX);
				y = ForageRangeY.GetMax();
			}
			/* south wall */
			else if(random_wall < 0.5) {
				x = RNG->Uniform(ForageRangeX);
				y = ForageRangeY.GetMin();
			}
			/* east wall */
			else if(random_wall < 0.75) {
				x = ForageRangeX.GetMax();
				y = RNG->Uniform(ForageRangeY);
			}
			/* west wall */
			else {
				x = ForageRangeX.GetMin();
				y = RNG->Uniform(ForageRangeY);
			}

			searchPosition.Set(x, y);
			
			// Check if the position is outside the red circle
			argos::Real distanceFromCenter = (searchPosition - LoopFunctions->RedCirclePosition).Length();
			if(distanceFromCenter > redCircleRadius) {
				validPosition = true;
			} else {
				// Try a different random wall if current position is invalid
				random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			}
		}
	}

        
    SetIsHeadingToNest(true); // Turn off error for this
    SetTarget(searchPosition);
}

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void CPFA_controller::SetHoldingFood() {
    // Is the iAnt already holding food?
    if(IsHoldingFood() == false) {
        // No, the iAnt isn't holding food. Check if we have found food at our
        // current position and update the food list if we have.

        std::vector<argos::CVector2> newFoodList;
        std::vector<argos::CColor> newFoodColoringList;
        size_t i = 0, j = 0;

        for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
            if((GetPosition() - LoopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance ) {
                // We found food! Calculate the nearby food density.
                isHoldingFood = true;
                CPFA_state = SURVEYING;
                j = i + 1;
                searchingTime+=SimulationTick()-startTime;
                startTime = SimulationTick();

                // distribute a new food - ensure it's outside the red circle
                argos::CVector2 placementPosition;
                bool validPosition = false;
                
                // Calculate red circle radius if not already set
                if(redCircleRadius == 0.0) {
                    redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
                }

                while(!validPosition) {
                    placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
                    
                    // Check if position is valid (outside red circle and not out of bounds)
                    if(!LoopFunctions->IsOutOfBounds(placementPosition, 1, 1)) {
						for (const auto& nestPos : LoopFunctions->NestPositions) {
							argos::Real distanceFromCenter = (placementPosition - nestPos).Length();
							if (distanceFromCenter > redCircleRadius) {
								validPosition = true;
							}
						}
                        // argos::Real distanceFromCenter = (placementPosition - LoopFunctions->NestPosition).Length();
                        // if(distanceFromCenter > redCircleRadius) {
                        //     validPosition = true;
                        // }
                    }
                }

                newFoodList.push_back(placementPosition);
                newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
                LoopFunctions->increaseNumDistributedFoodByOne();
                break;
            } else {
                //Return this unfound-food position to the list
                newFoodList.push_back(LoopFunctions->FoodList[i]);
                newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
            }
        }

        if(j>0){
            for(; j < LoopFunctions->FoodList.size(); j++) {
                newFoodList.push_back(LoopFunctions->FoodList[j]);
                newFoodColoringList.push_back(LoopFunctions->FoodColoringList[j]);
            }
        }
   
        // We picked up food. Update the food list minus what we picked up.
        if(IsHoldingFood()) {
            LoopFunctions->FoodList = newFoodList;
            LoopFunctions->FoodColoringList = newFoodColoringList;
            SetLocalResourceDensity();
        }
    }
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void CPFA_controller::SetLocalResourceDensity() {
	argos::CVector2 distance;

	// remember: the food we picked up is removed from the foodList before this function call
	// therefore compensate here by counting that food (which we want to count)
	ResourceDensity = 1;

	/* Calculate resource density based on the global food list positions. */
	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
		   distance = GetPosition() - LoopFunctions->FoodList[i];

		   if(distance.SquareLength() < LoopFunctions->SearchRadiusSquared*2) {
			      ResourceDensity++;
			      LoopFunctions->FoodColoringList[i] = argos::CColor::ORANGE;
			      LoopFunctions->ResourceDensityDelay = SimulationTick() + SimulationTicksPerSecond() * 10;
		   }
	}
 
	/* Set the fidelity position to the robot's current position. */
    SiteFidelityPosition = GetPosition();
    isUsingSiteFidelity = true;
    updateFidelity = true; 
    TrailToShare.push_back(SiteFidelityPosition);
    LoopFunctions->FidelityList[controllerID] = SiteFidelityPosition;
    /* Delay for 4 seconds (simulate iAnts scannning rotation). */
	//  Wait(4); // This function is broken. It causes the rover to move in the wrong direction after finishing its local resource density test 

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "(Survey): " << ResourceDensity << endl;
	//log_output_stream << "SiteFidelityPosition: " << SiteFidelityPosition << endl;
	//log_output_stream.close();
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void CPFA_controller::SetFidelityList(argos::CVector2 newFidelity) {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/*for(size_t i = 0; i < LoopFunctions->FidelityList.size(); i++) {
  if((LoopFunctions->FidelityList[i] - SiteFidelityPosition).SquareLength() != 0.0) {
			newFidelityList.push_back(LoopFunctions->FidelityList[i]);
		}
	} */


	/* Update the global fidelity list. */
	//LoopFunctions->FidelityList = newFidelityList;

        LoopFunctions->FidelityList[controllerID] = newFidelity;
	/* Add the robot's new fidelity position to the global fidelity list. */
	//LoopFunctions->FidelityList.push_back(newFidelity);
 

	/* Update the local fidelity position for this robot. */
	SiteFidelityPosition = newFidelity;
 
  updateFidelity = true;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void CPFA_controller::SetFidelityList() {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/* Update the global fidelity list. */
        LoopFunctions->FidelityList.erase(controllerID);
 SiteFidelityPosition = CVector2(10000, 10000);
 updateFidelity = true; 
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool CPFA_controller::SetTargetPheromone() {
	argos::Real maxStrength = 0.0, randomWeight = 0.0;
	bool isPheromoneSet = false;

 if(LoopFunctions->PheromoneList.size()==0) return isPheromoneSet; //the case of no pheromone.
	/* update the pheromone list and remove inactive pheromones */

	/* default target = nest; in case we have 0 active pheromones */
	//SetIsHeadingToNest(true);
	//SetTarget(LoopFunctions->NestPosition);
	/* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		if(LoopFunctions->PheromoneList[i].IsActive()) {
			maxStrength += LoopFunctions->PheromoneList[i].GetWeight();
		}
	}

	/* Calculate a random weight. */
	randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, maxStrength));

	/* Randomly select an active pheromone to follow. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		   if(randomWeight < LoopFunctions->PheromoneList[i].GetWeight()) {
			       /* We've chosen a pheromone! */
			       SetIsHeadingToNest(false);
          SetTarget(LoopFunctions->PheromoneList[i].GetLocation());
          TrailToFollow = LoopFunctions->PheromoneList[i].GetTrail();
          isPheromoneSet = true;
          /* If we pick a pheromone, break out of this loop. */
          break;
     }

     /* We didn't pick a pheromone! Remove its weight from randomWeight. */
     randomWeight -= LoopFunctions->PheromoneList[i].GetWeight();
	}

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "Found: " << LoopFunctions->PheromoneList.size()  << " waypoints." << endl;
	//log_output_stream << "Follow waypoint?: " << isPheromoneSet << endl;
	//log_output_stream.close();

	return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
argos::Real CPFA_controller::GetExponentialDecay(argos::Real w, argos::Real time, argos::Real lambda) {
	/* convert time into units of haLoopFunctions-seconds from simulation frames */
	//time = time / (LoopFunctions->TicksPerSecond / 2.0);

	//LOG << "time: " << time << endl;
	//LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	//return (value * std::exp(-lambda * time));
    Real     twoPi       = (CRadians::TWO_PI).GetValue();
    return w + (twoPi-w)* exp(-lambda * time);
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
argos::Real CPFA_controller::GetBound(argos::Real value, argos::Real min, argos::Real max) {
	/* Calculate an offset. */
	argos::Real offset = std::abs(min) + std::abs(max);

	/* Increment value by the offset while it's less than min. */
	while (value < min) {
			value += offset;
	}

	/* Decrement value by the offset while it's greater than max. */
	while (value > max) {
			value -= offset;
	}

	/* Return the bounded value. */
	return value;
}

size_t CPFA_controller::GetSearchingTime(){//qilu 10/22
    return searchingTime;
}
size_t CPFA_controller::GetTravelingTime(){//qilu 10/22
    return travelingTime;
}

string CPFA_controller::GetStatus(){//qilu 10/22
    //DEPARTING, SEARCHING, RETURNING
    if (CPFA_state == DEPARTING) return "DEPARTING";
    else if (CPFA_state ==SEARCHING)return "SEARCHING";
    else if (CPFA_state == RETURNING)return "RETURNING";
    else if (CPFA_state == SURVEYING) return "SURVEYING";
    //else if (MPFA_state == INACTIVE) return "INACTIVE";
    else return "SHUTDOWN";
    
}


/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
argos::Real CPFA_controller::GetPoissonCDF(argos::Real k, argos::Real lambda) {
	argos::Real sumAccumulator       = 1.0;
	argos::Real factorialAccumulator = 1.0;

	for (size_t i = 1; i <= floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += pow(lambda, i) / factorialAccumulator;
	}

	return (exp(-lambda) * sumAccumulator);
}

void CPFA_controller::UpdateTargetRayList() {
	if(SimulationTick() % LoopFunctions->DrawDensityRate == 0 && LoopFunctions->DrawTargetRays == 1) {
		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetPosition());
		argos::CVector3 position3d(p.GetX(), p.GetY(), 0.02);
		argos::CVector3 target3d(t.GetX(), t.GetY(), 0.02);

		/* scale the target ray to be <= searchStepSize */
		argos::Real length = std::abs(t.Length() - p.Length());

		if(length > SearchStepSize) {
			MyTrail.clear();
		} else {
			/* add the ray to the robot's target trail */
			argos::CRay3 targetRay(target3d, position3d);
			MyTrail.push_back(targetRay);

			/* delete the oldest ray from the trail */
			if(MyTrail.size() > MaxTrailSize) {
				MyTrail.erase(MyTrail.begin());
			}

			LoopFunctions->TargetRayList.insert(LoopFunctions->TargetRayList.end(), MyTrail.begin(), MyTrail.end());
			// loopFunctions.TargetRayList.push_back(myTrail);
		}
	}
}

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")
