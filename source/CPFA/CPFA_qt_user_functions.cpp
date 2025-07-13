#include "CPFA_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
CPFA_qt_user_functions::CPFA_qt_user_functions() :
	loopFunctions(dynamic_cast<CPFA_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
	RegisterUserFunction<CPFA_qt_user_functions, CFootBotEntity>(&CPFA_qt_user_functions::DrawOnRobot);
	RegisterUserFunction<CPFA_qt_user_functions, CFloorEntity>(&CPFA_qt_user_functions::DrawOnArena);
}

void CPFA_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
	CPFA_controller& c = dynamic_cast<CPFA_controller&>(entity.GetControllableEntity().GetController());

	if(c.IsHoldingFood()) {
		DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	}

	if(loopFunctions.DrawIDs == 1) {
		/* Disable lighting, so it does not interfere with the chosen text color */
		glDisable(GL_LIGHTING);
		/* Disable face culling to be sure the text is visible from anywhere */
		glDisable(GL_CULL_FACE);
		/* Set the text color */
		CColor cColor(CColor::BLACK);
		glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

		/* The position of the text is expressed wrt the reference point of the footbot
		 * For a foot-bot, the reference point is the center of its base.
		 * See also the description in
		 * $ argos3 -q foot-bot
		 */
		
		// Disable for now
		//GetOpenGLWidget().renderText(0.0, 0.0, 0.5,             // position
		//			     entity.GetId().c_str()); // text
		
			DrawText(CVector3(0.0, 0.0, 0.3),   // position
            entity.GetId().c_str()); // text
		/* Restore face culling */
		glEnable(GL_CULL_FACE);
		/* Restore lighting */
		glEnable(GL_LIGHTING);
	}
}
 
void CPFA_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
	DrawFood();
	DrawFidelity();
	DrawPheromones();
	DrawNest();

    if(loopFunctions.CircleCoordinates.empty()) {
        GenerateCircleCoordinates();
    }
    DrawCircleFromCoordinates();

    DrawSpiralPaths();
    DrawEntryPaths();
    DrawExitPaths();

	if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void CPFA_qt_user_functions::DrawNest() {
    // Draw each nest in the NestPositions vector
    for (const auto& nestPos : loopFunctions.NestPositions) {
        /* 2d cartesian coordinates of the nest */
        Real x_coordinate = nestPos.GetX();
        Real y_coordinate = nestPos.GetY();

        /* required: leaving this 0.0 will draw the nest inside of the floor */
        Real elevation = loopFunctions.NestElevation;

        /* 3d cartesian coordinates of the nest */
        CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

        /* Draw the nest on the arena. */
        DrawCylinder(nest_3d, CQuaternion(), loopFunctions.NestRadius, 0.008, CColor::GREEN);
    }

    // Draw entry point marker (north point of the circle)
    // CVector3 entryPoint(loopFunctions.RedCirclePosition.GetX(), 
    //                    loopFunctions.RedCirclePosition.GetY() + loopFunctions.RedCircleRadius,
    //                    0.1f);
    // DrawCylinder(entryPoint, CQuaternion(), 0.05, 0.05, CColor::BLUE);


    // SOLUTION 1: Only generate coordinates once (recommended)
    // if(loopFunctions.CircleCoordinates.empty()) {
    //     GenerateCircleCoordinates();
    // }
    
    // Draw the circle using pre-generated coordinates
    // DrawCircleFromCoordinates();

    // if(loopFunctions.SpiralPathCoordinates.empty()) {
    //     // GenerateSpiralPathCoordinates();
    //     LOG << "SpiralPathCoordinates is empty" << std::endl;
    // } else {
    //     DrawSpiralPaths();
    // }

    // if(!loopFunctions.nest1EntryPoints.empty()) {
    //     glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest1EntryPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest1EntryPoints[i].GetX(), loopFunctions.nest1EntryPoints[i].GetY(), loopFunctions.nest1EntryPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest2EntryPoints.empty()) {
    //     glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest2EntryPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest2EntryPoints[i].GetX(), loopFunctions.nest2EntryPoints[i].GetY(), loopFunctions.nest2EntryPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest3EntryPoints.empty()) {
    //     glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest3EntryPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest3EntryPoints[i].GetX(), loopFunctions.nest3EntryPoints[i].GetY(), loopFunctions.nest3EntryPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest4EntryPoints.empty()) {
    //     glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest4EntryPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest4EntryPoints[i].GetX(), loopFunctions.nest4EntryPoints[i].GetY(), loopFunctions.nest4EntryPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest1ExitPoints.empty()) {
    //     glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest1ExitPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest1ExitPoints[i].GetX(), loopFunctions.nest1ExitPoints[i].GetY(), loopFunctions.nest1ExitPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest2ExitPoints.empty()) {
    //     glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest2ExitPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest2ExitPoints[i].GetX(), loopFunctions.nest2ExitPoints[i].GetY(), loopFunctions.nest2ExitPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest3ExitPoints.empty()) {
    //     glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest3ExitPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest3ExitPoints[i].GetX(), loopFunctions.nest3ExitPoints[i].GetY(), loopFunctions.nest3ExitPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

    // if(!loopFunctions.nest4ExitPoints.empty()) {
    //     glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
    //     glLineWidth(1.0f);          // Optional: thicker line

    //     glBegin(GL_LINE_STRIP);     // Connect points in sequence
    //     for(size_t i = 0; i < loopFunctions.nest4ExitPoints.size(); ++i) {
    //         glVertex3f(loopFunctions.nest4ExitPoints[i].GetX(), loopFunctions.nest4ExitPoints[i].GetY(), loopFunctions.nest4ExitPoints[i].GetZ() + 0.01f); // Slight lift
    //     }
    //     glEnd();
    // }

}

void CPFA_qt_user_functions::DrawEntryPaths() {
    /* Disable lighting, so it does not interfere with the chosen text color */
    glDisable(GL_LIGHTING);
    /* Disable face culling to be sure the text is visible from anywhere */
    glDisable(GL_CULL_FACE);
    /* Set the text color */
    CColor cColor(CColor::BLUE);
    glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

    if(!loopFunctions.nest1EntryPoints.empty()) {
        // glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
        glLineWidth(1.0f);          // Optional: thicker line

        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        for(size_t i = 0; i < loopFunctions.nest1EntryPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest1EntryPoints[i].GetX(), loopFunctions.nest1EntryPoints[i].GetY(), loopFunctions.nest1EntryPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest2EntryPoints.empty()) {
        // glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
        glLineWidth(1.0f);          // Optional: thicker line

        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        for(size_t i = 0; i < loopFunctions.nest2EntryPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest2EntryPoints[i].GetX(), loopFunctions.nest2EntryPoints[i].GetY(), loopFunctions.nest2EntryPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest3EntryPoints.empty()) {
        // glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
        glLineWidth(1.0f);          // Optional: thicker line

        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        for(size_t i = 0; i < loopFunctions.nest3EntryPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest3EntryPoints[i].GetX(), loopFunctions.nest3EntryPoints[i].GetY(), loopFunctions.nest3EntryPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest4EntryPoints.empty()) {
        // glColor3f(0.0f, 0.0f, 1.0f); // Blue again for the line
        glLineWidth(1.0f);          // Optional: thicker line

        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        for(size_t i = 0; i < loopFunctions.nest4EntryPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest4EntryPoints[i].GetX(), loopFunctions.nest4EntryPoints[i].GetY(), loopFunctions.nest4EntryPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    /* Restore face culling */
    glEnable(GL_CULL_FACE);
    /* Restore lighting */
    glEnable(GL_LIGHTING);


}

void CPFA_qt_user_functions::DrawExitPaths() {
    /* Disable lighting, so it does not interfere with the chosen text color */
    glDisable(GL_LIGHTING);
    /* Disable face culling to be sure the text is visible from anywhere */
    glDisable(GL_CULL_FACE);
    /* Set the text color */
    CColor cColor(CColor::GREEN);
    glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());
    if(!loopFunctions.nest1ExitPoints.empty()) {

        glLineWidth(1.0f);          // Optional: thicker line
        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        // glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
        for(size_t i = 0; i < loopFunctions.nest1ExitPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest1ExitPoints[i].GetX(), loopFunctions.nest1ExitPoints[i].GetY(), loopFunctions.nest1ExitPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest2ExitPoints.empty()) {

        glLineWidth(1.0f);          // Optional: thicker line
        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        // glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
        for(size_t i = 0; i < loopFunctions.nest2ExitPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest2ExitPoints[i].GetX(), loopFunctions.nest2ExitPoints[i].GetY(), loopFunctions.nest2ExitPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest3ExitPoints.empty()) {

        glLineWidth(1.0f);          // Optional: thicker line
        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        // glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
        for(size_t i = 0; i < loopFunctions.nest3ExitPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest3ExitPoints[i].GetX(), loopFunctions.nest3ExitPoints[i].GetY(), loopFunctions.nest3ExitPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    if(!loopFunctions.nest4ExitPoints.empty()) {
        // glColor3f(0.0f, 1.0f, 0.0f); // Blue again for the line
        glLineWidth(1.0f);          // Optional: thicker line

        glBegin(GL_LINE_STRIP);     // Connect points in sequence
        for(size_t i = 0; i < loopFunctions.nest4ExitPoints.size(); ++i) {
            glVertex3f(loopFunctions.nest4ExitPoints[i].GetX(), loopFunctions.nest4ExitPoints[i].GetY(), loopFunctions.nest4ExitPoints[i].GetZ() + 0.01f); // Slight lift
        }
        glEnd();
    }

    /* Restore face culling */
    glEnable(GL_CULL_FACE);
    /* Restore lighting */
    glEnable(GL_LIGHTING);
}

void CPFA_qt_user_functions::GenerateSpiralPathCoordinates() {
    loopFunctions.SpiralPathCoordinates.clear();
    
    const Real fRedCircleRadius = loopFunctions.RedCircleRadius;
    const CVector2 center = loopFunctions.RedCirclePosition;

    // Entry point (bottom of the outer circle)
    CVector3 entryPoint(loopFunctions.RedCirclePosition.GetX(), 
                       loopFunctions.RedCirclePosition.GetY() + loopFunctions.RedCircleRadius,
                       0.1f);
    LOG << "entryPoint: " << entryPoint.GetX() << ", " << entryPoint.GetY() << std::endl;
    loopFunctions.SpiralPathCoordinates.push_back(entryPoint);
    
    // Specific point inside the red circle (you can adjust this)
    CVector3 startPoint(center.GetX(),  // 30% radius to the right
                       center.GetY() + (fRedCircleRadius * 0.9f),   // 40% radius down
                       0.1f);

    LOG << "startPoint: " << startPoint.GetX() << ", " << startPoint.GetY() << std::endl;
    
    loopFunctions.SpiralPathCoordinates.push_back(startPoint);
    
    const UInt32 unNumPoints = 270;
   
    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        Real radian = i * (M_PI / 180.0f);
        CVector3 point((fRedCircleRadius*0.9f) * Sin(CRadians(-radian)),
                       (fRedCircleRadius*0.9f) * Cos(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        loopFunctions.SpiralPathCoordinates.push_back(point);
    }

    CVector3 anotherPoint(center.GetX() + (fRedCircleRadius * 0.75f),
                          center.GetY(),
                          0.1f);
    loopFunctions.SpiralPathCoordinates.push_back(anotherPoint);

    // LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        Real radian = i * (M_PI / 180.0f);
        CVector3 point((fRedCircleRadius*0.75f) * Cos(CRadians(-radian)),
                       (fRedCircleRadius*0.75f) * Sin(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        loopFunctions.SpiralPathCoordinates.push_back(point);
    }

    anotherPoint.Set(center.GetX(),
                          center.GetY() + (fRedCircleRadius * 0.6f),
                          0.1f);
    loopFunctions.SpiralPathCoordinates.push_back(anotherPoint);

    LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        Real radian = i * (M_PI / 180.0f);
        CVector3 point((fRedCircleRadius*0.6f) * Sin(CRadians(-radian)),
                       (fRedCircleRadius*0.6f) * Cos(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        loopFunctions.SpiralPathCoordinates.push_back(point);
    }

    anotherPoint.Set(center.GetX(),
                          center.GetY() + (fRedCircleRadius * 0.5f),
                          0.1f);
    loopFunctions.SpiralPathCoordinates.push_back(anotherPoint);

    LOG << "anotherPoint: " << anotherPoint.GetX() << ", " << anotherPoint.GetY() << std::endl;

    for(UInt32 i = 0; i <= unNumPoints; ++i) {
        Real radian = i * (M_PI / 180.0f);
        CVector3 point((fRedCircleRadius*0.5f) * Cos(CRadians(-radian)),
                       (fRedCircleRadius*0.5f) * Sin(CRadians(-radian)),
                       0.1f);
        // LOG << "point at degree: " << i << " is " << point.GetX() << ", " << point.GetY() << std::endl;
        loopFunctions.SpiralPathCoordinates.push_back(point);
    }
    
}

void CPFA_qt_user_functions::DrawSpiralPaths() {

    /* Disable lighting, so it does not interfere with the chosen text color */
    glDisable(GL_LIGHTING);
    /* Disable face culling to be sure the text is visible from anywhere */
    glDisable(GL_CULL_FACE);
    /* Set the text color */
    CColor cColor(CColor::BLUE);
    glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

    glLineWidth(1.0f);          // Optional: thicker line

    glBegin(GL_LINE_STRIP);     // Connect points in sequence
    for(size_t i = 0; i < loopFunctions.SpiralPathCoordinates.size(); ++i) {
        glVertex3f(loopFunctions.SpiralPathCoordinates[i].GetX(), loopFunctions.SpiralPathCoordinates[i].GetY(), loopFunctions.SpiralPathCoordinates[i].GetZ() + 0.01f); // Slight lift
    }
    glEnd();
    /* Restore face culling */
    glEnable(GL_CULL_FACE);
    /* Restore lighting */
    glEnable(GL_LIGHTING);

}



// Helper function to generate circle coordinates only once
void CPFA_qt_user_functions::GenerateCircleCoordinates() {
    const Real fRedCircleRadius = loopFunctions.RedCircleRadius;
    const UInt32 unNumSegments = 50;
    const Real fDeltaAngle = CRadians::TWO_PI.GetValue() / unNumSegments;
    
    // Clear any existing coordinates (safety check)
    loopFunctions.CircleCoordinates.clear();
    
    // Generate all circle coordinates
    for(UInt32 i = 0; i < unNumSegments; ++i) {
        Real fAngle = fDeltaAngle * i;
        CVector3 cPoint(loopFunctions.RedCirclePosition.GetX() + fRedCircleRadius * Cos(CRadians(fAngle)),
                       loopFunctions.RedCirclePosition.GetY() + fRedCircleRadius * Sin(CRadians(fAngle)),
                       0.1f);
        loopFunctions.CircleCoordinates.push_back(cPoint);
    }
}

// Helper function to draw circle from pre-generated coordinates
void CPFA_qt_user_functions::DrawCircleFromCoordinates() {
    // Draw lines between consecutive points
    for(size_t i = 0; i < loopFunctions.CircleCoordinates.size(); ++i) {
        size_t nextIndex = (i + 1) % loopFunctions.CircleCoordinates.size();
        DrawRay(CRay3(loopFunctions.CircleCoordinates[i], 
                     loopFunctions.CircleCoordinates[nextIndex]), 
                CColor::RED, 1.0f);
    }
    
    // Draw entry point marker (north point of the circle)
    // CVector3 entryPoint(loopFunctions.RedCirclePosition.GetX(), 
    //                    loopFunctions.RedCirclePosition.GetY() + loopFunctions.RedCircleRadius,
    //                    0.1f);
    // DrawCylinder(entryPoint, CQuaternion(), 0.05, 0.05, CColor::BLUE);
}


void CPFA_qt_user_functions::DrawFood() {

	Real x, y;

	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
		x = loopFunctions.FoodList[i].GetX();
		y = loopFunctions.FoodList[i].GetY();
		DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
	}
 
	 //draw food in nests
	 for (size_t i=0; i< loopFunctions.CollectedFoodList.size(); i++)
	 { 
	        x = loopFunctions.CollectedFoodList[i].GetX();
	        y = loopFunctions.CollectedFoodList[i].GetY();
	        DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	  } 
}

void CPFA_qt_user_functions::DrawFidelity() {

	   Real x, y;
        for(map<string, CVector2>::iterator it= loopFunctions.FidelityList.begin(); it!=loopFunctions.FidelityList.end(); ++it) {
            x = it->second.GetX();
            y = it->second.GetY();
            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::YELLOW);
        }
}

void CPFA_qt_user_functions::DrawPheromones() {

	Real x, y, weight;
	vector<CVector2> trail;
	CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

	    for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
		       x = loopFunctions.PheromoneList[i].GetLocation().GetX();
		       y = loopFunctions.PheromoneList[i].GetLocation().GetY();

		       if(loopFunctions.DrawTrails == 1) {
			          trail  = loopFunctions.PheromoneList[i].GetTrail();
			          weight = loopFunctions.PheromoneList[i].GetWeight();
                

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = trailColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = trailColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = trailColor = CColor::RED;
      
             CRay3 ray;
             size_t j = 0;
      
             for(j = 1; j < trail.size(); j++) {
                 ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
		CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
                 
                 DrawRay(ray, trailColor, 1.0);
             }

	 DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
		       } 
         else {
			          weight = loopFunctions.PheromoneList[i].GetWeight();

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = CColor::RED;
      
             DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
         }
 }
}

void CPFA_qt_user_functions::DrawTargetRays() {
	//size_t tick = loopFunctions.GetSpace().GetSimulationClock();
	//size_t tock = loopFunctions.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() / 8;

	//if(tock == 0) tock = 1;

	//if(tick % tock == 0) {
		for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j], loopFunctions.TargetRayColorList[j]);
		}
	//}
}

/*
void CPFA_qt_user_functions::DrawTargetRays() {

	CColor c = CColor::BLUE;

	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j],c);
	}

	//if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond * 10) == 0) {
		// comment out for DSA, uncomment for CPFA
		loopFunctions.TargetRayList.clear();
	//}
}
*/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPFA_qt_user_functions, "CPFA_qt_user_functions")
