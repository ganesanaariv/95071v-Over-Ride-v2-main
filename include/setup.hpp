#pragma once

#include "mechanisms/drive.hpp"
#include "mechanisms/cascade.hpp"
#include "mechanisms/claw.hpp"

const int FL_PORT = 15;
const int ML_PORT = 14;
const int BL_PORT = 13;
const int FR_PORT = 17;
const int MR_PORT = 19;
const int BR_PORT = 18;

const int IMU1_PORT = 16;
const int DODOMF_PORT = 2;
const int DODOMB_PORT = 3;
const int DODOMR_PORT = 1;
const int DODOML_PORT = 10;

const int CASCADE_PORT = 11;
const int CASCADE_PORT2 = -20;
const char CLAW_PISTON_PORT = 'A';
const int DISTANCE_PORT = 12;

Drivetrain drive({FL_PORT, ML_PORT, BL_PORT}, {-FR_PORT, -MR_PORT, -BR_PORT}, IMU1_PORT,
                 {DODOMF_PORT, DODOMB_PORT, DODOMR_PORT, DODOML_PORT});
Cascade cascade(CASCADE_PORT, CASCADE_PORT2);
Claw claw(CLAW_PISTON_PORT, DISTANCE_PORT);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
