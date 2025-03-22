/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE FALSE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  TURN,
  RIGHT,
  LEFT,
  OUT_OF_BOUNDS
};

// define settings
uint8_t turn_around_wait_time = 4u;       // time to wait before turning around [s]

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t heading_setpoint = 0;
int16_t turn_around = 0;
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
static bool waypoints_set = false;
static bool out_of_bounds_handled = false;
float turning_setpoint = 5.f;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t mighty_mike, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t best_heading_angle, int16_t __attribute__((unused)) extra)
{
  heading_setpoint = best_heading_angle;
  // turn_around = mighty_mike;
  turn_around = 0;
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(4, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  float moveDistance = 1.f;

  // fprintf(stderr, "NAV_STATE: %d, Best Angle: %d degrees\n", navigation_state, heading_setpoint);
  fprintf(stderr, "Heading setpoint: %d\n", heading_setpoint);
  fprintf(stderr, "Mighty Mike: %d\n", turn_around);

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideFlightZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (turn_around){
        navigation_state = TURN;
        fprintf(stderr, "TURNAROUND\n");
      } else if (heading_setpoint > 0){
        fprintf(stderr, "RIGHT: Heading setpoint: %d\n", heading_setpoint);
        navigation_state = RIGHT;
      } else if (heading_setpoint < 0){
        fprintf(stderr, "LEFT: Heading setpoint: %d\n", heading_setpoint);
        navigation_state = LEFT;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }

      break;
    case TURN: {
      if (!waypoints_set) {
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_RETREAT);
        waypoint_move_here_2d(WP_TRAJECTORY);
        waypoints_set = true;
      }
      increase_nav_heading(turning_setpoint);

      // Check if 2 seconds have passed
      if (turn_around == 0) {
      fprintf(stderr, "TURNAROUND COMPLETE\n");
      waypoints_set = false; // reset parameter
      navigation_state = SAFE;
      }
      break;
    }
    case RIGHT:
      increase_nav_heading(heading_setpoint);
      if (heading_setpoint == 0){
        navigation_state = SAFE;
      }
      break;
    case LEFT:
      increase_nav_heading(heading_setpoint);
      if (heading_setpoint == 0){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      if (!out_of_bounds_handled) {
        // float dydx_drone = (WaypointY(WP_TRAJECTORY) - WaypointY(WP_GOAL)) / (WaypointX(WP_TRAJECTORY) - WaypointX(WP_GOAL));
        // float dydx_edge = 0.f;
        // (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        // edge one
        if ((WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ1)) * (WaypointY(WP_FZ2) - WaypointY(WP_FZ1)) - (WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ1)) * (WaypointX(WP_FZ2) - WaypointX(WP_FZ1)) <= 0) {
          fprintf(stderr, "EDGE ONE\n");
          float distance_traj_wp1 = sqrtf(powf(WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ1), 2) + powf(WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ1), 2));
          float distance_goal_wp1 = sqrtf(powf(WaypointX(WP_GOAL) - WaypointX(WP_FZ1), 2) + powf(WaypointY(WP_GOAL) - WaypointY(WP_FZ1), 2));
          if (distance_goal_wp1 < distance_traj_wp1) {
          turning_setpoint = 5.f;
          } else {
          turning_setpoint = -5.f;
          }
        }

        // edge two
        if ((WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ2)) * (WaypointY(WP_FZ3) - WaypointY(WP_FZ2)) - (WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ2)) * (WaypointX(WP_FZ3) - WaypointX(WP_FZ2)) <= 0) {
          fprintf(stderr, "EDGE TWO\n");
          float distance_traj_wp2 = sqrtf(powf(WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ2), 2) + powf(WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ2), 2));
          float distance_goal_wp2 = sqrtf(powf(WaypointX(WP_GOAL) - WaypointX(WP_FZ2), 2) + powf(WaypointY(WP_GOAL) - WaypointY(WP_FZ2), 2));
          if (distance_goal_wp2 < distance_traj_wp2) {
          turning_setpoint = 5.f;
          } else {
          turning_setpoint = -5.f;
          }
        }
        // edge three
        if ((WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ3)) * (WaypointY(WP_FZ4) - WaypointY(WP_FZ3)) - (WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ3)) * (WaypointX(WP_FZ4) - WaypointX(WP_FZ3)) <= 0) {
          fprintf(stderr, "EDGE THREE\n");
          float distance_traj_wp3 = sqrtf(powf(WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ3), 2) + powf(WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ3), 2));
          float distance_goal_wp3 = sqrtf(powf(WaypointX(WP_GOAL) - WaypointX(WP_FZ3), 2) + powf(WaypointY(WP_GOAL) - WaypointY(WP_FZ3), 2));
          if (distance_goal_wp3 < distance_traj_wp3) {
          turning_setpoint = 5.f;
          } else {
          turning_setpoint = -5.f;
          }
        }
        // edge four
        if ((WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ4)) * (WaypointY(WP_FZ1) - WaypointY(WP_FZ4)) - (WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ4)) * (WaypointX(WP_FZ1) - WaypointX(WP_FZ4)) <= 0) {
          fprintf(stderr, "EDGE FOUR\n");
          float distance_traj_wp4 = sqrtf(powf(WaypointX(WP_TRAJECTORY) - WaypointX(WP_FZ4), 2) + powf(WaypointY(WP_TRAJECTORY) - WaypointY(WP_FZ4), 2));
          float distance_goal_wp4 = sqrtf(powf(WaypointX(WP_GOAL) - WaypointX(WP_FZ4), 2) + powf(WaypointY(WP_GOAL) - WaypointY(WP_FZ4), 2));
          if (distance_goal_wp4 < distance_traj_wp4) {
          turning_setpoint = 5.f;
          } else {
          turning_setpoint = -5.f;
          }
        }
        out_of_bounds_handled = true;
      }

      increase_nav_heading(turning_setpoint);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);

      if (InsideFlightZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(turning_setpoint);
        out_of_bounds_handled = false; // reset parameter

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

