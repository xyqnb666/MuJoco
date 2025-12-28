// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/simple_car/simple_car.h"

#include <cmath>
#include <cstdio>
#include <string>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string SimpleCar::XmlPath() const {
  return GetModelPath("simple_car/task.xml");
}

std::string SimpleCar::Name() const { return "SimpleCar"; }

// ------- Residuals for simple_car task ------
//     Position: Car should reach goal position (x, y)
//     Control:  Controls should be small
// ------------------------------------------
void SimpleCar::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residual) const {
  // ---------- Position (x, y) ----------
  // Goal position from mocap body
  residual[0] = data->qpos[0] - data->mocap_pos[0];  // x position
  residual[1] = data->qpos[1] - data->mocap_pos[1];  // y position

  // ---------- Control ----------
  residual[2] = data->ctrl[0];  // forward control
  residual[3] = data->ctrl[1];  // turn control
}

// -------- Transition for simple_car task --------
//   If car is within tolerance of goal ->
//   move goal randomly.
// ------------------------------------------------
void SimpleCar::TransitionLocked(mjModel* model, mjData* data) {
  // Car position (x, y)
  double car_pos[2] = {data->qpos[0], data->qpos[1]};
  
  // Goal position from mocap
  double goal_pos[2] = {data->mocap_pos[0], data->mocap_pos[1]};
  
  // Distance to goal
  double car_to_goal[2];
  mju_sub(car_to_goal, goal_pos, car_pos, 2);
  
  // If within tolerance, move goal to random position
  if (mju_norm(car_to_goal, 2) < 0.2) {
    absl::BitGen gen_;
    data->mocap_pos[0] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[1] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[2] = 0.01;  // keep z at ground level
  }
}

// draw task-related geometry in the scene
void SimpleCar::ModifyScene(const mjModel* model, const mjData* data,
                             mjvScene* scene) const {
  // Get car body ID
  int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (car_body_id < 0) return;  // car body not found
  
  // Get car's linear velocity from sensor
  double* car_velocity = SensorByName(model, data, "car_velocity");
  if (!car_velocity) return;  // sensor not found
  
  // Compute speed (magnitude of velocity vector)
  double speed_ms = mju_norm3(car_velocity);
  double speed_kmh = speed_ms * 3.6;  // Convert m/s to km/h
  
  // Get car position
  double* car_pos = data->xpos + 3 * car_body_id;
  
  // Create speed label text
  char label[100];
  std::snprintf(label, sizeof(label), "Speed: %.2f m/s (%.1f km/h)", speed_ms, speed_kmh);
  
  // Add text label above the car
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_LABEL;
    geom->size[0] = geom->size[1] = geom->size[2] = 0.15;  // text size
    geom->pos[0] = car_pos[0];
    geom->pos[1] = car_pos[1];
    geom->pos[2] = car_pos[2] + 0.2;  // above the car
    geom->rgba[0] = 1.0f;  // white text
    geom->rgba[1] = 1.0f;
    geom->rgba[2] = 1.0f;
    geom->rgba[3] = 1.0f;
    std::strncpy(geom->label, label, sizeof(geom->label) - 1);
    geom->label[sizeof(geom->label) - 1] = '\0';  // ensure null termination
    scene->ngeom++;
  }
}

}  // namespace mjpc
