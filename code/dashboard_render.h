// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MJPC_DASHBOARD_RENDER_H_
#define MJPC_DASHBOARD_RENDER_H_

#include <mujoco/mujoco.h>
#include "mjpc/dashboard_data.h"

namespace mujoco {

// Base class for display elements
class DisplayElement {
 public:
  DisplayElement(double x, double y, double width, double height, const mjrContext* context)
      : x_(x), y_(y), width_(width), height_(height), mjr_context_(context) {}
  virtual ~DisplayElement() = default;
  
  // Pure virtual methods
  virtual void render() = 0;
  virtual void update(double value) = 0;
  
  // Update element position
  void UpdatePosition(double x, double y) {
    x_ = x;
    y_ = y;
  }
  
 protected:
  double x_, y_, width_, height_;
  const mjrContext* mjr_context_;
};

// Speedometer class
class Speedometer : public DisplayElement {
 public:
  Speedometer(double x, double y, double width, double height, const mjrContext* context)
      : DisplayElement(x, y, width, height, context), speed_(0.0) {}
  
  void render() override;
  void update(double speed) override;
  
 private:
  double speed_;
};

// Tachometer class
class Tachometer : public DisplayElement {
 public:
  Tachometer(double x, double y, double width, double height, const mjrContext* context)
      : DisplayElement(x, y, width, height, context), rpm_(0.0) {}
  
  void render() override;
  void update(double rpm) override;
  
 private:
  double rpm_;
};

// FuelGauge class
class FuelGauge : public DisplayElement {
 public:
  FuelGauge(double x, double y, double width, double height, const mjrContext* context)
      : DisplayElement(x, y, width, height, context), fuel_level_(0.0) {}
  
  void render() override;
  void update(double fuel_level) override;
  
 private:
  double fuel_level_;
};

// Dashboard class for rendering car dashboard
class Dashboard {
 public:
  Dashboard();
  ~Dashboard();

  // Initialize dashboard
  void Initialize();

  // Render dashboard
  void Render(const mjModel* model, const mjData* data, 
              const mjrContext* mjr_context, const mjrRect* rect);

  // Update dashboard data
  void UpdateData(const mjModel* model, const mjData* data);

 private:
  // Dashboard data
  DashboardData data_;

  // Rendering context
  const mjrContext* mjr_context_;
  
  // Display elements (composition)
  Speedometer* speedometer_;
  Tachometer* tachometer_;
  FuelGauge* fuel_gauge_;
};

}  // namespace mujoco

#endif  // MJPC_DASHBOARD_RENDER_H_
