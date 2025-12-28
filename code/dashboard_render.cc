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

#include "mjpc/dashboard_render.h"
#include <cstdio>
#include <cmath>
#include <string>

namespace mujoco {

// Dashboard implementation
Dashboard::Dashboard() {
  // Initialize dashboard data
  data_.speed = 0.0;
  data_.rpm = 0.0;
  data_.fuel_level = 100.0;

  mjr_context_ = nullptr;
  
  // Initialize display elements to null
  speedometer_ = nullptr;
  tachometer_ = nullptr;
  fuel_gauge_ = nullptr;
}

Dashboard::~Dashboard() {
  // Delete display elements
  delete speedometer_;
  delete tachometer_;
  delete fuel_gauge_;
}

void Dashboard::Initialize() {
  // Display elements are initialized in Render when dimensions are known
}

void Dashboard::UpdateData(const mjModel* model, const mjData* data) {
  // Calculate speed from velocity (m/s to km/h)
  double velocity[3] = {data->qvel[0], data->qvel[1], data->qvel[2]};
  double speed_ms = mju_norm3(velocity);
  data_.speed = speed_ms * 3.6;

  // Estimate RPM based on speed (simplified model)
  // Assume RPM is proportional to speed, with a scaling factor
  double rpm_scaling = 100.0; // Adjust this factor based on your car model
  data_.rpm = data_.speed * rpm_scaling;

  // Limit RPM to reasonable range
  if (data_.rpm > 8000.0) {
    data_.rpm = 8000.0;
  } else if (data_.rpm < 0.0) {
    data_.rpm = 0.0;
  }

  // Simulate fuel consumption (slowly decrease fuel level)
  data_.fuel_level -= 0.001; // Adjust this rate based on your preference
  if (data_.fuel_level < 0.0) {
    data_.fuel_level = 0.0;
  }
  
  // Update display elements
  if (speedometer_) speedometer_->update(data_.speed);
  if (tachometer_) tachometer_->update(data_.rpm);
  if (fuel_gauge_) fuel_gauge_->update(data_.fuel_level);
}

// Helper function to draw an arc using small rectangles
static void DrawArc(double center_x, double center_y, double radius, double start_angle, double end_angle, 
                    float r, float g, float b, float a, const mjrContext* mjr_context) {
  const int segments = 100; // Number of segments to approximate the arc
  const double angle_step = (end_angle - start_angle) / segments;
  const double segment_length = (radius * 0.05); // Width of each segment
  
  for (int i = 0; i < segments; ++i) {
    double angle = start_angle + (i * angle_step);
    double x = center_x + (radius * cos(angle));
    double y = center_y + (radius * sin(angle));
    
    // Draw a small rectangle as a segment of the arc
    mjrRect segment;
    segment.left = static_cast<int>(x - segment_length / 2);
    segment.bottom = static_cast<int>(y - segment_length / 2);
    segment.width = static_cast<int>(segment_length);
    segment.height = static_cast<int>(segment_length);
    mjr_rectangle(segment, r, g, b, a);
  }
}

// Helper function to draw a circle
static void DrawCircle(double center_x, double center_y, double radius, 
                       float r, float g, float b, float a, const mjrContext* mjr_context) {
  const int segments = 100;
  const double angle_step = (2 * M_PI) / segments;
  const double segment_length = (radius * 0.05);
  
  for (int i = 0; i < segments; ++i) {
    double angle = i * angle_step;
    double x = center_x + (radius * cos(angle));
    double y = center_y + (radius * sin(angle));
    
    mjrRect segment;
    segment.left = static_cast<int>(x - segment_length / 2);
    segment.bottom = static_cast<int>(y - segment_length / 2);
    segment.width = static_cast<int>(segment_length);
    segment.height = static_cast<int>(segment_length);
    mjr_rectangle(segment, r, g, b, a);
  }
}

// Helper function to draw a filled circle
static void DrawFilledCircle(double center_x, double center_y, double radius, 
                             float r, float g, float b, float a, const mjrContext* mjr_context) {
  // Draw concentric circles from outer to inner
  const int rings = static_cast<int>(radius / 2);
  for (int i = 0; i < rings; ++i) {
    double ring_radius = radius - (i * 2);
    if (ring_radius < 0) break;
    DrawCircle(center_x, center_y, ring_radius, r, g, b, a, mjr_context);
  }
}

void Dashboard::Render(const mjModel* model, const mjData* data, 
                      const mjrContext* mjr_context, const mjrRect* rect) {
  // Update dashboard data
  UpdateData(model, data);

  // Store rendering context
  mjr_context_ = mjr_context;

  // Draw dashboard elements
  double width = rect->width;
  double height = rect->height;

  // Note: Dashboard position calculation based on car position is temporarily disabled
  // This will be re-implemented once we have proper access to the actual camera used in rendering
  // The dashboard will currently be displayed at the center of the screen
  
  // // Get dashboard position from dashboard_marker site
  // // Find dashboard_marker site ID by name
  // int dashboard_site_id = mj_name2id(model, mjOBJ_SITE, "dashboard_marker");
  // double car_pos[3] = {0.0, 0.0, 0.0};
  // 
  // if (dashboard_site_id >= 0) {
  //   // Calculate index in xpos array (each site has 3 coordinates: x, y, z)
  //   int dashboard_pos_index = 3 * dashboard_site_id;
  //   car_pos[0] = data->xpos[dashboard_pos_index];
  //   car_pos[1] = data->xpos[dashboard_pos_index + 1];
  //   car_pos[2] = data->xpos[dashboard_pos_index + 2];
  // } else {
  //   // Fallback to car body position if dashboard_marker not found
  //   int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  //   if (car_body_id >= 0) {
  //     int car_pos_index = 3 * car_body_id;
  //     car_pos[0] = data->xpos[car_pos_index];
  //     car_pos[1] = data->xpos[car_pos_index + 1];
  //     car_pos[2] = data->xpos[car_pos_index + 2];
  //   }
  // }

  // Dashboard center is at the screen center (fallback implementation)
  // This will be replaced with proper world-to-screen conversion using the actual camera
  double center_x = width / 2.0;
  double center_y = height / 2.0;
  
  // Note: In a real implementation with proper camera access, you would use:
  // 1. Get the actual camera from the scene (scn.camera)
  // 2. Use mjr_coordinate to convert world coordinates to screen coordinates
  // 3. Position the dashboard based on the car's screen position
  // For now, we'll use a simplified approach that doesn't require camera parameters
  
  // Adjust for better visibility (move slightly above the car)
  center_y -= 100; // Move up 100 pixels
  
  // Speedometer (large circle in the middle)
  double speedometer_radius = 150.0;
  double speedometer_x = center_x;
  double speedometer_y = center_y;
  
  // Tachometer (small circle on the left)
  double small_gauge_radius = 80.0;
  double tachometer_x = center_x - speedometer_radius - small_gauge_radius - 20.0;
  double tachometer_y = center_y;
  
  // Fuel gauge (small circle on the right)
  double fuel_gauge_x = center_x + speedometer_radius + small_gauge_radius + 20.0;
  double fuel_gauge_y = center_y;
  
  // Create display elements if they don't exist
  if (!speedometer_) {
    speedometer_ = new Speedometer(speedometer_x, speedometer_y, speedometer_radius, speedometer_radius, mjr_context);
    tachometer_ = new Tachometer(tachometer_x, tachometer_y, small_gauge_radius, small_gauge_radius, mjr_context);
    fuel_gauge_ = new FuelGauge(fuel_gauge_x, fuel_gauge_y, small_gauge_radius, small_gauge_radius, mjr_context);
  } else {
    // Update positions of existing elements
    speedometer_->UpdatePosition(speedometer_x, speedometer_y);
    tachometer_->UpdatePosition(tachometer_x, tachometer_y);
    fuel_gauge_->UpdatePosition(fuel_gauge_x, fuel_gauge_y);
  }
  
  // Render display elements
  speedometer_->render();
  tachometer_->render();
  fuel_gauge_->render();
}

// Speedometer implementation
void Speedometer::render() {
  if (!mjr_context_) return;

  // Speed limits
  const double min_speed = 0.0;
  const double max_speed = 240.0;

  // Clamp speed
  double clamped_speed = speed_;
  if (clamped_speed > max_speed) {
    clamped_speed = max_speed;
  } else if (clamped_speed < min_speed) {
    clamped_speed = min_speed;
  }

  // Calculate center coordinates
  double center_x = x_;
  double center_y = y_;
  double radius = width_; // width_ is now the radius

  // Draw background circle
  DrawFilledCircle(center_x, center_y, radius, 0.1f, 0.1f, 0.1f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius, 0.8f, 0.8f, 0.8f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius - 5.0, 0.2f, 0.2f, 0.2f, 1.0f, mjr_context_);

  // Draw speed scale (semicircle from -π/2 to π/2)
  const double start_angle = -M_PI / 2;
  const double end_angle = M_PI / 2;
  const int num_marks = 13; // 0-240 km/h in 20 km/h increments
  
  for (int i = 0; i < num_marks; ++i) {
    double angle = start_angle + (end_angle - start_angle) * i / (num_marks - 1);
    double mark_radius = radius - 10.0;
    double mark_x = center_x + mark_radius * cos(angle);
    double mark_y = center_y + mark_radius * sin(angle);
    
    // Draw scale mark
    mjrRect mark_rect;
    mark_rect.left = static_cast<int>(mark_x - 1.0);
    mark_rect.bottom = static_cast<int>(mark_y - 1.0);
    mark_rect.width = 3;
    mark_rect.height = 3;
    mjr_rectangle(mark_rect, 1.0f, 1.0f, 1.0f, 1.0f);
    
    // Draw label
    if (i % 2 == 0) { // Every 40 km/h
      char speed_label[10];
      int speed_value = static_cast<int>(min_speed + (max_speed - min_speed) * i / (num_marks - 1));
      std::snprintf(speed_label, sizeof(speed_label), "%d", speed_value);
      
      // Position label slightly outside the mark
      double label_radius = radius - 30.0;
      double label_x = center_x + label_radius * cos(angle);
      double label_y = center_y + label_radius * sin(angle);
      mjr_text(0, speed_label, mjr_context_, static_cast<float>(label_x), static_cast<float>(label_y), 1.0f, 1.0f, 1.0f);
    }
  }

  // Draw colored arc for current speed
  double speed_angle = start_angle + (end_angle - start_angle) * (clamped_speed / max_speed);
  
  // Green segment (0-80 km/h)
  if (clamped_speed < 80.0) {
    DrawArc(center_x, center_y, radius - 20.0, start_angle, speed_angle, 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
  } else {
    DrawArc(center_x, center_y, radius - 20.0, start_angle, start_angle + (end_angle - start_angle) * (80.0 / max_speed), 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    
    // Yellow segment (80-160 km/h)
    if (clamped_speed < 160.0) {
      DrawArc(center_x, center_y, radius - 20.0, start_angle + (end_angle - start_angle) * (80.0 / max_speed), speed_angle, 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    } else {
      DrawArc(center_x, center_y, radius - 20.0, start_angle + (end_angle - start_angle) * (80.0 / max_speed), start_angle + (end_angle - start_angle) * (160.0 / max_speed), 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
      
      // Red segment (160+ km/h)
      DrawArc(center_x, center_y, radius - 20.0, start_angle + (end_angle - start_angle) * (160.0 / max_speed), speed_angle, 1.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
    }
  }

  // Draw needle
  double needle_length = radius - 40.0;
  double needle_x = center_x + needle_length * cos(speed_angle);
  double needle_y = center_y + needle_length * sin(speed_angle);
  
  // Draw needle as a series of small rectangles
  const int num_needle_segments = 20;
  for (int i = 0; i < num_needle_segments; ++i) {
    double t = static_cast<double>(i) / (num_needle_segments - 1);
    double segment_x = center_x + t * (needle_x - center_x);
    double segment_y = center_y + t * (needle_y - center_y);
    
    mjrRect segment_rect;
    segment_rect.left = static_cast<int>(segment_x - 2.0);
    segment_rect.bottom = static_cast<int>(segment_y - 2.0);
    segment_rect.width = 5;
    segment_rect.height = 5;
    mjr_rectangle(segment_rect, 1.0f, 0.0f, 0.0f, 1.0f);
  }

  // Draw center circle
  DrawFilledCircle(center_x, center_y, 15.0, 0.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, 15.0, 1.0f, 1.0f, 1.0f, 1.0f, mjr_context_);

  // Display current speed in center
  char speed_text[10];
  std::snprintf(speed_text, sizeof(speed_text), "%.0f", clamped_speed);
  mjr_text(0, speed_text, mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y + 5.0), 1.0f, 1.0f, 1.0f);
  
  // Display "km/h" text
  mjr_text(0, "km/h", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - 15.0), 0.8f, 0.8f, 0.8f);

  // Display gauge name
  mjr_text(0, "SPEED", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - radius - 10.0), 1.0f, 1.0f, 1.0f);
}

void Speedometer::update(double speed) {
  speed_ = speed;
}

// Tachometer implementation
void Tachometer::render() {
  if (!mjr_context_) return;

  // RPM limits
  const double min_rpm = 0.0;
  const double max_rpm = 8000.0;

  // Clamp RPM
  double clamped_rpm = rpm_;
  if (clamped_rpm > max_rpm) {
    clamped_rpm = max_rpm;
  } else if (clamped_rpm < min_rpm) {
    clamped_rpm = min_rpm;
  }

  // Calculate center coordinates
  double center_x = x_;
  double center_y = y_;
  double radius = width_; // width_ is now the radius

  // Draw background circle
  DrawFilledCircle(center_x, center_y, radius, 0.1f, 0.1f, 0.1f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius, 0.8f, 0.8f, 0.8f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius - 5.0, 0.2f, 0.2f, 0.2f, 1.0f, mjr_context_);

  // Draw RPM scale (semicircle from -π/2 to π/2)
  const double start_angle = -M_PI / 2;
  const double end_angle = M_PI / 2;
  const int num_marks = 9; // 0-8000 RPM in 1000 RPM increments
  
  for (int i = 0; i < num_marks; ++i) {
    double angle = start_angle + (end_angle - start_angle) * i / (num_marks - 1);
    double mark_radius = radius - 10.0;
    double mark_x = center_x + mark_radius * cos(angle);
    double mark_y = center_y + mark_radius * sin(angle);
    
    // Draw scale mark
    mjrRect mark_rect;
    mark_rect.left = static_cast<int>(mark_x - 1.0);
    mark_rect.bottom = static_cast<int>(mark_y - 1.0);
    mark_rect.width = 3;
    mark_rect.height = 3;
    mjr_rectangle(mark_rect, 1.0f, 1.0f, 1.0f, 1.0f);
    
    // Draw label
    char rpm_label[10];
    int rpm_value = static_cast<int>(min_rpm + (max_rpm - min_rpm) * i / (num_marks - 1));
    std::snprintf(rpm_label, sizeof(rpm_label), "%d", rpm_value / 1000);
    
    // Position label slightly outside the mark
    double label_radius = radius - 25.0;
    double label_x = center_x + label_radius * cos(angle);
    double label_y = center_y + label_radius * sin(angle);
    mjr_text(0, rpm_label, mjr_context_, static_cast<float>(label_x), static_cast<float>(label_y), 1.0f, 1.0f, 1.0f);
  }

  // Draw colored arc for current RPM
  double rpm_angle = start_angle + (end_angle - start_angle) * (clamped_rpm / max_rpm);
  
  // Green segment (0-4000 RPM)
  if (clamped_rpm < 4000.0) {
    DrawArc(center_x, center_y, radius - 15.0, start_angle, rpm_angle, 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
  } else {
    DrawArc(center_x, center_y, radius - 15.0, start_angle, start_angle + (end_angle - start_angle) * (4000.0 / max_rpm), 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    
    // Yellow segment (4000-6000 RPM)
    if (clamped_rpm < 6000.0) {
      DrawArc(center_x, center_y, radius - 15.0, start_angle + (end_angle - start_angle) * (4000.0 / max_rpm), rpm_angle, 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    } else {
      DrawArc(center_x, center_y, radius - 15.0, start_angle + (end_angle - start_angle) * (4000.0 / max_rpm), start_angle + (end_angle - start_angle) * (6000.0 / max_rpm), 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
      
      // Red segment (6000+ RPM)
      DrawArc(center_x, center_y, radius - 15.0, start_angle + (end_angle - start_angle) * (6000.0 / max_rpm), rpm_angle, 1.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
    }
  }

  // Draw needle
  double needle_length = radius - 30.0;
  double needle_x = center_x + needle_length * cos(rpm_angle);
  double needle_y = center_y + needle_length * sin(rpm_angle);
  
  // Draw needle as a series of small rectangles
  const int num_needle_segments = 15;
  for (int i = 0; i < num_needle_segments; ++i) {
    double t = static_cast<double>(i) / (num_needle_segments - 1);
    double segment_x = center_x + t * (needle_x - center_x);
    double segment_y = center_y + t * (needle_y - center_y);
    
    mjrRect segment_rect;
    segment_rect.left = static_cast<int>(segment_x - 1.5);
    segment_rect.bottom = static_cast<int>(segment_y - 1.5);
    segment_rect.width = 4;
    segment_rect.height = 4;
    mjr_rectangle(segment_rect, 1.0f, 0.0f, 0.0f, 1.0f);
  }

  // Draw center circle
  DrawFilledCircle(center_x, center_y, 10.0, 0.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, 10.0, 1.0f, 1.0f, 1.0f, 1.0f, mjr_context_);

  // Display current RPM in center
  char rpm_text[10];
  std::snprintf(rpm_text, sizeof(rpm_text), "%.0f", clamped_rpm / 1000);
  mjr_text(0, rpm_text, mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y + 5.0), 1.0f, 1.0f, 1.0f);
  
  // Display "RPM" text
  mjr_text(0, "RPM", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - 15.0), 0.8f, 0.8f, 0.8f);

  // Display gauge name
  mjr_text(0, "TACH", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - radius - 10.0), 1.0f, 1.0f, 1.0f);
}

void Tachometer::update(double rpm) {
  rpm_ = rpm;
}

// FuelGauge implementation
void FuelGauge::render() {
  if (!mjr_context_) return;

  // Clamp fuel level
  double clamped_fuel = fuel_level_;
  if (clamped_fuel > 100.0) {
    clamped_fuel = 100.0;
  } else if (clamped_fuel < 0.0) {
    clamped_fuel = 0.0;
  }

  // Calculate center coordinates
  double center_x = x_;
  double center_y = y_;
  double radius = width_; // width_ is now the radius

  // Draw background circle
  DrawFilledCircle(center_x, center_y, radius, 0.1f, 0.1f, 0.1f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius, 0.8f, 0.8f, 0.8f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, radius - 5.0, 0.2f, 0.2f, 0.2f, 1.0f, mjr_context_);

  // Draw fuel level scale (semicircle from π/2 to -π/2, inverted for fuel gauge)
  const double start_angle = M_PI / 2;
  const double end_angle = -M_PI / 2;
  const int num_marks = 11; // 0-100% in 10% increments
  
  for (int i = 0; i < num_marks; ++i) {
    double angle = start_angle + (end_angle - start_angle) * i / (num_marks - 1);
    double mark_radius = radius - 10.0;
    double mark_x = center_x + mark_radius * cos(angle);
    double mark_y = center_y + mark_radius * sin(angle);
    
    // Draw scale mark
    mjrRect mark_rect;
    mark_rect.left = static_cast<int>(mark_x - 1.0);
    mark_rect.bottom = static_cast<int>(mark_y - 1.0);
    mark_rect.width = 3;
    mark_rect.height = 3;
    mjr_rectangle(mark_rect, 1.0f, 1.0f, 1.0f, 1.0f);
    
    // Draw label
    if (i % 2 == 0) { // Every 20%
      char fuel_label[10];
      int fuel_value = static_cast<int>(100.0 - (clamped_fuel / 100.0) * i * 10.0);
      std::snprintf(fuel_label, sizeof(fuel_label), "%d", fuel_value);
      
      // Position label slightly outside the mark
      double label_radius = radius - 25.0;
      double label_x = center_x + label_radius * cos(angle);
      double label_y = center_y + label_radius * sin(angle);
      mjr_text(0, fuel_label, mjr_context_, static_cast<float>(label_x), static_cast<float>(label_y), 1.0f, 1.0f, 1.0f);
    }
  }

  // Draw colored arc for current fuel level
  double fuel_angle = start_angle + (end_angle - start_angle) * (1.0 - clamped_fuel / 100.0);
  
  // Green segment (75-100%)
  if (clamped_fuel > 75.0) {
    DrawArc(center_x, center_y, radius - 15.0, fuel_angle, end_angle, 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
  } else if (clamped_fuel > 25.0) {
    // Yellow segment (25-75%)
    double mid_angle = start_angle + (end_angle - start_angle) * 0.25;
    DrawArc(center_x, center_y, radius - 15.0, fuel_angle, mid_angle, 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    DrawArc(center_x, center_y, radius - 15.0, mid_angle, end_angle, 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
  } else {
    // Red segment (0-25%)
    double mid_angle = start_angle + (end_angle - start_angle) * 0.25;
    double low_angle = start_angle + (end_angle - start_angle) * 0.75;
    DrawArc(center_x, center_y, radius - 15.0, fuel_angle, low_angle, 1.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
    DrawArc(center_x, center_y, radius - 15.0, low_angle, mid_angle, 1.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
    DrawArc(center_x, center_y, radius - 15.0, mid_angle, end_angle, 0.0f, 1.0f, 0.0f, 1.0f, mjr_context_);
  }

  // Draw needle
  double needle_length = radius - 30.0;
  double needle_x = center_x + needle_length * cos(fuel_angle);
  double needle_y = center_y + needle_length * sin(fuel_angle);
  
  // Draw needle as a series of small rectangles
  const int num_needle_segments = 15;
  for (int i = 0; i < num_needle_segments; ++i) {
    double t = static_cast<double>(i) / (num_needle_segments - 1);
    double segment_x = center_x + t * (needle_x - center_x);
    double segment_y = center_y + t * (needle_y - center_y);
    
    mjrRect segment_rect;
    segment_rect.left = static_cast<int>(segment_x - 1.5);
    segment_rect.bottom = static_cast<int>(segment_y - 1.5);
    segment_rect.width = 4;
    segment_rect.height = 4;
    mjr_rectangle(segment_rect, 1.0f, 0.0f, 0.0f, 1.0f);
  }

  // Draw center circle
  DrawFilledCircle(center_x, center_y, 10.0, 0.0f, 0.0f, 0.0f, 1.0f, mjr_context_);
  DrawCircle(center_x, center_y, 10.0, 1.0f, 1.0f, 1.0f, 1.0f, mjr_context_);

  // Display current fuel level in center
  char fuel_text[10];
  std::snprintf(fuel_text, sizeof(fuel_text), "%.0f", clamped_fuel);
  mjr_text(0, fuel_text, mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y + 5.0), 1.0f, 1.0f, 1.0f);
  
  // Display "%" text
  mjr_text(0, "%", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - 15.0), 0.8f, 0.8f, 0.8f);

  // Display gauge name
  mjr_text(0, "FUEL", mjr_context_, static_cast<float>(center_x), static_cast<float>(center_y - radius - 10.0), 1.0f, 1.0f, 1.0f);
}

void FuelGauge::update(double fuel_level) {
  fuel_level_ = fuel_level;
}

}  // namespace mujoco
