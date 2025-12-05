#pragma once

#include <Eigen/Core>
#include <vector>

// Represents a single autonomous object (boid) in "flocking" simulations.
// Note: I asked ChatGPT to help me format this header comment nicely.
//
// Static Interface:
//   set_target     Globally sets a destination point that all boids will attempt to steer toward.
//   clear_target   Disables the global target so boids resume free flocking.
//
// Constructors:
//   Boid()                              Initializes a boid at the origin with zero velocity.
//   Boid(pos, vel)                      Initializes a boid with the specified position and velocity.
//
// Simulation Update:
//   update(dt, flock)                   Advances the boid’s state by dt seconds ( based on neighbours - can set weights ).
//
// Public State:
//   position                            Current spatial location of the boid.
//   velocity                            Current velocity vector of the boid.
//
// Internal Parameters (You can play with these):
//   max_speed_                          Upper bound on the boid’s speed magnitude (to keep things realistic, see textbook).
//   max_force_                          Upper bound on steering force applied per update (to keep things realistic, see textbook).
//   perception_radius_                  Distance within which other boids influence alignment and cohesion.
//   separation_radius_                  Distance defining the personal space enforced by separation steering.
//   target_position_                    Shared target location when target steering is active.
//   target_active_                      Flag indicating whether target steering should be applied.
//
// Steering Helpers (return steering forces):
//   steer_alignment(flock)              Push to match average heading of nearby boids.
//   steer_cohesion(flock)               Pulls the boid toward the centroid of nearby flockmates.
//   steer_separation(flock)             Pushes the boid away from neighbors that are too close.
//   steer_target()                      Directs the boid toward the global target when active.
//   limit_force(force)                  Clamps a steering force to respect max_force_ constraints.

class Boid final
{
public:
    static void set_target(const Eigen::Vector3d& target);
    static void clear_target();

    Boid();
    Boid(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);

    void update(double dt, const std::vector<Boid>& flock, double alignment_weight = 1.0, double cohesion_weight = 1.0, double separation_weight = 1.0, double target_weight = 1.0);

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

private:
    double max_speed_;
    double max_force_;
    double perception_radius_;
    double separation_radius_;
    inline static Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();
    inline static bool target_active_ = false;

    Eigen::Vector3d steer_alignment(const std::vector<Boid>& flock) const;
    Eigen::Vector3d steer_cohesion(const std::vector<Boid>& flock) const;
    Eigen::Vector3d steer_separation(const std::vector<Boid>& flock) const;
    Eigen::Vector3d steer_target() const;
    Eigen::Vector3d limit_force(const Eigen::Vector3d& force) const;
};
