// filepath: d:/4th Year - University of Toronto/CSC317/CSC317-Mass-Spring-System/src/boid.cpp
#include "boid.h"

void Boid::set_target(const Eigen::Vector3d& target)
{
    // Enable target seeking toward the provided position
    target_position_ = target;
    target_active_   = true;
}

void Boid::clear_target()
{
    // Disable target seeking
    target_active_ = false;
}

Boid::Boid()
    : position(Eigen::Vector3d::Zero())
    , velocity(Eigen::Vector3d::Zero())
    , max_speed_(6.0)
    , max_force_(4.0)
    , perception_radius_(5.0)
    , separation_radius_(1.5)
{
    // Initialize with default parameters
}

Boid::Boid(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel)
    : position(pos)
    , velocity(vel)
    , max_speed_(6.0)
    , max_force_(4.0)
    , perception_radius_(5.0)
    , separation_radius_(1.5)
{
    // Initialize with custom position and velocity
}

void Boid::update(double dt, const std::vector<Boid>& flock, double alignment_weight, double cohesion_weight, double separation_weight, double target_weight)
{
    // remember to apply weights to each behavior
    // Accumulate desired acceleration from behaviors
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    acceleration += alignment_weight  * steer_alignment(flock);
    acceleration += cohesion_weight   * steer_cohesion(flock);
    acceleration += separation_weight * steer_separation(flock);
    acceleration += target_weight     * steer_target();
    acceleration = limit_force(acceleration);

    // Integrate velocity and clamp to maximum speed
    velocity += acceleration * dt;
    const double speed = velocity.norm();
    if(speed > max_speed_)
    {
        velocity *= (max_speed_ / speed);
    }

    // Integrate position
    position += velocity * dt;
}

Eigen::Vector3d Boid::steer_alignment(const std::vector<Boid>& flock) const
{
    // Compute average heading of nearby flockmates
    Eigen::Vector3d average_velocity = Eigen::Vector3d::Zero();
    int count = 0;

    for(const auto& other : flock)
    {
        if(&other == this) continue;
        const double distance = (other.position - position).norm();
        if(distance < perception_radius_)
        {
            average_velocity += other.velocity;
            ++count;
        }
    }

    // quick check to avoid division by zero
    if(count == 0) return Eigen::Vector3d::Zero();
    average_velocity /= static_cast<double>(count);

    if(average_velocity.norm() == 0.0) return Eigen::Vector3d::Zero();

    // Steer toward the average heading while respecting force limits
    Eigen::Vector3d desired = average_velocity.normalized() * max_speed_;
    return limit_force(desired - velocity);
}

Eigen::Vector3d Boid::steer_cohesion(const std::vector<Boid>& flock) const
{
    // Pull toward the average position of neighbors
    Eigen::Vector3d center_mass = Eigen::Vector3d::Zero();
    int count = 0;

    for(const auto& other : flock)
    {
        if(&other == this) continue;
        const double distance = (other.position - position).norm();
        if(distance < perception_radius_)
        {
            center_mass += other.position;
            ++count;
        }
    }

    // quick check to avoid division by zero
    if(count == 0) return Eigen::Vector3d::Zero();
    center_mass /= static_cast<double>(count);

    // Steer toward the center of mass while respecting force limits
    Eigen::Vector3d desired = (center_mass - position);
    if(desired.norm() == 0.0) return Eigen::Vector3d::Zero();
    desired = desired.normalized() * max_speed_;

    return limit_force(desired - velocity);
}

Eigen::Vector3d Boid::steer_separation(const std::vector<Boid>& flock) const
{
    // Push away from neighbors that are too close
    Eigen::Vector3d steer = Eigen::Vector3d::Zero();
    int count = 0;

    for(const auto& other : flock)
    {
        if(&other == this) continue;
        Eigen::Vector3d diff = position - other.position;
        const double distance = diff.norm();
        if(distance > 0.0 && distance < separation_radius_)
        {
            steer += diff / (distance * distance);
            ++count;
        }
    }

    if(count == 0) return Eigen::Vector3d::Zero();
    steer /= static_cast<double>(count);

    // quick check to avoid division by zero
    if(steer.norm() == 0.0) return Eigen::Vector3d::Zero();
    // Steer away while respecting force limits
    steer = steer.normalized() * max_speed_;

    return limit_force(steer - velocity);
}

Eigen::Vector3d Boid::steer_target() const
{
    // Seek toward the active target point if set
    if(!target_active_) return Eigen::Vector3d::Zero();

    Eigen::Vector3d desired = target_position_ - position;
    if(desired.norm() == 0.0) return Eigen::Vector3d::Zero();

    desired = desired.normalized() * max_speed_;
    // Steer toward the target while respecting force limits
    return limit_force(desired - velocity);
}

Eigen::Vector3d Boid::limit_force(const Eigen::Vector3d& force) const
{
    // Clamp the steering force to the maximum permitted magnitude
    const double magnitude = force.norm();
    if(magnitude > max_force_)
    {
        return force * (max_force_ / magnitude);
    }
    return force;
}