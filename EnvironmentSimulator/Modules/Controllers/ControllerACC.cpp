/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

/*
 * This controller simulates a simple Adaptive Cruise Control
 */

#include "ControllerACC.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "IdealSensor.hpp"
#include "playerbase.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerACC(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerACC(initArgs);
}

ControllerACC::ControllerACC(InitArgs* args)
    : Controller(args),
    sensor_(nullptr),
      active_(false),
      timeGap_(1.5),
      setSpeed_(0),
      lateralDist_(5.0),
      currentSpeed_(0),
      setSpeedSet_(false),
    virtual_(false),
    perception_noisy_(false),
    sensor_x_(4.0),
    sensor_y_(0.0),
    sensor_z_(0.5),
    sensor_heading_(0.0),
    sensor_near_range_(1.0),
    sensor_far_range_(80.0),
    sensor_fov_deg_(70.0),
    sensor_max_objects_(100),
    show_sensor_frustum_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG);

    if (args && args->properties && args->properties->ValueExists("timeGap"))
    {
        timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
    }
    if (args && args->properties && args->properties->ValueExists("setSpeed"))
    {
        setSpeed_    = strtod(args->properties->GetValueStr("setSpeed"));
        setSpeedSet_ = true;
    }
    if (args && args->properties && args->properties->ValueExists("lateralDist"))
    {
        lateralDist_ = strtod(args->properties->GetValueStr("lateralDist"));
    }
    if (args && args->properties && !args->properties->ValueExists("mode"))
    {
        // Default mode for this controller is additive
        // which will use speed set by other actions as setSpeed
        // in override mode setSpeed is set explicitly (if missing
        // the current speed when controller is activated will be
        // used as setSpeed)
        mode_ = ControlOperationMode::MODE_ADDITIVE;
    }
    if (args && args->properties && args->properties->ValueExists("virtual"))
    {
        virtual_ = args->properties->GetValueStr("virtual") == "true" ? true : false;
    }
    if (args && args->properties && args->properties->ValueExists("perceptionMode"))
    {
        perception_noisy_ = args->properties->GetValueStr("perceptionMode") == "noisy";
    }
    if (args && args->properties)
    {
        if (args->properties->ValueExists("sensorX"))
            sensor_x_ = strtod(args->properties->GetValueStr("sensorX"));
        if (args->properties->ValueExists("sensorY"))
            sensor_y_ = strtod(args->properties->GetValueStr("sensorY"));
        if (args->properties->ValueExists("sensorZ"))
            sensor_z_ = strtod(args->properties->GetValueStr("sensorZ"));
        if (args->properties->ValueExists("sensorHeading"))
            sensor_heading_ = strtod(args->properties->GetValueStr("sensorHeading"));
        if (args->properties->ValueExists("sensorNearRange"))
            sensor_near_range_ = strtod(args->properties->GetValueStr("sensorNearRange"));
        if (args->properties->ValueExists("sensorFarRange"))
            sensor_far_range_ = strtod(args->properties->GetValueStr("sensorFarRange"));
        if (args->properties->ValueExists("sensorFovDeg"))
            sensor_fov_deg_ = strtod(args->properties->GetValueStr("sensorFovDeg"));
        if (args->properties->ValueExists("sensorMaxObjects"))
            sensor_max_objects_ = static_cast<int>(strtol(args->properties->GetValueStr("sensorMaxObjects"), nullptr, 10));
        if (args->properties->ValueExists("showSensorFrustum"))
            show_sensor_frustum_ = args->properties->GetValueStr("showSensorFrustum") == "true";
    }
    if (args && args->properties && args->properties->ValueExists("logFile"))
    {
        log_file_ = args->properties->GetValueStr("logFile");
    }
}

void ControllerACC::Init()
{
    Controller::Init();
}

void ControllerACC::InitPostPlayer()
{
    sensor_ = player_->GetObjectSensor(object_);
    if (sensor_ == nullptr)
    {
        player_->AddObjectSensor(
            object_,
            sensor_x_,
            sensor_y_,
            sensor_z_,
            sensor_heading_,
            sensor_near_range_,
            sensor_far_range_,
            sensor_fov_deg_ * M_PI / 180.0,
            sensor_max_objects_);
        sensor_ = player_->GetObjectSensor(object_);
    }

    if (perception_noisy_)
    {
        sensor_->SetPerceptionMode(PerceptionModel::Mode::NOISY);
    }

    if (show_sensor_frustum_)
    {
        player_->ShowObjectSensors(true);
    }

    if (!log_file_.empty())
    {
        log_stream_.open(log_file_, std::ios::out | std::ios::trunc);
        if (log_stream_.is_open())
        {
            log_stream_ << "time,perception_mode,target_id,perceived_distance,"
                            "true_distance,distance_error,ego_speed,target_speed,"
                            "commanded_speed,collision\n";
        }
    }
}

void ControllerACC::Step(double timeStep)
{
    double minGapLength = LARGE_NUMBER;
    // double minSpeedDiff = 0.0; // TODO: Commented out because it is not used
    const double minDist            = 3.0;  // minimum distance to keep to lead vehicle
    const double accelerationFactor = 0.7;
    double selectedPerceivedDistance = -1.0;
    double selectedTrueDistance      = -1.0;
    double selectedDistanceError     = 0.0;
    Object* selectedObject           = nullptr;

    // First check if speed has been set from somewhere else (another action or controller), respect it and update setSpeed
    if (virtual_)
    {
        currentSpeed_ = object_->GetSpeed();
    }
    else if (
        // mode_ == ControlOperationMode::MODE_ADDITIVE &&
        abs(object_->GetSpeed() - currentSpeed_) > 1e-3)
    {
        LOG_INFO("New setspeed: {:5.2f} -> {:5.2f}", setSpeed_, object_->GetSpeed());
        setSpeed_ = object_->GetSpeed();
    }

    // Lookahead distance is at least 50m or twice the distance required to stop
    // https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
    double lookaheadDist = MAX(50.0, 2 * minDist - pow(currentSpeed_, 2) / -object_->GetMaxDeceleration());  // (m)
    Object* minObj = nullptr;
    if (sensor_ != nullptr)
    {
        for (int i = 0; i < sensor_->GetNumberOfHits(); ++i)
        {
            const ObjectSensor::ObjectHit& hit = sensor_->GetHit(i);
            Object* pivot_obj = hit.obj_;
            if (pivot_obj == nullptr || pivot_obj == object_ || pivot_obj->GetType() != Object::Type::VEHICLE)
            {
                continue;
            }

            if (hit.x_ > 0.0 && abs(hit.y_) < lateralDist_ && hit.perceived_distance_ < lookaheadDist &&
                hit.perceived_distance_ > 0.0 &&
                (minObj == nullptr || hit.perceived_distance_ < minGapLength))
            {
                minGapLength = hit.perceived_distance_;
                minObj = pivot_obj;
                selectedObject = pivot_obj;
                selectedPerceivedDistance = hit.perceived_distance_;
                selectedTrueDistance = hit.true_distance_;
                selectedDistanceError = hit.distance_error_;
            }
        }
    }

    double acc = 0.0;
    if (minObj != nullptr)
    {
        if (minGapLength < 1)
        {
            currentSpeed_ = 0.0;
        }
        else
        {
            // Follow distance = minimum distance + timeGap_ seconds
            double speedForTimeGap = MAX(currentSpeed_, minObj->GetSpeed());
            double followDist      = minDist + timeGap_ * fabs(speedForTimeGap);  // (m)
            double dist            = minGapLength - followDist;
            double distFactor      = MIN(1.0, dist / followDist);

            double dvMin = currentSpeed_ - MIN(setSpeed_, minObj->GetSpeed());
            double dvSet = currentSpeed_ - setSpeed_;

            acc = 2.5 * distFactor - distFactor * dvSet - (1 - distFactor) * dvMin;  // weighted combination of relative distance and speed
            acc = CLAMP(acc, -object_->GetMaxDeceleration(), object_->GetMaxAcceleration());

            currentSpeed_ += acc * timeStep;

            // ensure positiove speed and not exceeding setSpeed
            currentSpeed_ = MIN(MAX(0.0, currentSpeed_), setSpeed_);
        }

        object_->SetLookaheadSensorPosition(minObj->pos_.GetX(), minObj->pos_.GetY(), minObj->pos_.GetZ());
    }
    else
    {
        // no lead vehicle to adapt to, adjust according to setSpeed
        acc             = (setSpeed_ - currentSpeed_) * accelerationFactor * object_->GetMaxAcceleration();
        acc             = CLAMP(acc, -object_->GetMaxDeceleration(), accelerationFactor * object_->GetMaxAcceleration());
        double tmpSpeed = currentSpeed_ + acc * timeStep;

        if (abs(tmpSpeed - setSpeed_) > abs(currentSpeed_ - setSpeed_))
        {
            // passed target speed
            currentSpeed_ = setSpeed_;
        }
        else
        {
            currentSpeed_ = tmpSpeed;
        }

        object_->SetLookaheadSensorPosition(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ());
    }

    if (mode_ == ControlOperationMode::MODE_OVERRIDE && !virtual_)
    {
        object_->MoveAlongS(currentSpeed_ * timeStep);
    }

    if (virtual_)
    {
        double acc_v[2] = {0.0, 0.0};
        RotateVec2D(acc, 0.0, object_->pos_.GetH(), acc_v[0], acc_v[1]);
        object_->SetAcc(acc_v[0], acc_v[1], 0.0);
    }
    else
    {
        object_->SetSpeed(currentSpeed_);
    }

    if (log_stream_.is_open())
    {
        log_stream_ << scenario_engine_->getSimulationTime() << ","
                    << (perception_noisy_ ? "noisy" : "ideal") << ","
                    << (selectedObject == nullptr ? -1 : selectedObject->GetId()) << ","
                    << selectedPerceivedDistance << ","
                    << selectedTrueDistance << ","
                    << selectedDistanceError << ","
                    << object_->GetSpeed() << ","
                    << (selectedObject == nullptr ? 0.0 : selectedObject->GetSpeed()) << ","
                    << currentSpeed_ << ","
                    << (!object_->collisions_.empty() ? 1 : 0) << "\n";
    }

    Controller::Step(timeStep);
}

int ControllerACC::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    currentSpeed_ = object_->GetSpeed();
    if (mode_ == ControlOperationMode::MODE_ADDITIVE || setSpeedSet_ == false)
    {
        setSpeed_ = object_->GetSpeed();
    }

    Controller::Activate(mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        // Make sure heading is aligned with road driving direction
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }

    if (player_)
    {
        player_->LookaheadSensorSetVisible(object_->GetId(), true);
    }

    return 0;
}

void ControllerACC::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}