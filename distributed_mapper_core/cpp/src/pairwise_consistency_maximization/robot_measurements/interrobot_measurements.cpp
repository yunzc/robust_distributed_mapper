// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "robot_measurements/interrobot_measurements.h"

namespace robot_measurements {

InterRobotMeasurements::InterRobotMeasurements(const graph_utils::Transforms& transforms,
                                               const graph_utils::LoopClosures& loop_closures,
                                               const unsigned char& robot1_id,
                                               const unsigned char& robot2_id):
                                               RobotMeasurements(transforms, loop_closures),
                                               robot1_id_(robot1_id), robot2_id_(robot2_id){}

const unsigned char& InterRobotMeasurements::getRobot1ID() const {
  return robot1_id_;
};

const unsigned char& InterRobotMeasurements::getRobot2ID() const {
  return robot2_id_;
};


}