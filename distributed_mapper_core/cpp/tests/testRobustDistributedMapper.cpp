/**
 * @file testRobustDistributedMapper.cpp
 * @author Pierre-Yves Lajoie (lajoie.py@gmail.com)
 * @brief unit tests for test multi robot case with outliers simulated by loading the datasets
 */
#include <distributed_mapper/DistributedMapper.h>
#include <distributed_mapper/DistributedMapperUtils.h>
#include <distributed_mapper/MultiRobotUtils.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include "distributed_pcm/distributed_pcm.h"

using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
using namespace multirobot_util;

/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimationWithOutliersNoRotation_2robots) {

    // Compare centralized and distributed pose estimates
    //COMPARE_VALUES_DATASET(nrRobots, centralized, distributed_estimates, 1e-1);
}


/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************
