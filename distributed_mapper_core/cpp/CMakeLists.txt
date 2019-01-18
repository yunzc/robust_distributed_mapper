cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(distributed-mapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

# for unittest examples
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
include(GtsamTesting)

###########################################################################
# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
message("Boost Libraries: ${Boost_LIBRARIES}")

###########################################################################
# Pairwise Consistency Maximization

# Overall include directories
include_directories(
  include
  include/pairwise_consistency_maximization
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/
)

# Graph utils library
add_library(graph_utils
        src/pairwise_consistency_maximization/graph_utils/graph_utils_functions.cpp
)
target_link_libraries(graph_utils
        gtsam
)

# Robot local map library
add_library(robot_local_map
        src/pairwise_consistency_maximization/robot_local_map/robot_measurements.cpp
        src/pairwise_consistency_maximization/robot_local_map/robot_local_map.cpp
        )
target_link_libraries(robot_local_map
        graph_utils
        )

# Paiwise Consistency computation library
add_library(pairwise_consistency
   src/pairwise_consistency_maximization/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(pairwise_consistency
       robot_local_map
)

# Maximum Clique Solver
add_library(fast_max-clique_finder
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.h
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findClique.cpp
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/utils.cpp 
    src/pairwise_consistency_maximization/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# Global map solver library
add_library(global_map_solver
   src/pairwise_consistency_maximization/global_map/global_map.cpp
)
target_link_libraries(global_map_solver
   graph_utils
   pairwise_consistency
   fast_max-clique_finder
)

# Distributed PCM library
add_library(distributed_pcm
        src/pairwise_consistency_maximization/distributed_pcm/distributed_pcm.cpp
)
target_link_libraries(distributed_pcm
        global_map_solver
)

# Distributed Mapper
add_library (distributed_mapper
        src/distributed_mapper/DistributedMapper.cpp
        src/distributed_mapper/MultiRobotUtils.cpp
        include/distributed_mapper/DistributedMapperUtils.h)

target_link_libraries(distributed_mapper
        ${Boost_LIBRARIES}
        gtsam
        distributed_pcm)


# Distributed mapper utils library
#add_library(distributed_mapper_utils
#        )
#target_link_libraries(distributed_mapper_utils
#        distributed_mapper
#        )

###########################################################################
# Add example executable
add_executable (runDistributedMapper
        examples/runDistributedMapper.cpp)
target_link_libraries (runDistributedMapper
        distributed_mapper)

###########################################################################
# Tests
add_subdirectory(tests)

###########################################################################
# Install
#file(GLOB headers "src/distributed_mapper/*.h")
#install(FILES ${headers} DESTINATION include/distributed_mapper/)
#configure_file (${PROJECT_SOURCE_DIR}/DistributedMapperConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/DistributedMapperConfig.cmake @ONLY)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/DistributedMapperConfig.cmake DESTINATION share/distributed_mapper)


