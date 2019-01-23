#include "distributed_mapper/evaluation_utils.h"

using namespace std;
using namespace gtsam;

namespace distributed_mapper {

static const Matrix I9 = eye(9);
static const Vector zero9 = Vector::Zero(9);
static const Matrix zero33 = Matrix::Zero(3, 3);
static const Key keyAnchor = symbol('Z', 9999999);

namespace multirobot_util{
  //*****************************************************************************
  Vector rowMajorVector(Matrix3 R) {
    return (Vector(9) << R(0, 0), R(0, 1), R(0, 2),/*  */ R(1, 0), R(1, 1), R(1, 2), /*  */ R(2, 0), R(2, 1), R(2,
                                                                                                                2)).finished();
  }

  //*****************************************************************************
  void printKeys(Values values) {
    cout << "Keys: ";
    for (const Values::ConstKeyValuePair &key_value: values) {
      Key key = key_value.key;
      cout << symbolChr(key) << symbolIndex(key) << " ";
    }
    cout << endl;
  }

  //*****************************************************************************
  void printKeys(NonlinearFactorGraph graph) {
    cout << "Factor Keys: ";
    for (size_t k = 0; k < graph.size(); k++) {
      KeyVector keys = graph.at(k)->keys();
      if (keys.size() == 2) {
        cout << "(" << symbolChr(keys.at(0)) << symbolIndex(keys.at(0)) << "," <<
             symbolChr(keys.at(1)) << symbolIndex(keys.at(1)) << ") ";
      }
    }
    cout << endl;
  }

  //*****************************************************************************
  Values pose3WithZeroTranslation(Values rotations) {
    Values poses;
    for (const Values::ConstKeyValuePair &key_value: rotations) {
      Key key = key_value.key;
      Pose3 pose(rotations.at<Rot3>(key), zero(3));
      poses.insert(key, pose);
    }
    return poses;
  }

  //*****************************************************************************
  VectorValues initializeVectorValues(Values rotations) {
    VectorValues vectorValues;
    for (const Values::ConstKeyValuePair &key_value: rotations) {
      Key key = key_value.key;
      vectorValues.insert(key, zero(6));
    }
    return vectorValues;
  }

  //*****************************************************************************
  VectorValues
  initializeZeroRotation(Values subInitials) {
    VectorValues subInitialsVectorValue;
    for (const Values::ConstKeyValuePair &key_value: subInitials) {
      Vector r = zero(9);
      subInitialsVectorValue.insert(key_value.key, r);
    }
    return subInitialsVectorValue;
  }

  //*****************************************************************************
  VectorValues
  rowMajorVectorValues(Values subInitials) {
    VectorValues subInitialsVectorValue;
    for (const Values::ConstKeyValuePair &key_value: subInitials) {
      Pose3 pose = subInitials.at<Pose3>(key_value.key);
      Matrix3 R = pose.rotation().matrix();
      Vector r = rowMajorVector(R);
      subInitialsVectorValue.insert(key_value.key, r);
    }
    return subInitialsVectorValue;
  }

  //*****************************************************************************
  Values retractPose3Global(Values initial, VectorValues delta) {
    Values estimate;
    for (const Values::ConstKeyValuePair &key_value: initial) {
      Key key = key_value.key;
      Vector6 deltaPose = delta.at(key);
      Rot3 R = initial.at<Pose3>(key).rotation().retract(deltaPose.head(3));
      Point3 t = initial.at<Pose3>(key).translation() + Point3(deltaPose.tail(3));
      estimate.insert(key, Pose3(R, t));
    }
    return estimate;
  }

  //*****************************************************************************
  pair<vector<NonlinearFactorGraph>, vector<Values> >
  loadSubgraphs(size_t numSubgraphs, string dataPath) {
    vector<NonlinearFactorGraph> subGraphs;
    vector<Values> subInitials;

    for (size_t i = 0; i < numSubgraphs; i++) { // for each robot
      string dataFile_i = dataPath + boost::lexical_cast<string>(i) + ".g2o";
      GraphAndValues graphAndValues_i = readG2o(dataFile_i, true);

      subGraphs.push_back(*(graphAndValues_i.first));
      subInitials.push_back(*(graphAndValues_i.second));
      cout << "Variables in ith subgraph: " << graphAndValues_i.second->size() << endl;
    }

    return make_pair(subGraphs, subInitials);
  }

  //*****************************************************************************
  pair<NonlinearFactorGraph, Values>
  loadGraphWithPrior(string dataFile, const SharedNoiseModel &priorModel) {

    GraphAndValues graphAndValues = readG2o(dataFile, true);
    NonlinearFactorGraph graphCentralized = *(graphAndValues.first);
    Values initialCentralized = *(graphAndValues.second);

    // Add prior
    NonlinearFactorGraph graphWithPrior = graphCentralized;
    graphWithPrior.add(PriorFactor<Pose3>(0, Pose3(), priorModel));

    return make_pair(graphWithPrior, initialCentralized);
  }

  //*****************************************************************************

  NonlinearFactorGraph
  convertToChordalGraph(NonlinearFactorGraph graph,
                        const SharedNoiseModel &betweenNoise,
                        bool useBetweenNoise) {
    NonlinearFactorGraph cenFG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (useBetweenNoise) {
          // Convert noise model to chordal factor noise
          SharedNoiseModel chordalNoise = multirobot_util::convertToChordalNoise(factor->get_noiseModel());
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordalNoise));
        } else {
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, betweenNoise));
        }
      }
    }
    return cenFG;
  }

  /* ************************************************************************* */
  GaussianFactorGraph
  buildLinearOrientationGraph(const NonlinearFactorGraph &g, bool useBetweenNoise) {

    GaussianFactorGraph linearGraph;
    SharedDiagonal model = noiseModel::Unit::Create(9);

    for (const boost::shared_ptr<NonlinearFactor> &factor: g) {
      Matrix3 Rij;

      boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
      if (pose3Between)
        Rij = pose3Between->measured().rotation().matrix();
      else
        std::cout << "Error in buildLinearOrientationGraph" << std::endl;

      // if using between noise, use the factor noise model converted to a conservative diagonal estimate
      if (useBetweenNoise) {
        model = multirobot_util::convertToDiagonalNoise(pose3Between->get_noiseModel());
      }

      const FastVector<Key> &keys = factor->keys();
      Key key1 = keys[0], key2 = keys[1];
      Matrix M9 = Matrix::Zero(9, 9);
      M9.block(0, 0, 3, 3) = Rij;
      M9.block(3, 3, 3, 3) = Rij;
      M9.block(6, 6, 3, 3) = Rij;

      linearGraph.add(key1, -I9, key2, M9, zero9, model);
    }
    // prior on the anchor orientation
    SharedDiagonal priorModel = noiseModel::Unit::Create(9);
    linearGraph.add(keyAnchor,
                    I9,
                    (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0).finished(),
                    priorModel);
    return linearGraph;
  }

  //*****************************************************************************
  Values
  centralizedEstimation(NonlinearFactorGraph graph,
                        const SharedNoiseModel &betweenNoise,
                        const SharedNoiseModel &priorNoise,
                        bool useBetweenNoise) {

    // STEP 1: solve for rotations first, using chordal relaxation (centralized)
    // This is the "expected" solution
    // pose3Graph only contains between factors, as the priors are converted to between factors on an anchor key
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graph);
    // this will also put a prior on the anchor
    GaussianFactorGraph centralizedLinearGraph = buildLinearOrientationGraph(pose3Graph, useBetweenNoise);
    VectorValues rotEstCentralized = centralizedLinearGraph.optimize();
    Values cenRot = InitializePose3::normalizeRelaxedRotations(rotEstCentralized);
    Values initial = pose3WithZeroTranslation(cenRot);

    // STEP 2: solve for poses using initial estimate from rotations
    // Linearize pose3 factor graph, using chordal formulation
    NonlinearFactorGraph cenFG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (useBetweenNoise) {
          // Convert noise model to chordal factor noise
          SharedNoiseModel chordalNoise = multirobot_util::convertToChordalNoise(factor->get_noiseModel());
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordalNoise));
        } else {
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, betweenNoise));
        }
      }
    }

    NonlinearFactorGraph chordalGraphWithoutPrior = cenFG.clone();

    // First key
    Key firstKey = KeyVector(initial.keys()).at(0);
    cenFG.add(PriorFactor<Pose3>(firstKey, initial.at<Pose3>(firstKey), priorNoise));

    Values estimate = initial;
    for (size_t iter = 0; iter < 1; iter++) {
      GaussianFactorGraph cenGFG = *(cenFG.linearize(estimate));
      VectorValues cenPose_VectorValues = cenGFG.optimize(); // optimize
      estimate = retractPose3Global(estimate, cenPose_VectorValues);
    }

    //std::cout << "Centralized Two Stage Error: " << chordalGraphWithoutPrior.error(estimate) << std::endl;
    return estimate;

  }

  //*****************************************************************************
  Values
  centralizedGNEstimation(NonlinearFactorGraph graph,
                          const SharedNoiseModel &betweenNoise,
                          const SharedNoiseModel &priorNoise,
                          bool useBetweenNoise) {

    // STEP 1: solve for rotations first, using chordal relaxation (centralized)
    // This is the "expected" solution
    // pose3Graph only contains between factors, as the priors are converted to between factors on an anchor key
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graph);
    // this will also put a prior on the anchor
    GaussianFactorGraph centralizedLinearGraph = InitializePose3::buildLinearOrientationGraph(pose3Graph);
    VectorValues rotEstCentralized = centralizedLinearGraph.optimize();
    Values cenRot = InitializePose3::normalizeRelaxedRotations(rotEstCentralized);
    Values initial = pose3WithZeroTranslation(cenRot);

    // STEP 2: solve for poses using initial estimate from rotations
    // Linearize pose3 factor graph, using chordal formulation
    NonlinearFactorGraph cenFG;
    for (size_t k = 0; k < graph.size(); k++) {
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if (factor) {
        Key key1 = factor->keys().at(0);
        Key key2 = factor->keys().at(1);
        Pose3 measured = factor->measured();

        if (useBetweenNoise) {
          // Convert noise model to chordal factor noise
          Rot3 rotation = cenRot.at<Rot3>(key1);
          SharedNoiseModel
              chordalNoise = multirobot_util::convertToChordalNoise(factor->get_noiseModel(), rotation.matrix());
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, chordalNoise));
        } else {
          cenFG.add(BetweenChordalFactor<Pose3>(key1, key2, measured, betweenNoise));
        }
      }
    }

    NonlinearFactorGraph chordalGraphWithoutPrior = cenFG.clone();

    // First key
    // Add prior
    Key firstKey = KeyVector(initial.keys()).at(0);
    NonlinearFactor::shared_ptr firstKeyPrior(new PriorFactor<Pose3>(firstKey, initial.at<Pose3>(firstKey), priorNoise));
    cenFG.push_back(firstKeyPrior);

    Values chordalGN = initial;
    for (size_t iter = 0; iter < 5; iter++) {
      GaussianFactorGraph chordalGFG = *(cenFG.linearize(chordalGN));
      VectorValues chordalVectorValues = chordalGFG.optimize(); // optimize
      chordalGN = retractPose3Global(chordalGN, chordalVectorValues);
    }

    // std::cout << "Centralized Two Stage + GN Error: " << chordalGraphWithoutPrior.error(chordalGN) << std::endl;

    return chordalGN;
  }

  //*****************************************************************************
  SharedNoiseModel convertToChordalNoise(SharedNoiseModel noise, Matrix Rhat) {

    // Converted chordal covariance
    Matrix CChordal = Matrix::Zero(12, 12);

    // Extract square root information matrix
    SharedGaussian gaussianNoise = boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise);
    Matrix R = gaussianNoise->R();
    Matrix C = gtsam::inverse(trans(R) * R); // get covariance from square root information
    //gaussianNoise->

    // convert rotation to a conservative isotropic noise model
    double maxC = DBL_MIN;
    for (size_t i = 0; i < 3; i++) {
      if (C(i, i) > maxC)
        maxC = C(i, i);
    }

    CChordal(0, 0) = maxC;
    CChordal(1, 1) = maxC;
    CChordal(2, 2) = maxC;
    CChordal(3, 3) = maxC;
    CChordal(4, 4) = maxC;
    CChordal(5, 5) = maxC;
    CChordal(6, 6) = maxC;
    CChordal(7, 7) = maxC;
    CChordal(8, 8) = maxC;

    // Translation noise is kept as it is
    Matrix translation = C.block(3, 3, 3, 3) + 0.01 * gtsam::eye(3, 3);
    CChordal.block(9, 9, 3, 3) = Rhat * translation * trans(Rhat);

    SharedNoiseModel chordalNoise = noiseModel::Diagonal::Gaussian::Covariance(CChordal);
    return chordalNoise;
  }

  //*****************************************************************************
  SharedDiagonal convertToDiagonalNoise(SharedNoiseModel noise) {

    // Extract square root information matrix
    SharedGaussian gaussianNoise = boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise);
    Matrix R = gaussianNoise->R();
    Matrix C = gtsam::inverse(trans(R) * R); // get covariance from square root information

    // convert rotation to a conservative isotropic noise model
    double maxC = DBL_MIN;
    for (size_t i = 0; i < 3; i++) {
      if (C(i, i) > maxC)
        maxC = C(i, i);
    }

    SharedDiagonal chordalNoise = noiseModel::Diagonal::Sigmas(repeat(9, maxC));
    return chordalNoise;
  }

  //*****************************************************************************
  void
  writeValuesAsTUM(gtsam::Values values, std::string filename) {

    // Open filestream
    fstream fileStream(filename.c_str(), fstream::out);

    for (const Values::ConstKeyValuePair &key_value: values) {
      Key key = key_value.key;

      // Since TUM rgbd tools are used to compare trajectory poses, do not process object landmarks
      if (isupper(symbolChr(key)))
        continue;

      long unsigned int index = (long unsigned int) symbolIndex(key);
      Pose3 pose = values.at<Pose3>(key);
      Quaternion quat = pose.rotation().toQuaternion();
      Point3 trans = pose.translation();

      char outputTime[18];
      sprintf(outputTime, "%010lu.000000", index);

      fileStream << outputTime << " " << trans.x() << " " << trans.y() << " " << trans.z() << " "
                 << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << endl;
    }

    // Close filestream
    fileStream.close();
  }

  GraphAndValues readFullGraph(size_t nrRobots, // number of robots
                               vector<GraphAndValues> graphAndValuesVec  // vector of all graphs and initials for each robot
  ) {
    std::cout << "Creating fullGraph by combining subgraphs." << std::endl;

    // Combined graph and Values
    NonlinearFactorGraph::shared_ptr combinedGraph(new NonlinearFactorGraph);
    Values::shared_ptr combinedValues(new Values);

    // Iterate over each robot
    for (size_t robot = 0; robot < nrRobots; robot++) {

      // Load graph and initial
      NonlinearFactorGraph graph = *(graphAndValuesVec[robot].first);
      Values initial = *(graphAndValuesVec[robot].second);

      // Iterate over initial and push it to the combinedValues, each initial value is present only once
      for (const Values::ConstKeyValuePair &key_value: initial) {
        Key key = key_value.key;
        if (!combinedValues->exists(key))
          combinedValues->insert(key, initial.at<Pose3>(key));
      }

      // Iterate over the graph and push the factor if it is not already present in combinedGraph
      for (size_t ksub = 0; ksub < graph.size(); ksub++) { //for each factor in the new subgraph
        bool isPresent = false;
        for (size_t k = 0; k < combinedGraph->size(); k++) {

          boost::shared_ptr<BetweenFactor<Pose3>> factorSub =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
          Key factorSubKey1 = factorSub->key1();
          Key factorSubKey2 = factorSub->key2();

          boost::shared_ptr<BetweenFactor<Pose3>> factorCombined =
              boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combinedGraph->at(k));
          Key factorCombinedKey1 = factorCombined->key1();
          Key factorCombinedKey2 = factorCombined->key2();

          // values don't match exactly that's why check with keys as well
          if (factorCombined->equals(*factorSub) ||
              ((factorSubKey1 == factorCombinedKey1) && (factorSubKey2 == factorCombinedKey2))) {
            isPresent = true;
            break;
          }
        }
        if (isPresent == false) // we insert the factor
          combinedGraph->add(graph.at(ksub));
      }
    }

    // Return graph and values
    return make_pair(combinedGraph, combinedValues);
  }

  std::pair<double, double> evaluateEstimates(const size_t &nrRobots,
                                              const gtsam::GraphAndValues &fullGraphAndValues,
                                              const gtsam::noiseModel::Diagonal::shared_ptr &priorModel,
                                              const gtsam::noiseModel::Isotropic::shared_ptr &model,
                                              const bool &useBetweenNoise,
                                              const gtsam::Values &distributedEstimates) {
    ////////////////////////////////////////////////////////////////////////////////
    // Extract full graph and add prior
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);
    Values fullInitial = *(fullGraphAndValues.second);

    // Add prior
    NonlinearFactorGraph fullGraphWithPrior = fullGraph.clone();
    Key priorKey = KeyVector(fullInitial.keys()).at(0);
    NonlinearFactor::shared_ptr prior(
        new PriorFactor<Pose3>(priorKey, fullInitial.at<Pose3>(priorKey), priorModel));
    fullGraphWithPrior.push_back(prior);

    ////////////////////////////////////////////////////////////////////////////////
    // Chordal Graph
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph chordalGraph = convertToChordalGraph(
        fullGraph, model, useBetweenNoise);

    ////////////////////////////////////////////////////////////////////////////////
    // Initial Error
    ////////////////////////////////////////////////////////////////////////////////
    std::cout << "Initial Error: " << chordalGraph.error(fullInitial) << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage
    ////////////////////////////////////////////////////////////////////////////////
    Values centralized = centralizedEstimation(fullGraphWithPrior,
                                                                   model, priorModel,
                                                                   useBetweenNoise);
    double centralized_error = chordalGraph.error(centralized);
    std::cout << "Centralized Two Stage Error: " << centralized_error << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Two Stage + Gauss Newton
    ////////////////////////////////////////////////////////////////////////////////
    Values chordalGN = centralizedGNEstimation(fullGraphWithPrior,
                                                                   model, priorModel,
                                                                   useBetweenNoise);
    std::cout << "Centralized Two Stage + GN Error: " << chordalGraph.error(chordalGN) << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    // Distributed Error
    ////////////////////////////////////////////////////////////////////////////////
    double distributed_error = chordalGraph.error(distributedEstimates);
    std::cout << "Distributed Error: " << distributed_error << std::endl;

    return std::make_pair(centralized_error, distributed_error);
  }
}
}

