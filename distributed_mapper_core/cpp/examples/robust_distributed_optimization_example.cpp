
#include "distributed_mapper/runDistributedMapper.h"

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {
    //////////////////////////////////////////////////////////////////////////////////////
    //Command line arguments
    //////////////////////////////////////////////////////////////////////////////////////
    size_t nrRobots = 2; // number of robots
    string logDir("/tmp/"); // log directory
    string dataDir("/tmp/"); // data directory
    string traceFile("/tmp/runG2o"); // data directory
    bool useXY = false;
    bool useOP = false;
    bool debug = false;

    ////////////////////////////////////////////////////////////////////////////////
    // Config (specify noise covariances and maximum number iterations)
    ////////////////////////////////////////////////////////////////////////////////
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t maxIter = 1000; // Maximum number of iterations of optimizer
    double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool useFlaggedInit = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType updateType = distributed_mapper::DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool useBetweenNoise = false; // use between factor noise or not
    bool useChrLessFullGraph = false; // whether full graph has character indexes or not
    bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double confidence_probability = 0.99; // confidence probability for the pairwise consistency computation.
    bool useCovariance = false; // use covariance in dataset file.
    bool usePCM = true; // Use pairwise consistency maximization.

    try {
        // Parse program options
        namespace po = boost::program_options;
        po::options_description desc("Options");
        desc.add_options()
                ("help", "Print help messages")
                ("nrRobots, n", po::value<size_t>(&nrRobots), "number of robots (default: 2)")
                ("dataDir, l", po::value<string>(&dataDir), "data directory (default: /tmp)")
                ("traceFile, t", po::value<string>(&traceFile), "trace file (default: runG2o)")
                ("logDir, l", po::value<string>(&logDir), "log directory (default: /tmp)")
                ("useXY, u", po::value<bool>(&useXY), "use x,y,z as naming convention or a,b,c (default: x,y,z)")
                ("useOP, o", po::value<bool>(&useOP), "use o,p,q as naming convention (default: x,y,z)")
                ("useFlaggedInit, f", po::value<bool>(&useFlaggedInit),
                 "use flagged initialization or not (default: true)")
                ("useBetweenNoise, b", po::value<bool>(&useBetweenNoise),
                 "use the given factor between noise instead of unit noise(default: false)")
                ("useChrLessFullGraph", po::value<bool>(&useChrLessFullGraph),
                 "whether full graph has character indexes or not (default: false)")
                ("useLandmarks, l", po::value<bool>(&useLandmarks), "use landmarks or not (default: false)")
                ("rthresh, r", po::value<double>(&rotationEstimateChangeThreshold),
                 "Specify difference between rotation estimate provides an early stopping condition (default: 1e-2)")
                ("pthresh, p", po::value<double>(&poseEstimateChangeThreshold),
                 "Specify difference between pose estimate provides an early stopping condition (default: 1e-2)")
                ("maxIter, m", po::value<size_t>(&maxIter), "maximum number of iterations (default: 100000)")
                ("confidence, c", po::value<double>(&confidence_probability), "confidence probability for the pairwise consistency computation (default: 0.99)")
                ("useCovariance, i", po::value<bool>(&useCovariance), "use covariance in dataset file (default: false)")
                ("debug, d", po::value<bool>(&debug), "debug (default: false)")
                ("usePCM", po::value<bool>(&usePCM), "use pairwise consistency maximization (default: true)");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
            if (vm.count("help")) { // --help option
                cout << "Run Distributed-Mapper" << endl
                     << "Example: ./rung2o --dataDir ../../../example/ --nrRobots 4"
                     << endl << desc << endl;
                return 0;
            }
            po::notify(vm); // throws on error, so do after help in case
        }
        catch (po::error &e) {
            cerr << "ERROR: " << e.what() << endl << endl;
            cerr << desc << endl;
            return 1;
        }
    }
    catch (exception &e) {
        cerr << "Unhandled Exception reached the top of main: "
             << e.what() << ", application will now exit" << endl;
        return 2;
    }
    std::tuple<double, double, int> results = distributed_mapper::runDistributedMapper(nrRobots, logDir, dataDir, traceFile,
            useXY, useOP, debug, priorModel, model,
            maxIter, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
            gamma, useFlaggedInit, updateType, useBetweenNoise,
            useChrLessFullGraph, useLandmarks, confidence_probability, useCovariance, usePCM);

    return 0;
}