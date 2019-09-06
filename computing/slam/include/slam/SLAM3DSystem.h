#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/g2o_core_api.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/vertex_se3_euler.h>

#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/csparse/csparse_extension.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <g2o/stuff/command_args.h>

#include <slam/VertexSE3WithData.h>

#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class SLAM3DSystem
{
public:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;
    g2o::OptimizationAlgorithm* _currentSolver;

    struct RPYpose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
    struct ids
    {
        int i;
        int j;
    };

    std::string save_path = "";

    std::map<int, g2o::VertexSE3WithData*> vertexMap;

    SLAM3DSystem();
    ~SLAM3DSystem();

    void initG2O(std::string tag);

    void setTF(Eigen::Matrix4f _tf_vtol);

    Eigen::Matrix4f getTF();

    int getID();

    void setMinAddShift(float min_add_shift);

    void createVertexSE3(const number_t *estimate, pcl::PointCloud<pcl::PointXYZI> data, g2o::VertexSE3WithData &v);

    void createEdgeSE3(g2o::VertexSE3WithData *vi, g2o::VertexSE3WithData *vj, g2o::EdgeSE3 &e);
    void createEdgeSE3(g2o::VertexSE3WithData *vi, g2o::VertexSE3WithData *vj, number_t *d, g2o::EdgeSE3 &e);

    void RPYposeToMatrix(const RPYpose pose, Eigen::Matrix4f& matrix);

    void MatrixToRPYpose(const Eigen::Matrix4f matrix, RPYpose& pose);

    void MatrixToMeasurement(const Eigen::Matrix4f matrix, number_t *d);

    void EstimateToMatrix(const number_t *d, Eigen::Matrix4f& matrix);

    void EstimateToRPYpose(const number_t *d, RPYpose& pose);

    void buildCloudMap(pcl::PointCloud<pcl::PointXYZI>& cloud_map);

    void buildLocalCloudMap(const int targetID, const int n_forward, const int n_backward, pcl::PointCloud<pcl::PointXYZI>& target_map);

    bool searchLoopClosing(const g2o::VertexSE3WithData* v_current, const float lookup_LCdist, const int skip_idx, std::vector<int> &LCids);

    bool addVertex(g2o::VertexSE3WithData *v);

    bool addEdge(g2o::EdgeSE3 *e);

    void optimizeGraph(int iterations, bool initialize);

    void ReadData(const std::string path);

    void makeGraphMarkArray(visualization_msgs::MarkerArray &markers, const std::vector<ids> LC_id_map);
    void makeGraphMarkArray(visualization_msgs::MarkerArray &markers);

    bool saveData();

protected:
    int _v_id;
    float _min_add_scan_shift;
    Eigen::Matrix4f _tf_vtol ;
    Eigen::Matrix4f _tf_ltov ;

private:
    g2o::SparseOptimizer _optimizer;
};