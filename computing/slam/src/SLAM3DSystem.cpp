#include <slam/SLAM3DSystem.h>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

SLAM3DSystem::SLAM3DSystem()
{
    _v_id = 0;
    _min_add_scan_shift = 2.0;
    _tf_vtol = Eigen::Matrix4f::Identity();
    _tf_ltov = Eigen::Matrix4f::Identity();
}
SLAM3DSystem::~SLAM3DSystem()
{
//    _optimizer.save("/home/pharos/test2_ws/src/computing/slam/data/g2o.g2o");
}

void SLAM3DSystem::createVertexSE3(const number_t *estimate, pcl::PointCloud<pcl::PointXYZI> data, g2o::VertexSE3WithData &v)
{
    v.setId(_v_id);
    v.setEstimateData(estimate);
    v.scan_data = data;
}

void SLAM3DSystem::createEdgeSE3(g2o::VertexSE3WithData *vi, g2o::VertexSE3WithData *vj, g2o::EdgeSE3 &e)
{
    if(vi->id()==0)
        std::cout << _optimizer.vertex(vi->id())->fixed() << std::endl;
    e.vertices()[0] = _optimizer.vertex(vi->id());
    e.vertices()[1] = _optimizer.vertex(vj->id());
    e.setMeasurementFromState();
    e.setInformation(g2o::EdgeSE3::InformationType::Identity());
}

void SLAM3DSystem::createEdgeSE3(g2o::VertexSE3WithData *vi, g2o::VertexSE3WithData *vj, number_t *d, g2o::EdgeSE3 &e)
{
    e.vertices()[0] = _optimizer.vertex(vi->id());
    e.vertices()[1] = _optimizer.vertex(vj->id());
    e.setMeasurementData(d);
//    e.setInformation(g2o::EdgeSE3::InformationType::Identity());
}

void SLAM3DSystem::RPYposeToMatrix(const RPYpose pose, Eigen::Matrix4f &matrix)
{
    Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f translation(pose.x, pose.y, pose.z);

    matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();
}

void SLAM3DSystem::MatrixToRPYpose(const Eigen::Matrix4f matrix, RPYpose& pose)
{
    tf::Matrix3x3 mat33;

    mat33.setValue(static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
                   static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
                   static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
                   static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
                   static_cast<double>(matrix(2, 2)));


    // Update localizer_pose.
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);
    mat33.getRPY(pose.roll, pose.pitch, pose.yaw, 1);
}

void SLAM3DSystem::MatrixToMeasurement(const Eigen::Matrix4f matrix, number_t *d)
{
    tf::Matrix3x3 mat33;

    mat33.setValue(static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
                   static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
                   static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
                   static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
                   static_cast<double>(matrix(2, 2)));

    tf::Quaternion q;
    mat33.getRotation(q);

    // Update measurement data
    d[0] = matrix(0, 3);
    d[1] = matrix(1, 3);
    d[2] = matrix(2, 3);
    d[3] = q.x();
    d[4] = q.y();
    d[5] = q.z();
    d[6] = q.w();
}

void SLAM3DSystem::EstimateToMatrix(const number_t *d, Eigen::Matrix4f& matrix)
{
    Eigen::Translation3f translation(d[0],d[1],d[2]);
    Eigen::Quaternionf q;
    q.x() = d[3];
    q.y() = d[4];
    q.z() = d[5];
    q.w() = d[6];
    Eigen::Matrix3f rotation = q.normalized().toRotationMatrix();

    matrix = (translation*rotation).matrix();
}

void SLAM3DSystem::EstimateToRPYpose(const number_t *d, RPYpose& pose)
{
    pose.x = d[0];
    pose.y = d[1];
    pose.z = d[2];

    tf::Quaternion q;
    q.setValue(d[3], d[4], d[5], d[6]);
    tf::Matrix3x3(q).getRPY(pose.roll, pose.pitch, pose.yaw);
}

void SLAM3DSystem::initG2O(std::string tag)
{
    g2o::OptimizationAlgorithmFactory* solverFactory = g2o::OptimizationAlgorithmFactory::instance();
    _currentSolver = solverFactory->construct(tag, _currentOptimizationAlgorithmProperty);

//    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
//    linearSolver->setBlockOrdering(false);
//    g2o::OptimizationAlgorithmGaussNewton* solverGauss = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    _optimizer.setAlgorithm(_currentSolver);

    std::cout << "initG2O" << std::endl;
}

void SLAM3DSystem::setTF(Eigen::Matrix4f tf_vtol)
{
    _tf_vtol = tf_vtol;
    _tf_ltov = tf_vtol.inverse();
}

Eigen::Matrix4f SLAM3DSystem::getTF()
{
    return _tf_vtol;
}

int SLAM3DSystem::getID()
{
    return _v_id;
}

void SLAM3DSystem::setMinAddShift(float min_add_shift)
{
    _min_add_scan_shift = min_add_shift;
}

bool SLAM3DSystem::addVertex(g2o::VertexSE3WithData *v)
{
    g2o::VertexSE3* vertex(new g2o::VertexSE3);
    double est[7];

    v->getEstimateData(est);
    vertex->setEstimateData(est);
    vertex->setId(_v_id);
    vertex->setFixed(v->fixed());

    if(_optimizer.addVertex(vertex))
    {
        vertexMap[_v_id] = v;
        _v_id++;
        return true;
    }
    return false;
}

bool SLAM3DSystem::addEdge(g2o::EdgeSE3 *e)
{
    if(_optimizer.addEdge(e))
        return true;

    return false;
}

//bool SLAM3DSystem::addLoopCloseEdge(g2o::EdgeSE3 *e)
//{
//    if(optimizer.addEdge(e))
//    {
//        LCindex++;
//        if()
//        return true;
//    } else return false;
//}

bool SLAM3DSystem::searchLoopClosing(const g2o::VertexSE3WithData* v_current, const float lookup_LCdist, const int skip_idx, std::vector<int> &LCids)
{
    bool find_LCids = false;
    float num_searching_idx = lookup_LCdist/_min_add_scan_shift*2;
    for(auto v = vertexMap.begin(); v!=vertexMap.end(); v++)
    {
        g2o::VertexSE3WithData *v_past = v->second;

        if(v_current->id() - v->first < num_searching_idx || v_current->id() <= v->first)
            continue;

        auto d_dist = sqrt(pow(v_past->estimate().translation()[0]-v_current->estimate().translation()[0],2)+pow(v_past->estimate().translation()[1]-v_current->estimate().translation()[1],2));
        if(d_dist > lookup_LCdist)
            continue;

        LCids.push_back(v->first);
        find_LCids = true;

        for(int i=0; i<skip_idx; i++)
        {
            v++;
            if(v == vertexMap.end())
                break;
        }
    }
    return find_LCids;
}

void SLAM3DSystem::buildLocalCloudMap(const int targetID, const int n_forward, const int n_backward, pcl::PointCloud<pcl::PointXYZI>& target_map)
{
    for(int id=targetID-n_backward; id<targetID+n_forward; id++)
    {
        if(id < 0 || id > vertexMap.size()-1)
            continue;

        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;

        Eigen::Matrix4d T = vertexMap[id]->estimate().matrix()*_tf_vtol.matrix().cast<double>();

        pcl::transformPointCloud(vertexMap[id]->scan_data, transformed_cloud, T);

        target_map += transformed_cloud;
    }
}

void SLAM3DSystem::buildCloudMap(pcl::PointCloud<pcl::PointXYZI>& cloud_map)
{
    for(size_t id=0; id<vertexMap.size(); id++)
    {
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;

        Eigen::Matrix4d T = vertexMap[id]->estimate().matrix()*_tf_vtol.matrix().cast<double>();

        pcl::transformPointCloud(vertexMap[id]->scan_data, transformed_cloud, T);

        cloud_map += transformed_cloud;
    }
}

void SLAM3DSystem::optimizeGraph(int iterations, bool initialize)
{
    if(initialize)
    {
        if(_optimizer.initializeOptimization())
        {
            std::cout << "Start Otimization ....." << std::endl;
            _optimizer.optimize(iterations);
            std::cout << "Graph Otimization Done" << std::endl;
        }
    }
    else
    {
        std::cout << "Start Otimization ....." << std::endl;
        _optimizer.optimize(iterations);
        std::cout << "Graph Otimization Done" << std::endl;
    }

    for(int i=0; i<_optimizer.vertices().size(); i++)
    {
        double est[7];
        _optimizer.vertex(i)->getEstimateData(est);
        vertexMap[i]->setEstimateData(est);
    }

}

void SLAM3DSystem::ReadData(const std::string path)
{
    std::cout << "Data loading ...  (" << path << ")"<<std::endl;

    std::string g2o_filename = path + "graph.g2o";

    _optimizer.load(g2o_filename.c_str(), true);
    std::cout << _optimizer.edges().size() << std::endl;
    for(int i=0; i<_optimizer.vertices().size(); i++)
    {
        double estimate[7];
        g2o::VertexSE3WithData *v(new g2o::VertexSE3WithData);
        pcl::PointCloud<pcl::PointXYZI> scan_data;
        std::string pcd_filename = path + std::to_string(i) + ".pcd";

        _optimizer.vertex(i)->getEstimateData(estimate);
        pcl::io::loadPCDFile(pcd_filename, scan_data);
        createVertexSE3(estimate, scan_data, *v);
        v->setId(_optimizer.vertex(i)->id());
        vertexMap[i] = v;
    }
    _v_id = _optimizer.vertices().size()-1;
    std::cout << "Done!" << std::endl;
}

void SLAM3DSystem::makeGraphMarkArray(visualization_msgs::MarkerArray &markers, const std::vector<ids> LC_id_map)
{
    for(int i=0; i<vertexMap.size(); i++)
    {
        visualization_msgs::Marker vertex_marker;
        vertex_marker.action = visualization_msgs::Marker::ADD;
        vertex_marker.header.frame_id = "map";
        vertex_marker.ns = "vertex_se3";
        vertex_marker.type = visualization_msgs::Marker::SPHERE;
        vertex_marker.scale.x = 0.5;
        vertex_marker.scale.y = 0.5;
        vertex_marker.scale.z = 0.5;
        vertex_marker.color.r = 1;
        vertex_marker.color.a = 1;
        vertex_marker.id = i;

        double pose1[7], pose2[7];
        vertexMap.at(i)->getEstimateData(pose1);
        vertex_marker.pose.position.x = pose1[0];
        vertex_marker.pose.position.y = pose1[1];
        vertex_marker.pose.position.z = pose1[2];

        markers.markers.push_back(vertex_marker);

        if(i==vertexMap.size()-1)
            continue;

        visualization_msgs::Marker line_marker;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.header.frame_id  = "map";
        line_marker.ns = "edges";
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.scale.x = 0.2;
        line_marker.color.a = 1;
        line_marker.color.g = 1;
        line_marker.id = i;

        vertexMap.at(i+1)->getEstimateData(pose2);
        line_marker.points.resize(2);
        line_marker.points[0].x = pose1[0];
        line_marker.points[0].y = pose1[1];
        line_marker.points[0].z = pose1[2];

        line_marker.points[1].x = pose2[0];
        line_marker.points[1].y = pose2[1];
        line_marker.points[1].z = pose2[2];

        markers.markers.push_back(line_marker);

    }

    for(int i=0; i<LC_id_map.size(); i++)
    {
        visualization_msgs::Marker line_lc_marker;
        line_lc_marker.action = visualization_msgs::Marker::ADD;
        line_lc_marker.header.frame_id  = "map";
        line_lc_marker.ns = "lc_edges";
        line_lc_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_lc_marker.scale.x = 0.1;
        line_lc_marker.color.a = 1;
        line_lc_marker.color.b = 1;
        line_lc_marker.id = i;

        double pose1[7], pose2[7];

        vertexMap.at(LC_id_map[i].i)->getEstimateData(pose1);
        vertexMap.at(LC_id_map[i].j)->getEstimateData(pose2);

        line_lc_marker.points.resize(2);
        line_lc_marker.points[0].x = pose1[0];
        line_lc_marker.points[0].y = pose1[1];
        line_lc_marker.points[0].z = pose1[2];

        line_lc_marker.points[1].x = pose2[0];
        line_lc_marker.points[1].y = pose2[1];
        line_lc_marker.points[1].z = pose2[2];

        markers.markers.push_back(line_lc_marker);
    }
}

void SLAM3DSystem::makeGraphMarkArray(visualization_msgs::MarkerArray &markers)
{
    g2o::HyperGraph::EdgeSet e;

    for(int i=0; i<vertexMap.size(); i++)
    {
        visualization_msgs::Marker vertex_marker;
        vertex_marker.action = visualization_msgs::Marker::ADD;
        vertex_marker.header.frame_id = "map";
        vertex_marker.ns = "vertex_se3";
        vertex_marker.type = visualization_msgs::Marker::SPHERE;
        vertex_marker.scale.x = 0.5;
        vertex_marker.scale.y = 0.5;
        vertex_marker.scale.z = 0.5;
        vertex_marker.color.r = 1;
        vertex_marker.color.a = 1;
        vertex_marker.id = i;

        double pose1[7], pose2[7];
        vertexMap.at(i)->getEstimateData(pose1);
        vertex_marker.pose.position.x = pose1[0];
        vertex_marker.pose.position.y = pose1[1];
        vertex_marker.pose.position.z = pose1[2];

        markers.markers.push_back(vertex_marker);

        if(i==vertexMap.size()-1)
            continue;

        visualization_msgs::Marker line_marker;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.header.frame_id  = "map";
        line_marker.ns = "edges";
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.scale.x = 0.2;
        line_marker.color.a = 1;
        line_marker.color.g = 1;
        line_marker.id = i;

        vertexMap.at(i+1)->getEstimateData(pose2);
        line_marker.points.resize(2);
        line_marker.points[0].x = pose1[0];
        line_marker.points[0].y = pose1[1];
        line_marker.points[0].z = pose1[2];

        line_marker.points[1].x = pose2[0];
        line_marker.points[1].y = pose2[1];
        line_marker.points[1].z = pose2[2];

        markers.markers.push_back(line_marker);
    }
}

bool SLAM3DSystem::saveData()
{
    if(_optimizer.save((save_path+"graph.g2o").c_str()) == 1 && save_path != ""){
        std::cout << "-- saved g2o"  << std::endl;
        std::cout << "-- saving pcd..." << std::endl;
        for(int i = 0; i<vertexMap.size(); i++)
        {
            std::string num = std::to_string(i);
            std::string fname = save_path+num+".pcd";
            pcl::io::savePCDFileASCII(fname, vertexMap[i]->scan_data);
        }
        std::cout << "-- done" << std::endl;
        return true;
    }
    else
    {
        std::cout << "-- Fail to saved g2o!! (path : " << save_path.c_str() << ")" << std::endl;
        return false;
    }

}