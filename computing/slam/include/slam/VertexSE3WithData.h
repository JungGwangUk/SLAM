//
// Created by ub1404 on 16. 6. 18.
//

#ifndef INTEL_SLAM_VERTEXSE3WITHDATA_H
#define INTEL_SLAM_VERTEXSE3WITHDATA_H
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace g2o {
    class G2O_TYPES_SLAM3D_API VertexSE3WithData : public g2o::VertexSE3{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual ~VertexSE3WithData(){}

        virtual bool read(std::istream& is)
        {
            g2o::VertexSE3::read(is);
            return true;
        };
        virtual bool write(std::ostream& os) const
        {
            g2o::VertexSE3::write(os);
            return os.good();
        };

        pcl::PointCloud<pcl::PointXYZI> scan_data;
    };
}

#endif //INTEL_SLAM_VERTEXSE3WITHDATA_H