#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

class IntMarkerAPI
{
public:
    void estTopose(const double *est, geometry_msgs::Pose &pose);

    void poseToest(const geometry_msgs::Pose pose, double *est);

    double rand( double min, double max );

    Marker makeBox( InteractiveMarker &msg );

    Marker makeLine( const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &msg );

    InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

    InteractiveMarkerControl& makeLineControl( const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &msg );

    void make6DofMarker(const std::string name, bool fixed, unsigned int interaction_mode, const double *position, bool show_6dof, InteractiveMarker &int_marker );

    void makeRandomDofMarker( const tf::Vector3& position, InteractiveMarker &int_marker );

    void makeViewFacingMarker( const tf::Vector3& position, InteractiveMarker &int_marker );

    void makeQuadrocopterMarker( const tf::Vector3& position, InteractiveMarker &int_marker );

    void makeChessPieceMarker( const tf::Vector3& position, InteractiveMarker &int_marker );

    void makePanTiltMarker( const tf::Vector3& position, InteractiveMarker &int_marker );

    void makeButtonLineMarker(const std::string name, const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &int_marker );
    void makeButtonBoxMarker(const std::string name, const tf::Vector3& position , InteractiveMarker &int_marker );

    void makeMovingMarker( const tf::Vector3& position, InteractiveMarker &int_marker );
};
