#include <slam/InteractiveMarkersAPI.h>


void IntMarkerAPI::estTopose(const double *est, geometry_msgs::Pose &pose)
{
    pose.position.x = est[0];
    pose.position.y = est[1];
    pose.position.z = est[2];
    pose.orientation.x = est[3];
    pose.orientation.y = est[4];
    pose.orientation.z = est[5];
    pose.orientation.w = est[6];
}

void IntMarkerAPI::poseToest(const geometry_msgs::Pose pose, double *est)
{
    est[0] = pose.position.x;
    est[1] = pose.position.y;
    est[2] = pose.position.z;
    est[3] = pose.orientation.x;
    est[4] = pose.orientation.y;
    est[5] = pose.orientation.z;
    est[6] = pose.orientation.w;
}

// %Tag(Box)%
Marker IntMarkerAPI::makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale*0.3;
    marker.scale.y = msg.scale*0.3;
    marker.scale.z = msg.scale*0.3;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    return marker;
}

Marker IntMarkerAPI::makeLine( const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::LINE_LIST;
    marker.scale.x = msg.scale;
    marker.scale.y = msg.scale;
    marker.scale.z = msg.scale;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.points.resize(2);
    marker.points[0].x = p1.x();
    marker.points[0].y = p1.y();
    marker.points[0].z = p1.z();

    marker.points[1].x = p2.x();
    marker.points[1].y = p2.y();
    marker.points[1].z = p2.z();

    return marker;
}

InteractiveMarkerControl& IntMarkerAPI::makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

InteractiveMarkerControl& IntMarkerAPI::makeLineControl( const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeLine(p1, p2, msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
// %EndTag(Box)%

double IntMarkerAPI::rand( double min, double max )
{
    double t = (double)std::rand() / (double)RAND_MAX;
    return min + t*(max-min);
}


// %Tag(6DOF)%
void IntMarkerAPI::make6DofMarker( const std::string name, bool fixed, unsigned int interaction_mode, const double *position, bool show_6dof, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    geometry_msgs::Pose pose;
    estTopose(position,pose);
    int_marker.pose = pose;
    int_marker.scale = 20;

    int_marker.name = name;
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
//        control.name = "move_z";
//        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }
}
// %EndTag(6DOF)%

// %Tag(RandomDof)%
void IntMarkerAPI::makeRandomDofMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "6dof_random_axes";
    int_marker.description = "6-DOF\n(Arbitrary Axes)";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    for ( int i=0; i<3; i++ )
    {
        control.orientation.w = rand(-1,1);
        control.orientation.x = rand(-1,1);
        control.orientation.y = rand(-1,1);
        control.orientation.z = rand(-1,1);
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }
}
// %EndTag(RandomDof)%


// %Tag(ViewFacing)%
void IntMarkerAPI::makeViewFacingMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "view_facing";
    int_marker.description = "View Facing 6-DOF";

    InteractiveMarkerControl control;

    // make a control that rotates around the view axis
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    control.name = "rotate";

    int_marker.controls.push_back(control);

    // create a box in the center which should not be view facing,
    // but move in the camera plane.
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.independent_marker_orientation = true;
    control.name = "move";

    control.markers.push_back( makeBox(int_marker) );
    control.always_visible = true;

    int_marker.controls.push_back(control);
}
// %EndTag(ViewFacing)%


// %Tag(Quadrocopter)%
void IntMarkerAPI::makeQuadrocopterMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "quadrocopter";
    int_marker.description = "Quadrocopter";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%
void IntMarkerAPI::makeChessPieceMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "chess_piece";
    int_marker.description = "Chess Piece\n(2D Move + Alignment)";

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    // make a box which also moves in the plane
    control.markers.push_back( makeBox(int_marker) );
    control.always_visible = true;
    int_marker.controls.push_back(control);
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%
void IntMarkerAPI::makePanTiltMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "pan_tilt";
    int_marker.description = "Pan / Tilt";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::INHERIT;
    int_marker.controls.push_back(control);
}
// %EndTag(PanTilt)%

// %Tag(Button)%
void IntMarkerAPI::makeButtonLineMarker(const std::string name, const tf::Vector3 p1, const tf::Vector3 p2, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    int_marker.scale = 1;

    int_marker.name = name;
    int_marker.description = "Button\n(Left Click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "button_control";

    Marker marker = makeLine( p1, p2, int_marker );
    control.markers.push_back( marker );
    control.always_visible = true;
    int_marker.controls.push_back(control);
}
void IntMarkerAPI::makeButtonBoxMarker(const std::string name, const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "hitlc_map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 5;

    int_marker.name = name;

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "button_control";

    Marker marker = makeBox( int_marker );
    control.markers.push_back( marker );
    control.always_visible = true;
    int_marker.controls.push_back(control);
}
// %EndTag(Button)%

// %Tag(Moving)%
void IntMarkerAPI::makeMovingMarker( const tf::Vector3& position, InteractiveMarker &int_marker )
{
    int_marker.header.frame_id = "moving_frame";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "moving";
    int_marker.description = "Marker Attached to a\nMoving Frame";

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers.push_back( makeBox(int_marker) );
    int_marker.controls.push_back(control);
}
// %EndTag(Moving)%
