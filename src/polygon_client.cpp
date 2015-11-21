#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_tools.h>
#include <tf/transform_listener.h>
#include <ros/wall_timer.h>

#include <visualization_msgs/Marker.h>

namespace rviz_semap_plugin{

/**
 * @brief Client for 
 */
class PolygonClient{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber point_;
    ros::Publisher point_viz_pub_;
    ros::Publisher polygon_pub_;
    ros::WallTimer point_viz_timer_;
    geometry_msgs::PolygonStamped input_;

    double proximity_;

    bool waiting_for_center_;

    geometry_msgs::Point toPoint(geometry_msgs::Point32 pt)
    {
       geometry_msgs::Point point;
       point.x = pt.x;
       point.y = pt.y;
       point.z = pt.z;
       return point;
    }

    geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt)
    {
       geometry_msgs::Point32 point32;
       point32.x = pt.x;
       point32.y = pt.y;
       point32.z = pt.z;
       return point32;
    }

    /**
     * @brief Publish markers for visualization of points for boundary polygon.
     */
    void vizPubCb(){
        visualization_msgs::Marker points, line_strip;

        points.header = line_strip.header = input_.header;
        points.ns = line_strip.ns = "polygon_points";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        if(!input_.polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = 0.2;
            line_strip.scale.x = 0.05;

            points.color.a = 1.0;
            points.color.b = 1.0;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;

            points.points.push_back(toPoint(input_.polygon.points.front()));

            for(unsigned int i = 0; i < input_.polygon.points.size(); i++){
                line_strip.points.push_back(toPoint(input_.polygon.points[i]));
            }

            if(waiting_for_center_){
                line_strip.points.push_back(toPoint(input_.polygon.points.front()));
            }
        }else{

            points.action = line_strip.action = visualization_msgs::Marker::DELETE;

        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);
    }

    /**
     * @brief Build boundary polygon from points received through rviz gui.
     * @param point Received point from rviz
     */
    void pointCb(const geometry_msgs::PointStampedConstPtr& point){
        if(waiting_for_center_){
            //flag is set so this is the last point of boundary polygon, i.e. center

            if(!pointInPolygon(point->point,input_.polygon)){
                ROS_ERROR("Center not inside polygon, restarting");
            }else{
                polygon_pub_.publish(input_);
            }
            waiting_for_center_ = false;
            input_.polygon.points.clear();

        }else if(input_.polygon.points.empty()){
            //first control point, so initialize header of boundary polygon

            input_.header = point->header;
            input_.polygon.points.push_back(toPoint32(point->point));

        }else if(input_.header.frame_id != point->header.frame_id){
            ROS_ERROR("Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();

        }else if(input_.polygon.points.size() > 1 && pointsAdjacent(toPoint(input_.polygon.points.front()), point->point, proximity_)){
            //check if last boundary point, i.e. adjacent to first point

            if(input_.polygon.points.size() < 3){
                ROS_ERROR("Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }else{
                waiting_for_center_ = true;
                ROS_WARN("Please select an initial point for exploration inside the polygon");
            }

        }else{
            //otherwise, must be a regular point inside boundary polygon
            input_.polygon.points.push_back(toPoint32(point->point));
            input_.header.stamp = ros::Time::now();
        }
        vizPubCb();
    }

public:

    /**
     * @brief Constructor for the client.
     */
    PolygonClient() :
        nh_(),
        private_nh_("~"),
        waiting_for_center_(false)
    {
        input_.header.frame_id = "map";
        private_nh_.param<double>("proximity", proximity_, 0.2);
        point_ = nh_.subscribe("/clicked_point",10,&PolygonClient::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("polygon_marker", 10);
        polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 10);
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon_client");

    rviz_semap_plugin::PolygonClient client;
    ros::spin();
    return 0;
}
