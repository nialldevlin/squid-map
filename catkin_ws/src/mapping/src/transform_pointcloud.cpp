#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <cmath>
#include "quickselect.h"

ros::Publisher map_pub;
tf2_ros::Buffer tf2_buffer;
std::string map_frame;
std::string pointcloud_topic;

// in m
int map_x_size = 30;
int map_y_size = 30;
float cell_size = 0.1;

int min_vals = 5;
int x_size = map_x_size / cell_size + 0.5;
int y_size = map_y_size / cell_size + 0.5;
float z_min = INT_MAX;
float z_max = -INT_MAX;
nav_msgs::OccupancyGrid grid;
float est_floor_height = -1.5;
std::vector<float> z_vals(x_size * y_size, -INT_MAX);
float alpha = 0.25;


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Check if transform is available
    if (tf2_buffer.canTransform(map_frame, cloud_msg->header.frame_id, ros::Time(0)))
    {
        sensor_msgs::PointCloud2 transformed_cloud;
        try
        {
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped = tf2_buffer.lookupTransform(map_frame,
                cloud_msg->header.frame_id,
                ros::Time(0)
            );
            tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);

            // Iterate over the data in the PointCloud2 message
            // std::vector<float> sums(x_size * y_size, 0);
            // std::vector<std::vector<float>> z_vals(x_size * y_size);
            for(sensor_msgs::PointCloud2ConstIterator<float>
                iter_x(transformed_cloud, "x"),
                iter_y(transformed_cloud, "y"),
                iter_z(transformed_cloud, "z");
                iter_x != iter_x.end();
                ++iter_x, ++iter_y, ++iter_z)
            {
                float x = *iter_x;
                float y = *iter_y;
                float z = *iter_z;
                // x = x * 0.0254;
                // y = y * 0.0254;
                // z = z * 0.0254;
                // std::cout << x << " " << y << " " << z << "\n";

                if (x < map_x_size / 2 && x > -map_x_size / 2 &&
                    y < map_y_size / 2 && y > -map_y_size / 2)
                {
                    z_min = z < z_min ? z : z_min;
                    z_max = z > z_max ? z : z_max;

                    int x_coord = x / cell_size + x_size/2;
                    int y_coord = y / cell_size + y_size/2;
                    int i = x_size * y_coord + x_coord;

                    // std::cout << x_coord << " " << y_coord << " " << i << "\n";
                    // if (z > z_vals.at(i)) {
                    //     z_vals.at(i) = z;
                    // }
                    // sums[i] += z;
                    // z_vals[i].push_back(z);
                }
            }

            // for (int i = 0; i < x_size * y_size; i++) {
                // STDEV
                // int num_z = z_vals[i].size();
                // float mean = sums[i] / num_z;
                // float stdev = 0;
                // for (int j = 0; j < num_z; j++) {
                //     float add = z_vals[i][j] - mean;
                //     stdev += add * add;
                // }
                // stdev /= num_z;
                // stdev = sqrt(stdev);
                // grid.data.at(i) = 100 * (mean + stdev - z_min) / (z_max - z_min);

            // }

            // int c_min = quickselect(count, 0, count.size() - 1, 0);
            // int c_max = quickselect(count, 0, count.size() - 1, count.size() - 1);
            // for (size_t i = 0; i < count.size(); i++) {
            //     std::cout << z_vals[i].size() << std::endl;
            //     if (z_vals[i].size() > min_vals){
            //         // float z = getMedian(z_vals[i]);
            //         float sum = 0;
            //         for (size_t j = 0; j < z_vals[i].size(); j++) {
            //             sum += z_vals[i][j];
            //         }
            //         float mean = sum / z_vals.size();
            //         // float z = mean;
            //         // z = (z - z_min) / (z_max - z_min) * 100;
            //         int scaled_mean = 100 * (mean - z_min) / (z_max - z_min);
            //         // std::cout << scaled_mean << std::endl;
            //         grid.data.at(i) = scaled_mean;
            //         // std::cout << z << " ";
            //         // if (i % x_size == 0) {
            //         //     std::cout << "\n";
            //         // }
            //     }
            // }
           
            grid.header.stamp = ros::Time::now();

            map_pub.publish(grid);
        }
        catch (tf2::TransformException &ex)
        {
            std::cout << ex.what() << std::endl;
            ROS_WARN("%s", ex.what());
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_transformer_node");

    ros::NodeHandle nh;

    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "points");
    nh.param<std::string>("map_frame", map_frame, "map");

    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    ros::Subscriber pointcloud_sub;
    pointcloud_sub = nh.subscribe(pointcloud_topic, 10, callback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

    grid.header.frame_id = map_frame;
    grid.info.resolution = cell_size;
    grid.info.width = x_size;
    grid.info.height = y_size;
    grid.info.origin.position.x = -map_x_size/2;
    grid.info.origin.position.y = -map_y_size/2;
    grid.data.assign(x_size * y_size, -1);

    ros::spin();
    // delete[] z_vals;
    return 0;
}

