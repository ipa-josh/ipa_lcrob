#!/bin/bash

#rosrun topic_proxy client 192.168.1.102 &
rosservice call /request_message '{ topic: tf, interval: 50000000, compressed: true }'
rosservice call /request_message '{ topic: scan, interval: 100000000, compressed: true }'
rosservice call /request_message '{ topic: map_karto, interval: 5000000000, compressed: true }'
rosservice call /request_message '{ topic: coverage_map, interval: 2000000000, compressed: true }'
rosservice call /request_message '{ topic: dist_map, interval: 5000000000, compressed: true }'
rosservice call /request_message '{ topic: move_base_linear_simple/goal, interval: 100000000, compressed: true }'
rosservice call /request_message '{ topic: move_base/current_goal, interval: 100000000, compressed: true }'

#rosservice call /add_publisher '{ topic: move_base_linear_simple/goal }'
