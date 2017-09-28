#!/bin/bash

rostopic pub -1 /mode std_msgs/String "data: $1"
