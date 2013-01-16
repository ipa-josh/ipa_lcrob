#include <stdio.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"
#include "ipa_odroidx_ultrasonic_interface/ExRange.h"
#include "ipa_odroidx_ultrasonic_interface/ExRangeArray.h"
#include "sensor_msgs/Range.h"

#define PINGING_SENSOR 	-1
#define SENSOR_NOT_USED 255
#define PINGING_AND_LISTENING_SENSOR -2

#define MAX_SENSORS 14

#define MAX_RANGE 10 //in meters
#define MIN_RANGE 0.5 // in meters, not sure

#define SPEED_OF_SOUND 343.2 // meters per second taken from http://en.wikipedia.org/wiki/Speed_of_sound
#define TIMER_PRESCALER 8
#define F_CPU 2304000

enum ACK_RECEIVED {NO, MAYBE, YES}; // Three states of ACK 

/*generateConfigVector stores the pinging sensors, to which pinging sensor is a listening
 * sensor bound to in a cycle and which sensors are not used at all.
 * In the output, the pinging sensors are represented by PINGING_SENSOR, location of
 * each listening sensor holds the address of its corresponding pinging sensor
 * and the sensors not used at all are represented by SENSOR_NOT_USED.
 */
std::vector <std::vector<int> > generateConfigVector(XmlRpc::XmlRpcValue config_list)
{
	std::vector<std::vector<int> > config;
	XmlRpc::XmlRpcValue current_cycle;
	ROS_ASSERT(config_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i = 0; i < config_list.size(); i++)
	{
		ROS_ASSERT(config_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
		std::vector<int> cycle_vector; //For holding configuration for each cycle.
		cycle_vector.resize(MAX_SENSORS);
		for (int j=0; j<MAX_SENSORS; j++)
			cycle_vector[j]=SENSOR_NOT_USED;

		//better to keep it dynamic
		for (int j = 0; j<MAX_SENSORS; j++){
			char str_index[3] = {'\0','\0', '\0'}; // for storing int to hex conv. 
			sprintf(str_index,"%d", j); // Unsure if yamlcpp parser reads hexadecimal values
			if(config_list[i].hasMember(str_index))
			{
				cycle_vector[j]=PINGING_SENSOR; //Set -1 for pinging sensor
				current_cycle = config_list[i][str_index];
				ROS_ASSERT(current_cycle.getType() == XmlRpc::XmlRpcValue::TypeArray);
				ROS_ASSERT(current_cycle.size()>0);
				//Assign address of pinging sensor to each listening sensor
				for (int k = 0; k<current_cycle.size(); k++){
					cycle_vector[static_cast<int>(current_cycle[k])]=((static_cast<int>(current_cycle[k])!=j)?j:PINGING_AND_LISTENING_SENSOR);
				}
			}
		}
		config.push_back(cycle_vector);
	}
	return config;
}
float get_time(int timer_value)
{
	return (((float)(TIMER_PRESCALER)/(float)(F_CPU)) * timer_value);
}
float get_distance(float fall_time, float base_time)
{
	return ((fall_time-base_time) * SPEED_OF_SOUND); // Not to be divided by 2
}

/* Generates the config string once given the configuration vector generated by generateConfigVector
 */
int generateConfigString(std::vector< std::vector<int> >config_vector,unsigned char * config_string)
{
	unsigned char temp_config_string[100];
	int config_string_length=config_vector.size()*2+1; //setting up length of config string

	if(config_vector.size()==0)
		return 0;
	temp_config_string[0]=(unsigned int)(config_vector.size()) & 0xff;
	int count=1;
	for (std::vector< std::vector<int> >::iterator list_it = config_vector.begin(); list_it != config_vector.end(); list_it++)
	{
		int temp_mask=0;
		if((*list_it).size()==0)
			return 0;
		//better to keep it dynamic
		for (int i=0; i<MAX_SENSORS; i++)
		{
			if(((*list_it)[i]==PINGING_SENSOR) || ((*list_it)[i]==PINGING_AND_LISTENING_SENSOR))
			{
				if (i<7)
					temp_mask|=(1<<i); //For first port
				else
					temp_mask|=(1<<(i+1)); // For second port
			}
		}
		temp_config_string[count++] = (temp_mask & 0xff00) >> 8;
		temp_config_string[count++] = (temp_mask & 0xff);
	}
	memcpy(config_string, temp_config_string, config_string_length); // copying contents of temp config onto config string before return.
	return config_string_length;
}

ipa_odroidx_ultrasonic_interface::ExRange setupExRangeMeasurement(int sensor_address)
{
	std::ostringstream ss_sensor;
	ipa_odroidx_ultrasonic_interface::ExRange temp_range;
	temp_range.measurement.radiation_type = sensor_msgs::Range::ULTRASOUND;
	temp_range.measurement.min_range = MIN_RANGE;
	temp_range.measurement.max_range = MAX_RANGE;
	ss_sensor<<"us"<<sensor_address;
	temp_range.measurement.header.frame_id = ss_sensor.str();
	return temp_range;
}

/*Gernerate ExRangeArray from the result gathered from the sensor
 */
ipa_odroidx_ultrasonic_interface::ExRangeArray generateExRangeArray(std::map<int, std::vector< int > > input_map, std::vector<std::vector<int> > config_vector, int sequence_number )
{
	ipa_odroidx_ultrasonic_interface::ExRangeArray measurement_array;
	//Traverse over all sensors to check if they are used in this config
	//and calculate relative distance from the assigned pinging sensor
	//in case they are.
 	for(unsigned int i = 0; i< config_vector[sequence_number].size(); i++)
	{
		if(config_vector[sequence_number][i]!=SENSOR_NOT_USED)
		{
			if(config_vector[sequence_number][i]==PINGING_AND_LISTENING_SENSOR)
			{
				if(input_map.count(i)) //Received any reading 
				{
					if(input_map[i].size()>=2) // Considering the first reading after the ping.
					{
						// Setting up Range msg first.
						ipa_odroidx_ultrasonic_interface::ExRange temp_range = setupExRangeMeasurement(i);
						temp_range.measurement.range = get_distance(get_time(input_map[i][1]), get_time(input_map[i][0]));
						temp_range.sender_ch = i;
						temp_range.receiver_ch = i;
						measurement_array.measurements.push_back(temp_range);
					}
				}
			}
			else //Is a listening sensor
			{
				if(input_map.count(i)) //Received any reading 
				{
					if(input_map.count(config_vector[sequence_number][i])) // To check if corresponding pinging sensor received any readings
					{
						if(input_map[config_vector[sequence_number][i]].size()>0) // To check if corresponding pinging sensor has base value
						{
							ipa_odroidx_ultrasonic_interface::ExRange temp_range = setupExRangeMeasurement(i);
							temp_range.measurement.range = get_distance(get_time(input_map[i][0]), get_time(input_map[config_vector[sequence_number][i]][0]));
							temp_range.sender_ch = config_vector[sequence_number][i];
							temp_range.receiver_ch = i;
							measurement_array.measurements.push_back(temp_range);
						}
					}
				}
			}
		}
	}
	return measurement_array;
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "us_driver");
	ros::NodeHandle nh_;
	XmlRpc::XmlRpcValue config_list_;
	std::map<int, std::vector< int > > input_map_;
	ros::init(argc, argv, "us_driver");
	if(!nh_.hasParam("us_driver/configurations"))
	{
		ROS_ERROR("Sensor configurations not found.");
		return(EXIT_FAILURE);
	}
	ROS_INFO("configurations found.");
	ros::Publisher pub = nh_.advertise<ipa_odroidx_ultrasonic_interface::ExRangeArray>("us_reading", 5);

	CommPortDriver * comm_port_ = new UARTDriver("/dev/ttyUSB0");

	nh_.getParam("us_driver/configurations", config_list_);
	ROS_ASSERT(config_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);

	std::vector <std::vector<int> > config_vector_ = generateConfigVector(config_list_);
	ROS_INFO("%d", (int)config_vector_.size());
	for (int i=0; i<MAX_SENSORS; i++){
		ROS_INFO("%d", config_vector_[0][i]);
	}

	unsigned char config_string_[100]; // Must be declared before use.
	int config_string_length_ = 0;

	config_string_length_ = generateConfigString(config_vector_, config_string_);
	ROS_INFO("config_string_length_ = %d", config_string_length_);
	for (int i= 0; i<config_string_length_; i++)
	{
		ROS_INFO("0x%02x", config_string_[i]);
	}
	ROS_INFO(" ");
	ROS_ASSERT(comm_port_->writeBytes(config_string_, config_string_length_)==config_string_length_);
	unsigned char * buffer_ = new unsigned char[100];

	ACK_RECEIVED ack_received_ = NO;
	bool ack_stage_2 = false;
	int sequence_number= -1;
	while(ros::ok())
	{
		comm_port_->readBytes(buffer_, 1);
	//	ROS_INFO("%02x", buffer_[0]);
		if(ack_received_ == NO)
		{
			ROS_INFO("ACK NO");
			if(buffer_[0] == 0x12)
				ack_received_ = MAYBE;
		}
		else if(ack_received_ == MAYBE)
		{
			ROS_INFO("ACK MAYBE");
			if(ack_stage_2 == false)
			{
				if (buffer_[0] == 0x00)
				{
					ROS_INFO("ack_stage_2 = true");
					ack_stage_2 = true;
					sequence_number = 0;
				}
				else
					ack_received_ = NO;
			}
			else if ((buffer_[0] & 0xf0) == 0xd0)
			{
				ack_received_ = YES;
			}
		}
		if(ack_received_==YES) //ack reception confirmed.
		{
//			ROS_INFO("ACK YES");
//			ROS_INFO("0x%02x", buffer_[0]);
			if(sequence_number == -1) //previous cycle complete.
			{
				sequence_number = buffer_[0];
				ROS_INFO("sequence_number: 0x%x", sequence_number);
			}
			else
			{
				for (int i=0; i< MAX_SENSORS; i++)
				{
					if (i>0){
						comm_port_->readBytes(buffer_,1);
					}
					int total_sensor_readings = (buffer_[0] & 0x0f) ;
//					ROS_INFO("sensor address: %x, total_sensor_readings: %d",(buffer_[0] & 0xf0) >> 4, total_sensor_readings);
					if(total_sensor_readings > 0)
					{
						int current_sensor_address = (buffer_[0] & 0xf0) >> 4;
						for (int j = 0; j < total_sensor_readings; j++)
						{
							int temp_reading = 0;
							comm_port_->readBytes(buffer_, 1);
							temp_reading= (buffer_[0]<<8 & 0xff00);
							comm_port_->readBytes(buffer_, 1);
							temp_reading |= (buffer_[0] & 0xff);
							input_map_[current_sensor_address].push_back(temp_reading & 0xffff);
//							ROS_INFO("Readings read: %d", input_map_[current_sensor_address].size());
						}
					}
				}
				for (std::map<int, std::vector<int> >::iterator map_it = input_map_.begin(); map_it != input_map_.end(); map_it++)
				{
					ROS_INFO("Sensor address: %d", map_it->first);
					ROS_INFO("----");
					for (std::vector<int>::iterator reading_list_it = (*map_it).second.begin(); reading_list_it != (*map_it).second.end(); reading_list_it++)
					{
						ROS_INFO("0x%04x", (*reading_list_it) & 0xffff);
					}
					ROS_INFO("----");
				}
				
				ipa_odroidx_ultrasonic_interface::ExRangeArray ex_range_array = generateExRangeArray(input_map_, config_vector_, sequence_number);
				ROS_INFO("Printing ExRangeArray");
				ROS_INFO("----------");
				for (unsigned int i = 0; i<ex_range_array.measurements.size(); i++)
				{
					ROS_INFO("Sender: %d, Receiver: %d, Range: %f", ex_range_array.measurements[i].sender_ch, ex_range_array.measurements[i].receiver_ch, ex_range_array.measurements[i].measurement.range);
				}
				ROS_INFO("----------");
				pub.publish(ex_range_array);
				//After one complete cycle has been processed.
				input_map_.clear();
				sequence_number = -1;
			}
		}
	}
	delete comm_port_;
	return 0;
}
