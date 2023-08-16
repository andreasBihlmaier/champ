/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <champ_msgs/msg/contacts_stamped.hpp>
#include <champ/utils/urdf_loader.h>

#include <gz/transport.hh>
#include <ignition/msgs.hh>

std::vector<std::string> split(const std::string& input, const std::string& delim) {
	std::vector<std::string> results;
	size_t start = 0;
	size_t end = input.find(delim);
	while (end != std::string::npos) {
		results.push_back(input.substr(start, end - start));
		start = end + delim.length();
		end = input.find(delim, start);
	}
	results.push_back(input.substr(start, end));
	return results;
}

class ContactSensor: public rclcpp::Node
{
	bool foot_contacts_[4];
	std::vector<std::string> foot_links_;
	rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
  gz::transport::Node node;
	std::mutex foot_contacts_mutex_;

	public:
		ContactSensor():
			foot_contacts_ {false,false,false,false},
			Node("contacts_sensor",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true))
		{
			std::vector<std::string> gz_topics = {
				"/world/empty/model/mini-pupper/link/lb3/sensor/lb_sensor_contact/contact",
				"/world/empty/model/mini-pupper/link/lf3/sensor/lf_sensor_contact/contact",
				"/world/empty/model/mini-pupper/link/rb3/sensor/rb_sensor_contact/contact",
				"/world/empty/model/mini-pupper/link/rf3/sensor/rf_sensor_contact/contact"
			};
			std::vector<std::string> joint_names;

			joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
			foot_links_.push_back(joint_names[2]);
			foot_links_.push_back(joint_names[6]);
			foot_links_.push_back(joint_names[10]);
			foot_links_.push_back(joint_names[14]);
			RCLCPP_INFO_STREAM(get_logger(), "Foot links: " << foot_links_[0] << " " << foot_links_[1] << " " << foot_links_[2] << " " << foot_links_[3]);

			contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

			for (const auto& topic : gz_topics) {
				if (!node.Subscribe(topic, &ContactSensor::gz_cb, this))
				{
					RCLCPP_ERROR_STREAM(get_logger(), "Error subscribing to topic: " << topic);
				}
			}
		}

		void gz_cb(const gz::msgs::Contacts &_msg)
		{

			for (int i = 0; i < _msg.contact_size(); ++i) 
			{
				std::string collision = _msg.contact(i).collision1().name();
				std::vector<std::string> results = split(collision, "::");

				for(size_t j = 0; j < 4; j++)
				{
					if(foot_links_[j] == results[1])
					{
						const std::lock_guard<std::mutex> lock(foot_contacts_mutex_);
						foot_contacts_[j] = true;
						break;
					}
				}
			}

		}

		void publishContacts()
		{
			champ_msgs::msg::ContactsStamped contacts_msg;
			contacts_msg.header.stamp = this->get_clock()->now();
			contacts_msg.contacts.resize(4);

			{
				const std::lock_guard<std::mutex> lock(foot_contacts_mutex_);
				for(size_t i = 0; i < 4; i++)
				{
					contacts_msg.contacts[i] = foot_contacts_[i];
				}
				for(size_t i = 0; i < 4; i++)
				{
					foot_contacts_[i] = false;
				}
			}

			contacts_publisher_->publish(contacts_msg);
		}
};

void exitHandler(int sig)
{
	rclcpp::shutdown();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ContactSensor>();
	// This assumes the gz contact topics publish (much) more frequently than 50 Hz
	rclcpp::Rate loop_rate(50);

	while (rclcpp::ok())
	{
		node->publishContacts();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}