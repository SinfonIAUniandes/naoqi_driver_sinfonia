/*
 * Copyright 2015 Aldebaran
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

#ifndef BASE_PUBLISHER_HPP
#define BASE_PUBLISHER_HPP

#include <string>

/**
* ROS includes
*/
#include <ros/ros.h>

namespace alros
{
namespace publisher
{

// CRTP
template<class T>
class BasePublisher
{

public:
  BasePublisher( const std::string& topic ):
    topic_( topic ),
    is_initialized_( false )
  {}

  virtual ~BasePublisher() {}

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  virtual inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
      return pub_.getNumSubscribers() > 0;
  }

protected:
  std::string topic_;

  bool is_initialized_;

  /** Publisher */
  ros::Publisher pub_;
}; // class

} // publisher
} // alros

#endif
