///////////////////////////////////////////////////////////////////////////////
// multimux is a ROS topic multiplexer for sets of inputs.  Multiple sets of
// input topics are subscribed to, and one of the sets is passed through to the
// output topics.
// A service is provided to select between the outputs.
//
// Copyright (C) 2013, Willow Garage
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


#include <cstdio>
#include <vector>
#include <list>
#include "ros/console.h"
#include "std_msgs/String.h"
#include "topic_tools/MuxSelect.h"
#include "topic_tools/MuxAdd.h"
#include "topic_tools/MuxList.h"
#include "topic_tools/MuxDelete.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::list;
using namespace topic_tools;

////////////////////////////////////////////////////////////////////////////////
// Class SingleMux
////////////////////////////////////////////////////////////////////////////////

class SingleMux
{
public:
  const static string NO_PREFIX;

private:
  ros::NodeHandle *node_;
  bool lazy_;
  bool advertised_;
  string output_topic_;
  ros::Publisher pub_;

  struct sub_info_t
  {
    string prefix;
    string topic_name; // fully-resolved topic name
    ros::Subscriber *sub;
    ShapeShifter* msg;
  };

  list<struct sub_info_t> subs_;
  list<struct sub_info_t>::iterator selected_;

public:
  SingleMux( ros::NodeHandle* node, bool lazy, const vector<string>& prefixes, string output_topic )
    : node_( node )
    , lazy_( lazy )
    , advertised_( false )
    , subs_()
    , selected_( subs_.end() )
  {
    ROS_DEBUG("multimux: SingleMux( lazy = %s, output_topic = %s )", (lazy?"true":"false"), output_topic.c_str());
    output_topic_ = output_topic;

    for( size_t i = 0; i < prefixes.size(); i++ )
    {
      struct sub_info_t sub_info;
      sub_info.prefix = prefixes[ i ];
      sub_info.msg = new ShapeShifter;
      sub_info.topic_name = ros::names::resolve( sub_info.prefix + "/" + output_topic );
      sub_info.sub = new ros::Subscriber( node_->subscribe<ShapeShifter>( sub_info.topic_name, 10,
                                                                          boost::bind( &SingleMux::in_cb, this, _1, sub_info.msg )));

      subs_.push_back(sub_info);
    }
  }

public:
  ~SingleMux()
  {
    for (list<struct sub_info_t>::iterator it = subs_.begin();
         it != subs_.end();
         ++it)
    {
      if (it->sub)
        it->sub->shutdown();
      delete it->sub;
      delete it->msg;
    }

    subs_.clear();
  }

private:
  void conn_cb( const ros::SingleSubscriberPublisher& )
  {
    ROS_DEBUG("multimux: conn_cb for output_topic = %s", output_topic_.c_str());

    // If we're in lazy subscribe mode, and the first subscriber just
    // connected, then subscribe
    if(lazy_ && selected_ != subs_.end() && !selected_->sub)
    {
      ROS_DEBUG("mux %s lazy mode; resubscribing to %s", output_topic_.c_str(), selected_->topic_name.c_str());
      selected_->sub = new ros::Subscriber( node_->subscribe<ShapeShifter>( selected_->topic_name, 10,
                                                                            boost::bind( &SingleMux::in_cb, this, _1, selected_->msg )));
    }
  }

public:
  bool select_prefix( string new_prefix )
  {
    ROS_DEBUG("multimux: select_prefix for output_topic = %s, prefix = %s", output_topic_.c_str(), new_prefix.c_str());
    bool ret = false;
    if (selected_ != subs_.end()) {

      // Unsubscribe to old topic if lazy
      if (lazy_) {
        ROS_DEBUG("mux %s Unsubscribing to %s, lazy", output_topic_.c_str(), selected_->topic_name.c_str());
        if (selected_->sub)
          selected_->sub->shutdown();
        delete selected_->sub;
        selected_->sub = NULL;
      }
    }

    // see if it's the magical '__none' topic, in which case we open the circuit
    if (new_prefix == NO_PREFIX)
    {
      ROS_INFO("mux %s selected to no input.", output_topic_.c_str());

      selected_ = subs_.end();
      ret = true;
    }
    else
    {
      ROS_INFO("trying to switch mux %s to %s", output_topic_.c_str(), new_prefix.c_str());
      // spin through our vector of inputs and find this guy
      for (list<struct sub_info_t>::iterator it = subs_.begin();
           it != subs_.end();
           ++it)
      {
        if (it->prefix == new_prefix)
        {
          selected_ = it;
          ROS_INFO("mux %s selected input prefix: [%s]", output_topic_.c_str(), it->prefix.c_str());
          ret = true;
        
          if (!selected_->sub && (!advertised_ || (advertised_ && pub_.getNumSubscribers()))) {
            selected_->sub = new ros::Subscriber( node_->subscribe<ShapeShifter>( selected_->topic_name, 10,
                                                                                  boost::bind( &SingleMux::in_cb, this, _1, selected_->msg )));
          }
        }
      }
    }
  
    return ret;
  }

private:
  void in_cb( const boost::shared_ptr<ShapeShifter const>& msg,
              ShapeShifter* s )
  {
    ROS_DEBUG("multimux: in_cb for output_topic = %s", output_topic_.c_str());
    if (!advertised_)
    {
      ROS_INFO("mux %s advertising", output_topic_.c_str());
      pub_ = msg->advertise(*node_, output_topic_, 10, false, boost::bind( &SingleMux::conn_cb, this, _1 ));
      advertised_ = true;
    
      // If lazy, unregister from all but the selected topic
      if (lazy_) {
        for( list<struct sub_info_t>::iterator it = subs_.begin(); it != subs_.end(); ++it ) {
          if( it != selected_ ) {
            ROS_INFO("mux %s Unregistering from %s", output_topic_.c_str(), it->topic_name.c_str());
            if (it->sub)
              it->sub->shutdown();
            delete it->sub;
            it->sub = NULL;
          }
        }
      }
    }
  
    if (s != selected_->msg)
      return;
  
    // If we're in lazy subscribe mode, and nobody's listening, then unsubscribe
    if (lazy_ && !pub_.getNumSubscribers() && selected_ != subs_.end()) {
      ROS_INFO("mux %s lazy mode; unsubscribing", output_topic_.c_str());
      selected_->sub->shutdown();
      delete selected_->sub;
      selected_->sub = NULL;
    }
    else
      pub_.publish(msg);
  }

public:
  bool add_prefix( string new_prefix )
  {
    struct sub_info_t sub_info;
    sub_info.prefix = new_prefix;
    sub_info.msg = new ShapeShifter;
    sub_info.topic_name = ros::names::resolve( new_prefix + "/" + output_topic_ );
    try
    {
      if (lazy_)
        sub_info.sub = NULL;
      else
        sub_info.sub = new ros::Subscriber( node_->subscribe<ShapeShifter>( sub_info.topic_name, 10,
                                                                            boost::bind( &SingleMux::in_cb, this, _1, sub_info.msg )));
    }
    catch(ros::InvalidNameException& e)
    {
      ROS_WARN("failed to add topic %s to mux %s, because it's an invalid name: %s",
               sub_info.topic_name.c_str(), output_topic_.c_str(), e.what());
      delete sub_info.msg;
      return false;
    }
    subs_.push_back(sub_info);
    return true;
  }

public:
  void delete_prefix( string old_prefix )
  {
    for (list<struct sub_info_t>::iterator it = subs_.begin();
         it != subs_.end();
         ++it)
    {
      if( it->prefix == old_prefix )
      {
        if( it->sub )
        {
          it->sub->shutdown();
        }
        delete it->sub;
        delete it->msg;
        subs_.erase(it);
        return;
      }
    }
  }
};

const string SingleMux::NO_PREFIX = "__none";

////////////////////////////////////////////////////////////////////////////////
// Global data
////////////////////////////////////////////////////////////////////////////////

static string g_selected_prefix = SingleMux::NO_PREFIX;
ros::Publisher g_pub_selected;
static vector<SingleMux*> g_muxes;
static vector<string> g_prefixes;

////////////////////////////////////////////////////////////////////////////////
// Service callbacks
////////////////////////////////////////////////////////////////////////////////

bool select_prefix_cb( topic_tools::MuxSelect::Request  &req,
                       topic_tools::MuxSelect::Response &res )
{
  bool ret = true;
  string new_prefix = req.topic;

  res.prev_topic = g_selected_prefix;

  bool found = false;
  if( new_prefix == SingleMux::NO_PREFIX )
  {
    found = true;
  }
  else
  {
    for( size_t i = 0; i < g_prefixes.size(); i++ )
    {
      if( g_prefixes[ i ] == new_prefix )
      {
        found = true;
        break;
      }
    }
  }
  if( !found )
  {
    ROS_WARN("failed to select prefix %s, because it is not listed.", new_prefix.c_str());
    return false;
  }

  for( size_t i = 0; i < g_muxes.size(); i++ )
  {
    ret &= g_muxes[ i ]->select_prefix( new_prefix );
  }

  if(ret)
  {
    std_msgs::String t;
    t.data = req.topic;
    g_pub_selected.publish(t);
  }

  return ret;
}

bool list_prefix_cb( topic_tools::MuxList::Request& req,
                     topic_tools::MuxList::Response& res)
{
  for( size_t i = 0; i < g_prefixes.size(); i++ )
  {
    res.topics.push_back( g_prefixes[ i ] );
  }

  return true;
}

bool add_prefix_cb(topic_tools::MuxAdd::Request& req,
                   topic_tools::MuxAdd::Response& res)
{
  string new_prefix = req.topic;

  // Check that it's not already in our list
  ROS_INFO("trying to add %s to mux", new_prefix.c_str());
  
  // Can't add the __none topic
  if( new_prefix == SingleMux::NO_PREFIX )
  {
    ROS_WARN( "failed to add prefix %s to mux, because it's reserved for special use",
              new_prefix.c_str() );
    return false;
  }

  for( size_t i = 0; i < g_prefixes.size(); i++ )
  {
    if( new_prefix == g_prefixes[ i ])
    {
      ROS_WARN( "tried to add a prefix that mux was already using: [%s]", new_prefix.c_str() );
      return false;
    }
  }

  for( size_t i = 0; i < g_muxes.size(); i++ )
  {
    g_muxes[ i ]->add_prefix( new_prefix );
  }
  g_prefixes.push_back( new_prefix );

  ROS_INFO("added %s to mux", new_prefix.c_str());

  return true;
}

bool delete_prefix_cb(topic_tools::MuxDelete::Request& req,
                      topic_tools::MuxDelete::Response& res)
{
  string old_prefix = req.topic;

  // Check that it's in our list
  ROS_INFO("trying to delete %s from mux", old_prefix.c_str());

  for( size_t prefix_index = 0; prefix_index < g_prefixes.size(); prefix_index++ )
  {
    if( old_prefix == g_prefixes[ prefix_index ])
    {
      // Can't delete the currently selected input, #2863
      if( g_prefixes[ prefix_index ] == g_selected_prefix )
      {
        ROS_WARN("tried to delete currently selected prefix %s from mux", old_prefix.c_str());
        return false;
      }
      for( size_t mux_index = 0; mux_index < g_muxes.size(); mux_index++ )
      {
        g_muxes[ mux_index ]->delete_prefix( old_prefix );
      }
      ROS_INFO("deleted prefix %s from mux", old_prefix.c_str());
      return true;
    }
  }

  ROS_WARN("tried to delete non-listed prefix %s from mux", old_prefix.c_str());
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// main program
////////////////////////////////////////////////////////////////////////////////

bool read_arguments( const vector<string>& args_in, vector<string>& topics_out, vector<string>& input_prefixes_out)
{
  bool reading_topics = false;
  bool reading_prefixes = false;
  topics_out.clear();
  input_prefixes_out.clear();

  for( size_t i = 1; i < args_in.size(); i++ )
  {
    if( args_in[ i ] == "--topics" )
    {
      reading_topics = true;
      reading_prefixes = false;
    }
    else if( args_in[ i ] == "--input_prefixes" )
    {
      reading_topics = false;
      reading_prefixes = true;
    }
    else if( args_in[ i ].substr( 0, 2 ) == "--" )
    {
      printf( "\nUnsupported command line option '%s'.\n", args_in[ i ].c_str() );
      return false;
    }
    else if( !reading_topics && !reading_prefixes )
    {
      printf( "\nFirst argument must be --topics or --input_prefixes, was '%s'.\n", args_in[ i ].c_str() );
      return false;
    }
    else if( reading_topics )
    {
      topics_out.push_back( args_in[ i ]);
    }
    else
    {
      input_prefixes_out.push_back( args_in[ i ]);
    }
  }
  if( topics_out.size() == 0 )
  {
    printf( "\nCannot run with empty topic list.\n" );
    return false;
  }
  if( input_prefixes_out.size() == 0 )
  {
    printf( "\nCannot run with empty prefix list.\n" );
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  vector<string> args;
  ros::removeROSArgs(argc, (const char**)argv, args);

  vector<string> topics;
  if( !read_arguments( args, topics, g_prefixes ))
  {
    printf("\nusage: multimux --topics IN_TOPIC1 [IN_TOPIC2 [...]] --input_prefixes PREFIX1 [PREFIX2 [...]]\n\n");
    return 1;
  }
  ros::init(argc, argv, "multimux", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  // Put our API into the "mux" namespace, which the user should usually remap
  ros::NodeHandle mux_nh("mux"), pnh("~");
  bool lazy;
  pnh.getParam( "lazy", lazy );
  // testing...
//  lazy = false;

  // Latched publisher for selected input topic name
  g_pub_selected = mux_nh.advertise<std_msgs::String>(string("selected"), 1, true);

  for( size_t topics_index = 0; topics_index < topics.size(); topics_index++ )
  {
    g_muxes.push_back( new SingleMux( &n, lazy, g_prefixes, topics[ topics_index ]));
  }

  g_selected_prefix = g_prefixes[ 0 ]; // select first prefix to start
  std_msgs::String t;
  t.data = g_selected_prefix;
  g_pub_selected.publish(t);

  // New service
  ros::ServiceServer ss_select = mux_nh.advertiseService(string("select"), select_prefix_cb);
  ros::ServiceServer ss_add = mux_nh.advertiseService(string("add"), add_prefix_cb);
  ros::ServiceServer ss_list = mux_nh.advertiseService(string("list"), list_prefix_cb);
  ros::ServiceServer ss_del = mux_nh.advertiseService(string("delete"), delete_prefix_cb);
  ros::spin();

  for( size_t muxes_index = 0; muxes_index < g_muxes.size(); muxes_index++ )
  {
    delete g_muxes[ muxes_index ];
  }
  g_muxes.clear();

  return 0;
}

