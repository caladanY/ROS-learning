// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"


/* ---------------------------------------------------------------------------
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * ---------------------------------------------------------------------------
 */
 
 
   /**
    * This is the callback function that will get called when a new message 
    * has arrived on the chatter topic. The message is passed in a 
    * [boost shared_ptr](http://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm),
    * which means you can store it off if you want, without worrying about it 
    * getting deleted underneath you, and without copying the underlying data. 
    */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%


int main(int argc, char **argv)
{


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  /**
   * Subscribe to the chatter topic with the master. ROS will call the chatterCallback() 
   * function whenever a new message arrives. 
   *
   * NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on
   * to until you want to unsubscribe. When the Subscriber object is destructed, 
   * it will automatically unsubscribe from the chatter topic. 
   *
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   *There are versions of the NodeHandle::subscribe() function which allow you to
   * specify a class member function, or even anything callable by a Boost.Function object. 
   * [The roscpp overview contains more information.](http://wiki.ros.org/roscpp/Overview)
   * 
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   * In this case, if the queue reaches 1000 messages, we will start throwing away
   * old messages as new ones arrive. 
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%


  /**
   * ros::spin() enters a loop, calling message callbacks as fast as possible. 
   * Don't worry though, if there's nothing for it to do it won't use much CPU. 
   * ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() 
   * has been called, either by the default Ctrl-C handler, the master telling us 
   * to shutdown, or it being called manually. 
   *
   * With this version, all callbacks will be called from within this thread (the main one).  
   * ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   *
   * There are other ways of pumping callbacks, but we won't worry about those here. 
   * The roscpp_tutorials package has some demo applications which demonstrate this. 
   * The roscpp overview also contains more information. 
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%


/**
 * -------------------------------------------------------------------
 * Here's a condensed version of what's going on:

 *    Initialize the ROS system
 *    Subscribe to the chatter topic
 *    Spin, waiting for messages to arrive
 *    When a message arrives, the chatterCallback() function is called 
 * -------------------------------------------------------------------
 */
 
  return 0;
}
// %EndTag(FULLTEXT)%
