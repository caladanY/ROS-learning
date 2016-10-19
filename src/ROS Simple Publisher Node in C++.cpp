// %Tag(FULLTEXT)%

/**
 * "ros/ros.h" is a convenience include that includes all the headers necessary 
 * to use the most common public pieces of the ROS system.
 */
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
 

/**
 * This includes the std_msgs/String message, which resides in the std_msgs package. 
 * This is a header generated automatically from the String.msg file in that package. 
 * For more information on message definitions, see the msg page. 
 */
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

 
#include <sstream>


/**
 * --------------------------------------------------------------------------
 * This tutorial demonstrates simple sending of messages over the ROS system.
 * --------------------------------------------------------------------------
 */
 
 
int main(int argc, char **argv)
{
  /**
   * Initialize ROS. This allows ROS to do name remapping through the command line 
   * -- not important for now. This is also where we specify the name of our node. 
   * Node names must be unique in a running system. 
   *
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%


  /**
   * Create a handle to this process' node. 
   *
   * The first NodeHandle created will actually 
   * do the initialization of the node, and the last one destructed will cleanup any 
   * resources the node was using. 
   *
   * NodeHandle is the main access point to communications with the ROS system.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%


  /**
   * Tell the master that we are going to be publishing a message 
   * of type std_msgs/String on the topic chatter. 
   * This lets the master tell any nodes listening on chatter that 
   * we are going to publish data on that topic. 
   * The second argument is the size of our publishing queue. 
   * 
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages. If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   * In this case if we are publishing too quickly it will buffer up a maximum
   * of 1000 messages before beginning to throw away old ones.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%


  /**
   * A ros::Rate object allows you to specify a frequency that you would like to loop at.
   * It will keep track of how long it has been since the last call to Rate::sleep(), 
   * and sleep for the correct amount of time. 
   */
// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   *
   * By default roscpp will install a SIGINT handler which provides Ctrl-C handling 
   * which will cause ros::ok() to return false if that happens. 
   *
   * ros::ok() will return false if:
   *     a SIGINT is received (Ctrl-C)
   *     we have been kicked off the network by another node with the same name
   *     ros::shutdown() has been called by another part of the application.
   *     all ros::NodeHandles have been destroyed 
   * Once ros::ok() returns false, all ROS calls will fail.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%


    /**
     * This is a message object. You stuff it with data, and then publish it.
     *
     * We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. 
     * More complicated datatypes are possible, but for now we're going to use the standard String message, 
     * which has one member: "data". 
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%


   /**
    * ROS_INFO and friends are our replacement for printf/cout.
    * See the [rosconsole documentation](http://wiki.ros.org/rosconsole) for more information. 
    */
// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%


    /**
     * Now we actually broadcast the message to anyone who is connected. 
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%


   /**
    * Calling ros::spinOnce() here is not necessary for this simple program, 
    * because we are not receiving any callbacks. However, if you were to add 
    * a subscription into this application, and did not have ros::spinOnce() here, 
    * your callbacks would never get called. So, add it for good measure. 
    */
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%


   /**
    * Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate. 
    */
// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }
  

/**
 * ---------------------------------------------------------------------------------------------------------
 * Here's the condensed version of what's going on:
 *     Initialize the ROS system
 *     Advertise that we are going to be publishing std_msgs/String messages on the chatter topic to the master
 *     Loop while publishing messages to chatter 10 times a second 
 * ---------------------------------------------------------------------------------------------------------
 */
  return 0;
}
// %EndTag(FULLTEXT)%
