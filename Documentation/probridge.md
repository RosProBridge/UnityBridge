# ProBridge Documentation


## Usage Guide

The ProBridge system consists of two main components that should be added to your Unity scene: `ProBridgeServer` and `ProBridgeHost`.

### ProBridgeServer

You only need a single `ProBridgeServer` component in your scene. This component is responsible for receiving messages from external sources. It requires three parameters:

- **IP**: The IP address on which the server will listen for incoming messages.
- **Port**: The port number on which the server will listen.
- **Queue Buffer**: The size of the buffer that manages incoming message queues.

These parameters determine how and where the ProBridgeServer listens for incoming ROS messages.

### ProBridgeHost

The `ProBridgeHost` component acts as your publisher in the scene. Unlike the `ProBridgeServer`, you can have multiple `ProBridgeHost` components to handle different IPs and ports. Each `ProBridgeHost` requires two parameters:

- **IP**: The IP address to which the host will send messages.
- **Port**: The port number used for publishing messages.

This setup allows you to send ROS messages to multiple destinations by configuring different hosts.

### Adding Publishers and Subscribers

Once you have configured your `ProBridgeServer` and `ProBridgeHost` components, you can add your publishers and subscribers as components to your GameObjects in the scene.

#### Subscribers (Rx)

For subscribers, there is one main parameter you need to configure:

- **Topic Name**: The name of the ROS topic that the subscriber will listen to. This parameter tells the subscriber which topic to monitor for incoming messages.

You can find the subscribers by checking the `Runtime/Rx` directory or if you want to create your own subscriber see [Creating Custom Subscribers](#creating-custom-subscribers)

#### Publishers (Tx)

For publishers, there are several parameters you need to adjust:

- **Host**: A reference to the `ProBridgeHost` you want to use for publishing. This links your publisher to a specific host configuration.
- **Send Rate**: The interval between consecutive messages, specified in seconds. This controls how frequently messages are sent.
- **Topic**: The name of the ROS topic that the publisher will send messages to.
- **Compression Level**: The level of compression to apply to the ROS messages (Note: This feature is currently not functional).
- **QOS**: Quality of Service settings, applicable only for ROS2. This parameter allows you to configure the reliability and durability of the message delivery.

You can find the subscribers by checking the `Runtime/Tx/Msgs` directory or if you want to create your own subscriber see [Creating Custom Publishers](#creating-custom-publishers)

## Creating Custom Publishers

Creating custom publishers in ProBridge allows you to transmit custom ROS messages based on specific data from your
Unity environment. This guide will walk you through the steps required to create a custom publisher by leveraging the
provided example classes.

### Understanding the Base Class Structure

Custom publishers in ProBridge are typically derived from the base classes `ProBridgeTx<T>` or `ProBridgeTxStamped<T>`,
where `T` is the ROS message type that you intend to publish. You can fine the available types in
the `Runtime/RosMsgs.cs` file. And if you want to add extra ROS messages see For more details,
see [Creating New Message Classes](#creating-new-message-classes).

- **`ProBridgeTx<T>`**: Used for messages that do not require a timestamped header.
- **`ProBridgeTxStamped<T>`**: Used for messages that include a `Header` with a timestamp.

### Defining the Custom Publisher Class

To create a custom publisher, define a new class that inherits from one of the base classes mentioned above. This class
will be responsible for gathering data, populating the message, and sending it through the ProBridge.

Example:

```csharp
using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Custom
{
    [AddComponentMenu("ProBridge/Tx/Custom/MyCustomPublisher")]
    public class MyCustomPublisher : ProBridgeTx<ROS.Msgs.Custom.MyCustomMsg>
    {
        // Add fields to store data that you want to publish
        public float customData;

        protected override void OnStart()
        {
            // Initialization logic, if needed
        }
        
        protected override void OnStop()
        {
            // Destruction logic, if needed
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            // Populate the message with data
            data.customField = customData;

            // Return the populated message
            return base.GetMsg(ts);
        }
    }
}
```

### Handling Timestamps and Stamped Messages

If your message includes a timestamped `Header`, ensure that your custom publisher inherits
from `ProBridgeTxStamped<T>`. This base class automatically handles the population of the `Header` field with the
correct timestamp. You only need to focus on the specific data fields.

Example:

```csharp
public class MyStampedPublisher : ProBridgeTxStamped<ROS.Msgs.Custom.MyStampedMsg>
{
    protected override ProBridge.Msg GetMsg(TimeSpan ts)
    {
        data.customField = customData;
        // The timestamp in the header is automatically handled by ProBridgeTxStamped
        return base.GetMsg(ts);
    }
}
```

### Examples

For more examples on how to implement custom publishers, refer to the classes in the `Runtime/Tx/Msgs` directory.

## Creating Custom Subscribers

Creating custom subscribers in ProBridge allows you to receive and handle specific ROS messages within your Unity
environment. This guide will walk you through the steps required to create a custom subscriber by leveraging the
provided example classes.

### Understanding the Base Class Structure

Custom subscribers in ProBridge are typically derived from the `ProBridgeRx<T>` base class, where `T` is the ROS message
type that you intend to subscribe to and handle. You can fine the available types in the `Runtime/RosMsgs.cs` file. And
if you want to add extra ROS messages see For more details,
see [Creating New Message Classes](#creating-new-message-classes).

- **`ProBridgeRx<T>`**: This is an abstract base class designed to manage subscription to a ROS topic and handle
  incoming messages. You will need to implement the `OnMessage(T msg)` method to define how your subscriber handles the
  received message.

### Defining the Custom Subscriber Class

To create a custom subscriber, define a new class that inherits from `ProBridgeRx<T>`. This class will be responsible
for processing the incoming ROS messages on a specific topic.

Example:

```csharp
using UnityEngine;
using ProBridge.Rx;

namespace ProBridge.Rx.Custom
{
    public class MyCustomSubscriber : ProBridgeRx<ROS.Msgs.Custom.MyCustomMsg>
    {
        protected override void OnMessage(MyCustomMsg msg)
        {
            // Handle the received message
            Debug.Log("Received custom message: " + msg.customField);
        }
    }
}
```

### Assigning a Topic

Each subscriber needs to specify the topic it is subscribed to. You can set the `topic` field directly in the Unity
Inspector or programmatically within your script. This ensures that your subscriber listens to the correct ROS topic.

Example:

```csharp
public string topic = "/custom_topic";
```

Alternatively, set the topic in the Unity Inspector after attaching the script to a GameObject.

### Example: Custom Clock Subscriber

Here is a complete example of a custom subscriber similar to the provided `ClockRx` class:

```csharp
using UnityEngine;

namespace ProBridge.Rx
{
    public class ClockRx : ProBridgeRx<ROS.Msgs.Rosgraph.Clock>
    {
        protected override void OnMessage(Clock msg)
        {
            Debug.Log("Clock output: " + msg.clock.sec + " seconds and " + msg.clock.nanosec + " nanoseconds.");
        }
    }
}
```

## Creating New Message Classes

### Overview

In the context of ProBridge, message classes are data classes that resemble ROS message types (rosmsg). These classes
are necessary for the ProBridge to send messages of the corresponding types. This guide provides instructions on how to
create new message classes based on the existing structure.

### Step-by-Step Guide

#### 1. **Create a New Class**

To define a new message class, create a new class that implements the `IRosMsg` interface. This interface requires
implementing the `GetRosType()` method, which returns the ROS type string corresponding to the message.

Example:

```csharp
using System;
using ProBridge.ROS.Msgs;

namespace ProBridge.ROS.Msgs.Custom
{
    public class CustomMsg : IRosMsg
    {
        // Define the data fields of the message
        public float customField1;
        public int customField2;
        public string customField3;

        // Implement the GetRosType method to return the corresponding ROS message type string
        public string GetRosType()
        {
            return "custom_msgs.msg.CustomMsg";
        }
    }
}
```

#### 2. **Implement the `GetRosType` Method**

The `GetRosType` method should return a string that corresponds to the full ROS message type, including the package and
message name. This string is used by the ProBridge to correctly identify and handle the message.

Example:

```csharp
public string GetRosType()
{
    return "custom_msgs.msg.CustomMsg";
}
```

#### 3. **Define the Data Fields**

Define the data fields in the class that represent the message data. These fields should match the structure and types
used in the corresponding ROS message.

Example:

```csharp
public float customField1;
public int customField2;
public string customField3;
```

#### 4. **Handle Time Fields (if applicable)**

If your message includes time fields, you can use the `Time` class provided in the namespace. Depending on whether
ROS_V2 is defined, the `Time` class will have different field names (`sec` and `nanosec` or `secs` and `nsecs`).

Example:

```csharp
public Time timestamp = new Time();
```

#### 5. **Use Wrapper Classes for Complex Data Types**

For complex data types, such as vectors, points, or orientations, use the provided wrapper classes
like `Vector3`, `Quaternion`, `Point`, etc. These classes are designed to encapsulate the complex data structures used
in ROS messages, ensuring compatibility and ease of use.

Example:

```csharp
public Vector3 position = new Vector3();
public Quaternion orientation = new Quaternion();
```

#### 6. **Optional: Implement the `IStamped` Interface**

If your message includes a `Header` field and represents a stamped message, implement the `IStamped` interface. This
interface ensures that your message has a `Header` field and provides a standardized structure for stamped messages.

Example:

```csharp
public class CustomStampedMsg : IRosMsg, IStamped
{
    public Header header { get; set; } = new Header();
    public float customField1;
    public int customField2;
    public Vector3 position = new Vector3();

    public string GetRosType()
    {
        return "custom_msgs.msg.CustomStampedMsg";
    }
}
```

#### 7. **Add the Class to the Appropriate Namespace**

Place your new message class within the appropriate namespace, typically under `ProBridge.ROS.Msgs.<Package>`. This
organization ensures that your message classes are easy to locate and maintain.

Example:

```csharp
namespace ProBridge.ROS.Msgs.Custom
{
    // Your message class here
}
```

### Example: Creating a New Message Class

```csharp
using System;
using ProBridge.ROS.Msgs;

namespace ProBridge.ROS.Msgs.Custom
{
    public class ExampleMsg : IRosMsg
    {
        public string name;
        public int id;
        public Time timestamp = new Time();
        public Vector3 position = new Vector3();
        public Quaternion orientation = new Quaternion();

        public string GetRosType()
        {
            return "custom_msgs.msg.ExampleMsg";
        }
    }
}
```

### Quick Method for Creating New Messages

A convenient way to create new message classes is by using ChatGPT with the following prompt:

*"Based on these classes, can you write a class
for [Insert full ROS message name here]: [Paste the contents of RosMsgs.cs here]."*

However, it’s important to verify the generated class against
the [official ROS documentation](https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html) to ensure the correct
order and types of variables are used. Additionally, if the message includes member variables that are instances of
unimplemented ROS subclasses, you’ll need to implement those first. For example, you would need to implement
the `NavSatStatus` class before creating the `NavSatFix` class.
