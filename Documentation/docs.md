# ProBridge Documentation

<details>
  <summary>Table of Contents</summary>

- [ProBridge Documentation](#probridge-documentation)
  - [Usage Guide](#usage-guide)
    - [ProBridgeServer](#probridgeserver)
    - [ProBridgeHost](#probridgehost)
    - [Adding Publishers and Subscribers](#adding-publishers-and-subscribers)
      - [Subscribers (Rx)](#subscribers-rx)
      - [Publishers (Tx)](#publishers-tx)
      - [Services (Srv)](#services-srv)
  - [Creating Custom Publishers](#creating-custom-publishers)
    - [Understanding the Base Class Structure](#understanding-the-base-class-structure)
    - [Defining the Custom Publisher Class](#defining-the-custom-publisher-class)
    - [Handling Timestamps and Stamped Messages](#handling-timestamps-and-stamped-messages)
    - [Examples](#examples)
  - [Creating Custom Subscribers](#creating-custom-subscribers)
    - [Understanding the Base Class Structure](#understanding-the-base-class-structure-1)
    - [Defining the Custom Subscriber Class](#defining-the-custom-subscriber-class)
    - [Assigning a Topic](#assigning-a-topic)
    - [Example: Custom Clock Subscriber](#example-custom-clock-subscriber)
  - [Creating New Message Classes](#creating-new-message-classes)
    - [Overview](#overview)
    - [Step-by-Step Guide](#step-by-step-guide)
      - [Create a New Class](#1-create-a-new-class)
      - [Implement the `GetRosType` Method](#2-implement-the-getrostype-method)
      - [Define the Data Fields](#3-define-the-data-fields)
      - [Handle Time Fields (if applicable)](#4-handle-time-fields-if-applicable)
      - [Use Wrapper Classes for Complex Data Types](#5-use-wrapper-classes-for-complex-data-types)
      - [Optional: Implement the `IStamped` Interface](#6-optional-implement-the-istamped-interface)
      - [Add the Class to the Appropriate Namespace](#7-add-the-class-to-the-appropriate-namespace)
    - [Example: Creating a New Message Class](#example-creating-a-new-message-class)

</details>



## Usage Guide

The ProBridge system consists of two main components that should be added to your Unity scene: `ProBridgeServer` and `ProBridgeHost`.

### ProBridgeServer

The `ProBridgeServer` component handles incoming messages from external sources. Only one instance is needed per scene. It requires the following parameters:

- **IP**: The IP address for the server to listen on. (Use your machine's IP if on a network, or `localhost` (127.0.0.1) if you are running the system locally.)
- **Port**: The port number on which the server listens.
- **Queue Buffer**: The buffer size that controls the queue of incoming messages.

These settings determine how and where the `ProBridgeServer` listens for incoming ROS messages.

> **Note:** Adding `ProBridgeServer` to your scene automatically includes an `InitializationManager`, which is essential for the system to function properly.

### ProBridgeHost

The `ProBridgeHost` component acts as your publisher in the scene. Unlike the `ProBridgeServer`, you can have multiple `ProBridgeHost` components to handle different IPs and ports. Each `ProBridgeHost` requires two parameters:

- **IP**: The IP address to which the host will send messages. (Use the recipient machine's IP if on a network, or `localhost` (127.0.0.1) if you are running the system locally.)
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
- **Send Rate**: The interval between consecutive messages, in simulation seconds. Messages are sent at most once per physics step; `0` sends every physics step.

  Only building the message and its CDR serialization (into a pooled buffer) run on the main thread; the header, compression and the socket send run on a sender thread of the `ProBridgeHost` (in order; the oldest message is dropped if more than 64 are waiting).
- **Topic**: The name of the ROS topic that the publisher will send messages to.
- **Compression Level**: The level of compression to apply to the ROS messages.
- **Use Without Link**: Off by default. While the host has no connection to the ROS side, the publisher does nothing: `GetMsg` is not called, so sensors spend no CPU. Turn it on when other scripts read the publisher's `data` or subscribe to `OnSendMessage` at runtime without ROS; messages are then built, but still not serialized or sent until the link is up.
- **QOS**: Quality of Service settings, applicable only for ROS2. This parameter allows you to configure the reliability and durability of the message delivery. A publisher type can set its own default for new components by overriding `CreateDefaultQos()`.

> **Note:** Dynamic TF (`TfSender`) is also skipped while its host is disconnected. Static TF is sent on every new connection.

You can find the subscribers by checking the `Runtime/Tx/Msgs` directory or if you want to create your own subscriber see [Creating Custom Publishers](#creating-custom-publishers)

#### Services (Srv)

ROS services in both directions (ROS2 bridge only for now).

**Served by Unity, called from ROS** — `ProBridgeService<TRequest, TResponse>`:

- **Host**: The `ProBridgeHost` connected to the ROS bridge. The service is advertised through it on enable and on every connection, so the bridge creates the service in ROS by itself (nothing to add to the bridge config) and recreates it after a restart. The advertisement carries the `ProBridgeServer` port; the bridge connects to it at this machine's IP to send the calls, so `ProBridgeServer` must listen on an address reachable from the bridge.
- **Service**: The service name, e.g. `/sim/reload`.

Built-in services:

- **Scene Reload** (`ProBridge/Srv/std_srvs/Scene Reload`, `std_srvs/srv/Trigger`, e.g. `/sim/reload`): reloads the active scene. The response is sent first, the scene is reloaded on the next frame. Simulation time is not reset (it is based on `Time.fixedTimeAsDouble`), so ROS nodes with `use_sim_time` see no time jump.
- **Sim Pause** (`ProBridge/Srv/std_srvs/Sim Pause`, `std_srvs/srv/SetBool`, e.g. `/sim/pause`): `data: true` pauses the simulation, `false` resumes it.
  - Pause sets `Time.timeScale = 0` (physics, animations, simulation time / `/clock` and publishers stop) and pauses audio; resume restores the previous time scale. Scripts using unscaled time and UI keep working (e.g. a "resume" button).
  - The editor Pause button is not pressed, but a pause by the button counts as paused (reported by the service), and resuming releases it.
  - Incoming messages and service calls are still handled while paused (also under the editor Pause button), so `data: false` always gets through.
  - The pause survives a scene reload (`Time.timeScale` is global).
  - The same pause is available from code and UI through `ProBridge.SimPause`: `Pause()`, `Resume()`, `SetPaused(bool)`, `IsPaused` and the `Changed` event (raised on pause and resume, including the editor Pause button).

Custom service: derive from `ProBridgeService<TRequest, TResponse>` and implement `OnRequest`, which runs on the main thread and returns the response:

```csharp
public class EnableSensorsService : ProBridgeService<SetBool_Request, SetBool_Response>
{
    public GameObject sensorsRoot;

    protected override SetBool_Response OnRequest(SetBool_Request request)
    {
        sensorsRoot.SetActive(request.data);
        return new SetBool_Response { success = true, message = request.data ? "enabled" : "disabled" };
    }
}
```

**Served in ROS, called from Unity** — `ProBridgeServiceClient<TRequest, TResponse>`: list the service in `services` of the ROS bridge config. The response comes back to the `ProBridgeServer` of the caller (its port is sent with the request). Fields: **Host**, **Service**, **Timeout**.

```csharp
public class SaveMapClient : ProBridgeServiceClient<Trigger_Request, Trigger_Response> { }

saveMapClient.Call(new Trigger_Request(),
    response => Debug.Log($"{response.success}: {response.message}"),
    error => Debug.LogWarning(error));   // not connected or timeout
```

Request and response classes follow the rosidl names `<Service>_Request` / `<Service>_Response` (see `RosMsgs/StdSrvs/std_srvs.cs` for `Trigger`, `SetBool`, `Empty`); the service type is the request type without the `_Request` suffix. In ROS2 an empty request or response has a single `byte structure_needs_at_least_one_member` field.

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

> Note: It is recommended to suffix publisher class names with `Tx` for consistency and clarity.

Example:

```csharp
using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Custom
{
    [AddComponentMenu("ProBridge/Tx/Custom/MyCustomPublisher")]
    public class MyCustomPublisherTx : ProBridgeTx<ROS.Msgs.Custom.MyCustomMsg>
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
public class MyStampedPublisherTx : ProBridgeTxStamped<ROS.Msgs.Custom.MyStampedMsg>
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

> Note: It is recommended to suffix subscriber class names with `Rx` for consistency and clarity.

Example:

```csharp
using UnityEngine;
using ProBridge.Rx;

namespace ProBridge.Rx.Custom
{
    public class MyCustomSubscriberRx : ProBridgeRx<ROS.Msgs.Custom.MyCustomMsg>
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

Here is a complete example of a custom subscriber:

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
are necessary for the ProBridge to send or receive messages of the corresponding types. This guide provides instructions on how to
create new message classes based on the existing structure.

### Step-by-Step Guide

#### 1. **Create a New Class**

To define a new message class, create a new class that implements the `IRosMsg` interface. This interface requires
implementing the `GetRosType()` method, which returns the ROS type string corresponding to the message.

Example:

```csharp
namespace custom_msgs
{
    namespace msg
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

> Note: Ensure you use the ProBridge implementation of these classes. For example, the Unity version of `Vector3` will not work.

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



#### Optional: Helper Fields and Reused Buffers

- Fields marked `[System.NonSerialized]` are not part of the message (not serialized or deserialized).
- To send only the first part of an unbounded array (e.g. reuse a large `byte[]` buffer instead of allocating an exact-size array for every message), implement `std_msgs.ICdrArrayLength`: `GetSerializedLength(fieldName, arrayLength)` returns the number of elements to send. `sensor_msgs.msg.PointCloud2` does this with `dataLength` (`-1`: the whole `data` array).

#### 7. **Add the Class to the Appropriate Namespace**

If you are adding the message `custom_msgs.msg.CustomStampedMsg`, you need to place it in the `custom_msgs` namespace, followed by the `msg` namespace.

Example:

```csharp
namespace custom_msgs
{
    namespace msg
    {
        // Your message class here
    }
}
```


### Example: Creating a New Message Class

```csharp
using System;
using ProBridge.ROS.Msgs;

namespace custom_msgs
{
    namespace msg
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
}
```
