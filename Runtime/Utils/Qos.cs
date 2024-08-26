using System;
using UnityEngine;

[CreateAssetMenu(fileName = "Qos", menuName = "ProBridge/Qos")]
public class Qos : ScriptableObject
{
    public enum Reliability
    {
        SYSTEM_DEFAULT,
        BEST_EFFORT,
        RELIABLE,
        UNKNOWN
    }

    public enum History
    {
        KEEP_ALL,
        KEEP_LAST,
        SYSTEM_DEFAULT,
        UNKNOWN
    }

    public enum Durability
    {
        SYSTEM_DEFAULT,
        TRANSIENT_LOCAL,
        VOLATILE,
        UNKNOWN
    }

    public enum Liveliness
    {
        SYSTEM_DEFAULT,
        AUTOMATIC,
        MANUAL_BY_TOPIC,
        UNKNOWN
    }
    
    public enum QOSStr
    {
        qos_profile_parameters,
        qos_profile_sensor_data,
        qos_profile_services_default,
        qos_profile_system_default,
        qos_profile_unknown,
    }

    public enum QOSType
    {
        Int,
        Enum,
        Dict
    }

    public QOSType qosType = QOSType.Int;

    public int intQos;
    public QOSStr enumQos; 

    // Dictionary-like structure
    public Reliability reliability = Reliability.SYSTEM_DEFAULT;
    public History history = History.SYSTEM_DEFAULT;
    public int depth = 1;
    public Durability durability = Durability.SYSTEM_DEFAULT;
    public Liveliness liveliness = Liveliness.SYSTEM_DEFAULT;

    public object GetValue()
    {
        // TODO: Implement return mechanism
        return null;
    }
}