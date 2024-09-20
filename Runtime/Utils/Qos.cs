using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;

[Serializable]
public class Qos
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


    public Qos(Object qosObject)
    {
        Type qosObjectType = qosObject.GetType();

        if (qosObjectType == typeof(long))
        {
            qosType = QOSType.Int;
            var tmpVal = (long)qosObject;
            intQos = (int)tmpVal;
        }
        else if (qosObjectType == typeof(string))
        {
            qosType = QOSType.Enum;
            string val = (string)qosObject;
            enumQos = (QOSStr)Enum.Parse(typeof(QOSStr), val);
        }
        else
        {
            var qosDict = (JObject)qosObject;

            qosType = QOSType.Dict;

            string val = (string)qosDict["reliability"];
            reliability = (Reliability)Enum.Parse(typeof(Reliability), val);

            val = (string)qosDict["history"];
            history = (History)Enum.Parse(typeof(History), val);

            var numVal = (long)qosDict["depth"];
            depth = (int)numVal;

            val = (string)qosDict["durability"];
            durability = (Durability)Enum.Parse(typeof(Durability), val);

            val = (string)qosDict["liveliness"];
            liveliness = (Liveliness)Enum.Parse(typeof(Liveliness), val);
        }
    }

    public object GetValue()
    {
        if (qosType == QOSType.Int) return intQos;
        if (qosType == QOSType.Enum) return enumQos.ToString();
        if (qosType == QOSType.Dict)
        {
            Dictionary<string, object> dict = new Dictionary<string, object>();
            dict["reliability"] = reliability.ToString();
            dict["history"] = history.ToString();
            dict["depth"] = depth;
            dict["durability"] = durability.ToString();
            dict["liveliness"] = liveliness.ToString();
            return dict;
        }

        return null;
    }
}