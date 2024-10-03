using System.IO;
using System.Linq;
using Newtonsoft.Json;
using UnityEngine;

namespace ProBridge
{
    public static class ParamsReader
    {
        public class ParamsData
        {
            public class ParamsObject
            {
                public class KeyValue
                {
                    public string name = "";
                    public string value;
                }

                public string name = "";
                public string type = "";
                public KeyValue[] properties = new KeyValue[0];
                public KeyValue[] fields = new KeyValue[0];

                public string GetProperty(string propertyName)
                {
                    return properties.FirstOrDefault(property => property.name == propertyName)?.value;
                }

                public string GetField(string fieldName)
                {
                    return fields.FirstOrDefault(field => field.name == fieldName)?.value;
                }
            }

            public ParamsObject[] objects = new ParamsObject[0];

            public ParamsObject GetObjectByType(string typeName)
            {
                return objects.FirstOrDefault(obj => obj.type == typeName);
            }

            public ParamsObject GetObjectByTypeAndName(string typeName, string objectName)
            {
                return objects.FirstOrDefault(obj => obj.name == objectName && obj.type == typeName);
            }

            public ParamsObject GetObjectByName(string objectName)
            {
                return objects.FirstOrDefault(obj => obj.name == objectName);
            }
        }

        public static string FileName
        {
            get { return Path.Combine(Application.dataPath, GetArg("--cfg", "cfg.json")); }
        }

        public static ParamsData Value
        {
            get
            {
                if (__value is null) Load();
                return __value;
            }
        }

        private static ParamsData __value = null;

        public static void Load()
        {
            var path = FileName;
            if (File.Exists(path))
                __value = JsonConvert.DeserializeObject<ParamsData>(File.ReadAllText(path));
            else
                __value = new ParamsData();
        }

        private static string GetArg(string name, string def = null)
        {
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == name && args.Length > i + 1)
                    return args[i + 1];
            }

            return def;
        }
    }
}