using System;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace ProBridge.Utils
{   
    public static class CDRSerializer
    {
        
#if ROS_V2
        // Used to add alignment and string terminator incase of ROS2
        private static bool ROS2Serialization = true;
#else
        private static bool ROS2Serialization = false;
#endif
        public static byte[] Serialize(object obj)
        {
            using (MemoryStream ms = new MemoryStream())
            {
                using (BinaryWriter writer = new BinaryWriter(ms))
                {
                    // CDR Header
                    if(ROS2Serialization)
                    {
                        writer.Write((byte)0x00);
                        writer.Write((byte)0x01);
                        writer.Write((byte)0x00);
                        writer.Write((byte)0x00);
                    }
                    
                    SerializeObject(writer, obj);
                }
                return ms.ToArray();
            }
        }

        private static void SerializeObject(BinaryWriter writer, object obj)
        {
            if (obj == null) return;

            Type type = obj.GetType();

            foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
            {
                object value = field.GetValue(obj);
                Type fieldType = value.GetType();

                if (ROS2Serialization) AlignStream(writer, GetAlignment(fieldType));

                if (fieldType == typeof(byte))
                {
                    writer.Write((byte)value);
                }
                else if (fieldType == typeof(sbyte))
                {
                    writer.Write((sbyte)value);
                }
                else if (fieldType == typeof(short))
                {
                    writer.Write((short)value);
                }
                else if (fieldType == typeof(ushort))
                {
                    writer.Write((ushort)value);
                }
                else if (fieldType == typeof(int))
                {
                    writer.Write((int)value);
                }
                else if (fieldType == typeof(uint))
                {
                    writer.Write((uint)value);
                }
                else if (fieldType == typeof(float))
                {
                    writer.Write((float)value);
                }
                else if (fieldType == typeof(double))
                {
                    writer.Write((double)value);
                }
                else if (fieldType == typeof(string))
                {
                    WriteString(writer, (string)value);
                }
                else if (fieldType.IsArray)
                {
                    Object tmpInstence = Activator.CreateInstance(type);
                    Object arrayValue = field.GetValue(tmpInstence);
                    if (arrayValue == null)
                    {
                        WriteArray(writer, value as Array, true);
                    }
                    else
                    {
                        WriteArray(writer, value as Array);
                    }
                    
                }
                else if (fieldType.IsClass && !fieldType.IsPrimitive) // Classes
                {
                    SerializeObject(writer, value);
                }
                else
                {
                    throw new InvalidOperationException($"Unsupported data type: {fieldType.Name}");
                }
            }
        }

        private static void WriteString(BinaryWriter writer, string value)
        {
            if (value == null)
            {
                writer.Write(0); // Length 0 for null strings
            }
            else
            {
                var stringBytes = Encoding.UTF8.GetBytes(value);
                writer.Write(stringBytes.Length + (ROS2Serialization ? 1 : 0));
                writer.Write(stringBytes);
                if (ROS2Serialization) writer.Write((byte)0);
            }
        }

        private static void WriteArray(BinaryWriter writer, Array array, bool addLength = false)
        {
            if (addLength)
            {
                if (ROS2Serialization) AlignStream(writer, GetAlignment(typeof(int)));
                writer.Write(array.Length);
            }

            foreach (var item in array)
            {
                Type itemType = item.GetType();
                if (ROS2Serialization) AlignStream(writer, GetAlignment(itemType));

                if (itemType == typeof(byte))
                {
                    writer.Write((byte)item);
                }
                else if (itemType == typeof(sbyte))
                {
                    writer.Write((sbyte)item);
                }
                else if (itemType == typeof(short))
                {
                    writer.Write((short)item);
                }
                else if (itemType == typeof(ushort))
                {
                    writer.Write((ushort)item);
                }
                else if (itemType == typeof(int))
                {
                    writer.Write((int)item);
                }
                else if (itemType == typeof(uint))
                {
                    writer.Write((uint)item);
                }
                else if (itemType == typeof(float))
                {
                    writer.Write((float)item);
                }
                else if (itemType == typeof(double))
                {
                    writer.Write((double)item);
                }
                else if (itemType == typeof(string))
                {
                    WriteString(writer, (string)item);
                }
                else if (itemType.IsClass && !itemType.IsPrimitive) // Classes
                {
                    SerializeObject(writer, item);
                }
                else
                {
                    throw new InvalidOperationException($"Unsupported array item type: {itemType.Name}");
                }
            }
        }

        private static int GetAlignment(Type type)
        {
            if (type == typeof(byte)) return 1;
            if (type == typeof(sbyte)) return 1;
            if (type == typeof(short)) return 2;
            if (type == typeof(ushort)) return 2;
            if (type == typeof(int)) return 4;
            if (type == typeof(uint)) return 4;
            if (type == typeof(float)) return 4;
            if (type == typeof(string)) return 4;
            if (type == typeof(double)) return 8;
            if (type.IsClass) return 1; // Classes and arrays don't need alignment and align based on their contents
            throw new InvalidOperationException($"Unknown type for alignment: {type.Name}");
        }

        private static void AlignStream(BinaryWriter writer, int alignment)
        {
            long position = writer.BaseStream.Position;
            if (ROS2Serialization)
            {
                position -= 4; // To account for the CDR header
            }
            long padding = (alignment - (position % alignment)) % alignment;
            for (int i = 0; i < padding; i++)
            {
                writer.Write((byte)0); // Write padding bytes
            }
        }

        public static T Deserialize<T>(byte[] data) where T : new()
        {
            using (MemoryStream ms = new MemoryStream(data))
            {
                using (BinaryReader reader = new BinaryReader(ms))
                {
                    reader.BaseStream.Seek(4, SeekOrigin.Begin); // Skip the first 4 header bytes
                    return (T)DeserializeObject(reader, typeof(T));
                }
            }
        }
        
        
        private static object DeserializeObject(BinaryReader reader, Type type)
        {
            object obj = Activator.CreateInstance(type);

            foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
            {
                if(ROS2Serialization) AlignStream(reader, GetAlignment(field.FieldType));

                if (field.FieldType == typeof(byte))
                {
                    field.SetValue(obj, reader.ReadByte());
                }
                else if (field.FieldType == typeof(sbyte))
                {
                    field.SetValue(obj, reader.ReadSByte());
                }
                else if (field.FieldType == typeof(short))
                {
                    field.SetValue(obj, reader.ReadInt16());
                }
                else if (field.FieldType == typeof(ushort))
                {
                    field.SetValue(obj, reader.ReadUInt16());
                }
                else if (field.FieldType == typeof(int))
                {
                    field.SetValue(obj, reader.ReadInt32());
                }
                else if (field.FieldType == typeof(uint))
                {
                    field.SetValue(obj, reader.ReadUInt32());
                }
                else if (field.FieldType == typeof(float))
                {
                    field.SetValue(obj, reader.ReadSingle());
                }
                else if (field.FieldType == typeof(double))
                {
                    field.SetValue(obj, reader.ReadDouble());
                }
                else if (field.FieldType == typeof(string))
                {
                    field.SetValue(obj, ReadString(reader));
                }
                else if (field.FieldType.IsArray)
                {
                    var arr = (Array)field.GetValue(obj);
                    int arrLen;
                    if (arr == null)
                    {
                        if(ROS2Serialization) AlignStream(reader, GetAlignment(typeof(int)));
                        arrLen = reader.ReadInt32();
                    }
                    else
                    {
                        arrLen = arr.Length;
                    }
                    field.SetValue(obj, ReadArray(reader, field.FieldType.GetElementType(), arrLen));
                }
                else if (field.FieldType.IsClass && !field.FieldType.IsPrimitive) // Classes
                {
                    field.SetValue(obj, DeserializeObject(reader, field.FieldType));
                }
                else
                {
                    throw new InvalidOperationException($"Unsupported data type: {field.FieldType.Name}");
                }
            }

            return obj;
        }

        private static string ReadString(BinaryReader reader)
        {
            int length = reader.ReadInt32();
            if (length == 0)
            {
                return null;
            }

            var str = Encoding.UTF8.GetString(reader.ReadBytes(length - (ROS2Serialization ? 1 : 0)));
            
            // Skip the null terminator
            if(ROS2Serialization) reader.ReadByte();
            
            
            return str;
        }

        private static Array ReadArray(BinaryReader reader, Type elementType, int length)
        {
            if (length == 0)
            {
                return Array.CreateInstance(elementType, 0);
            }

            Array array = Array.CreateInstance(elementType, length);

            for (int i = 0; i < length; i++)
            {
                if(ROS2Serialization) AlignStream(reader, GetAlignment(elementType));

                if (elementType == typeof(byte))
                {
                    array.SetValue(reader.ReadByte(), i);
                }
                else if (elementType == typeof(sbyte))
                {
                    array.SetValue(reader.ReadSByte(), i);
                }
                else if (elementType == typeof(short))
                {
                    array.SetValue(reader.ReadInt16(), i);
                }
                else if (elementType == typeof(ushort))
                {
                    array.SetValue(reader.ReadUInt16(), i);
                }
                else if (elementType == typeof(int))
                {
                    array.SetValue(reader.ReadInt32(), i);
                }
                else if (elementType == typeof(uint))
                {
                    array.SetValue(reader.ReadUInt32(), i);
                }
                else if (elementType == typeof(float))
                {
                    array.SetValue(reader.ReadSingle(), i);
                }
                else if (elementType == typeof(double))
                {
                    array.SetValue(reader.ReadDouble(), i);
                }
                else if (elementType == typeof(string))
                {
                    array.SetValue(ReadString(reader), i);
                }
                else if (elementType.IsClass && !elementType.IsPrimitive) // Classes
                {
                    array.SetValue(DeserializeObject(reader, elementType), i);
                }
                else
                {
                    throw new InvalidOperationException($"Unsupported array element type: {elementType.Name}");
                }
            }

            return array;
        }

        private static void AlignStream(BinaryReader reader, int alignment)
        {
            long position = reader.BaseStream.Position;
            if (ROS2Serialization)
            {
                position -= 4; // To account for the CDR header
            }
            long padding = (alignment - (position % alignment)) % alignment;
            reader.BaseStream.Seek(padding, SeekOrigin.Current); // Skip padding bytes
        }
    }
}