using System;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace ProBridge.Utils
{   
    public static class CDRSerializer
    {
        public static byte[] Serialize(object obj)
        {
            using (MemoryStream ms = new MemoryStream())
            {
                using (BinaryWriter writer = new BinaryWriter(ms))
                {
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
                
                AlignStream(writer, GetAlignment(fieldType));

                if (fieldType == typeof(byte))
                {
                    writer.Write((byte)value);
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
                else if (fieldType == typeof(string))
                {
                    WriteString(writer, (string)value);
                }
                else if (fieldType.IsArray)
                {
                    WriteArray(writer, value as Array);
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
                writer.Write(stringBytes.Length);
                writer.Write(stringBytes);
            }
        }

        private static void WriteArray(BinaryWriter writer, Array array)
        {
            if (array == null)
            {
                writer.Write(0); // Length 0 for null arrays
                return;
            }

            writer.Write(array.Length);

            foreach (var item in array)
            {
                Type itemType = item.GetType();
                AlignStream(writer, GetAlignment(itemType));

                if (itemType == typeof(byte))
                {
                    writer.Write((byte)item);
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
            if (type == typeof(int)) return 4;
            if (type == typeof(uint)) return 4;
            if (type == typeof(float)) return 4;
            if (type == typeof(string)) return 4;
            if (type.IsClass) return 1; // Assuming classes don't need alignment
            throw new InvalidOperationException($"Unknown type for alignment: {type.Name}");
        }

        private static void AlignStream(BinaryWriter writer, int alignment)
        {
            long position = writer.BaseStream.Position;
            long padding = (alignment - (position % alignment)) % alignment;
            for (int i = 0; i < padding; i++)
            {
                writer.Write((byte)0); // Write padding bytes
            }
        }
    }
}