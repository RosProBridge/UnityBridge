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

        public static T Deserialize<T>(byte[] data) where T : new()
        {
            using (MemoryStream ms = new MemoryStream(data))
            {
                using (BinaryReader reader = new BinaryReader(ms))
                {
                    return (T)DeserializeObject(reader, typeof(T));
                }
            }
        }
        private static object DeserializeObject(BinaryReader reader, Type type)
        {
            object obj = Activator.CreateInstance(type);

            foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
            {
                AlignStream(reader, GetAlignment(field.FieldType));

                if (field.FieldType == typeof(byte))
                {
                    field.SetValue(obj, reader.ReadByte());
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
                else if (field.FieldType == typeof(string))
                {
                    field.SetValue(obj, ReadString(reader));
                }
                else if (field.FieldType.IsArray)
                {
                    field.SetValue(obj, ReadArray(reader, field.FieldType.GetElementType()));
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
            return Encoding.UTF8.GetString(reader.ReadBytes(length));
        }

        private static Array ReadArray(BinaryReader reader, Type elementType)
        {
            int length = reader.ReadInt32();
            if (length == 0)
            {
                return Array.CreateInstance(elementType, 0);
            }

            Array array = Array.CreateInstance(elementType, length);

            for (int i = 0; i < length; i++)
            {
                AlignStream(reader, GetAlignment(elementType));

                if (elementType == typeof(byte))
                {
                    array.SetValue(reader.ReadByte(), i);
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
            long padding = (alignment - (position % alignment)) % alignment;
            reader.BaseStream.Seek(padding, SeekOrigin.Current); // Skip padding bytes
        }
    }
}