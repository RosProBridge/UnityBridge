using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace ProBridge.Utils
{
    [BurstCompile]
    public struct IArrayToBytesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> items;
        [ReadOnly] public int itemSize;

        [WriteOnly]public NativeArray<byte> serialized;

        public void Execute(int index)
        {
            UnManagedBitConverter.GetBytesByte(items[index]).CopyTo(serialized.GetSubArray(index * itemSize, itemSize));
        }
    }


    public static class UnManagedBitConverter
    {
       public static NativeArray<byte> GetBytesBool(bool value)
    {
        var bytes = new NativeArray<byte>(1, Allocator.Temp);
        bytes[0] = value ? (byte)1 : (byte)0;
        return bytes;
    }

    public static NativeArray<byte> GetBytesByte(byte value)
    {
        var bytes = new NativeArray<byte>(1, Allocator.Temp);
        bytes[0] = value;
        return bytes;
    }

    public static NativeArray<byte> GetBytesSByte(sbyte value)
    {
        var bytes = new NativeArray<byte>(1, Allocator.Temp);
        bytes[0] = (byte)value;
        return bytes;
    }

    public static NativeArray<byte> GetBytesShort(short value)
    {
        var bytes = new NativeArray<byte>(2, Allocator.Temp);
        bytes[0] = (byte)(value & 0xFF);
        bytes[1] = (byte)((value >> 8) & 0xFF);
        return bytes;
    }

    public static NativeArray<byte> GetBytesUShort(ushort value)
    {
        var bytes = new NativeArray<byte>(2, Allocator.Temp);
        bytes[0] = (byte)(value & 0xFF);
        bytes[1] = (byte)((value >> 8) & 0xFF);
        return bytes;
    }

    public static NativeArray<byte> GetBytesInt(int value)
    {
        var bytes = new NativeArray<byte>(4, Allocator.Temp);
        bytes[0] = (byte)(value & 0xFF);
        bytes[1] = (byte)((value >> 8) & 0xFF);
        bytes[2] = (byte)((value >> 16) & 0xFF);
        bytes[3] = (byte)((value >> 24) & 0xFF);
        return bytes;
    }

    public static NativeArray<byte> GetBytesUInt(uint value)
    {
        var bytes = new NativeArray<byte>(4, Allocator.Temp);
        bytes[0] = (byte)(value & 0xFF);
        bytes[1] = (byte)((value >> 8) & 0xFF);
        bytes[2] = (byte)((value >> 16) & 0xFF);
        bytes[3] = (byte)((value >> 24) & 0xFF);
        return bytes;
    }

    public static NativeArray<byte> GetBytesFloat(float value)
    {
        unsafe
        {
            int intValue = *((int*)&value);
            return GetBytesInt(intValue);
        }
    }

    public static NativeArray<byte> GetBytesDouble(double value)
    {
        unsafe
        {
            long longValue = *((long*)&value);
            var bytes = new NativeArray<byte>(8, Allocator.Temp);
            bytes[0] = (byte)(longValue & 0xFF);
            bytes[1] = (byte)((longValue >> 8) & 0xFF);
            bytes[2] = (byte)((longValue >> 16) & 0xFF);
            bytes[3] = (byte)((longValue >> 24) & 0xFF);
            bytes[4] = (byte)((longValue >> 32) & 0xFF);
            bytes[5] = (byte)((longValue >> 40) & 0xFF);
            bytes[6] = (byte)((longValue >> 48) & 0xFF);
            bytes[7] = (byte)((longValue >> 56) & 0xFF);
            return bytes;
        }
    }
    }
}