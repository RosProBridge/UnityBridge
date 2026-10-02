namespace std_msgs
{
    public interface IRosMsg
    {
        public string GetRosType();
    }

    /// <summary>
    /// A message that sends only the first part of an unbounded array, so a large buffer can be reused
    /// between messages instead of allocating an array of the exact size every time.
    /// </summary>
    public interface ICdrArrayLength
    {
        /// <summary>Number of elements of array field <paramref name="fieldName"/> to serialize.</summary>
        int GetSerializedLength(string fieldName, int arrayLength);
    }
}