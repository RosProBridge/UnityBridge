using UnityEditor;
using UnityEngine;
using ProBridge.Utils;

[CustomPropertyDrawer(typeof(Qos))]
public class QosPropertyDrawer : PropertyDrawer
{
    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        // Begin property handling
        EditorGUI.BeginProperty(position, label, property);

        // Find the properties in the serialized class
        SerializedProperty qosTypeProp = property.FindPropertyRelative("qosType");
        SerializedProperty intQosProp = property.FindPropertyRelative("intQos");
        SerializedProperty enumQosProp = property.FindPropertyRelative("enumQos");
        SerializedProperty reliabilityProp = property.FindPropertyRelative("reliability");
        SerializedProperty historyProp = property.FindPropertyRelative("history");
        SerializedProperty depthProp = property.FindPropertyRelative("depth");
        SerializedProperty durabilityProp = property.FindPropertyRelative("durability");
        SerializedProperty livelinessProp = property.FindPropertyRelative("liveliness");

        // Calculate the position of the fields
        Rect fieldRect = new Rect(position.x, position.y, position.width, EditorGUIUtility.singleLineHeight);

        // Draw the QOSType dropdown
        EditorGUI.PropertyField(fieldRect, qosTypeProp);
        fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

        Qos.QOSType qosType = (Qos.QOSType)qosTypeProp.enumValueIndex;

        // Draw the fields based on the selected QOSType
        switch (qosType)
        {
            case Qos.QOSType.Int:
                EditorGUI.PropertyField(fieldRect, intQosProp, new GUIContent("QOS Int"));
                break;

            case Qos.QOSType.Enum:
                EditorGUI.PropertyField(fieldRect, enumQosProp, new GUIContent("QOS Enum"));
                break;

            case Qos.QOSType.Dict:
                EditorGUI.LabelField(fieldRect, "Dictionary Fields");
                fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

                EditorGUI.PropertyField(fieldRect, reliabilityProp, new GUIContent("Reliability"));
                fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
                EditorGUI.PropertyField(fieldRect, historyProp, new GUIContent("History"));
                fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
                EditorGUI.PropertyField(fieldRect, depthProp, new GUIContent("Depth"));
                fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
                EditorGUI.PropertyField(fieldRect, durabilityProp, new GUIContent("Durability"));
                fieldRect.y += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;
                EditorGUI.PropertyField(fieldRect, livelinessProp, new GUIContent("Liveliness"));
                break;
        }

        // End property handling
        EditorGUI.EndProperty();
    }

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        SerializedProperty qosTypeProp = property.FindPropertyRelative("qosType");
        Qos.QOSType qosType = (Qos.QOSType)qosTypeProp.enumValueIndex;

        int lines = 1; // QOSType dropdown

        // Add lines based on the selected QOSType
        switch (qosType)
        {
            case Qos.QOSType.Int:
            case Qos.QOSType.Enum:
                lines += 1;
                break;

            case Qos.QOSType.Dict:
                lines += 6; // Label + 5 dictionary fields
                break;
        }

        return EditorGUIUtility.singleLineHeight * lines + EditorGUIUtility.standardVerticalSpacing * (lines - 1);
    }
}
