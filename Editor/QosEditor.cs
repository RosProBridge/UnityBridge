using ProBridge.Utils;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Qos))]
public class QosEditor : UnityEditor.Editor
{
    SerializedProperty qosTypeProp;
    SerializedProperty intQosProp;
    SerializedProperty enumQosProp;
    SerializedProperty reliabilityProp;
    SerializedProperty historyProp;
    SerializedProperty depthProp;
    SerializedProperty durabilityProp;
    SerializedProperty livelinessProp;

    private void OnEnable()
    {
        qosTypeProp = serializedObject.FindProperty("qosType");
        intQosProp = serializedObject.FindProperty("intQos");
        enumQosProp = serializedObject.FindProperty("enumQos");
        reliabilityProp = serializedObject.FindProperty("reliability");
        historyProp = serializedObject.FindProperty("history");
        depthProp = serializedObject.FindProperty("depth");
        durabilityProp = serializedObject.FindProperty("durability");
        livelinessProp = serializedObject.FindProperty("liveliness");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(qosTypeProp);

        Qos.QOSType qosType = (Qos.QOSType)qosTypeProp.enumValueIndex;

        switch (qosType)
        {
            case Qos.QOSType.Int:
                EditorGUILayout.PropertyField(intQosProp, new GUIContent("QOS Int"));
                break;

            case Qos.QOSType.Enum:
                EditorGUILayout.PropertyField(enumQosProp, new GUIContent("QOS Enum"));
                break;

            case Qos.QOSType.Dict:
                EditorGUILayout.LabelField("Dictionary Fields");

                // Render the dictionary fields with appropriate dropdowns and input fields
                EditorGUILayout.PropertyField(reliabilityProp, new GUIContent("Reliability"));
                EditorGUILayout.PropertyField(historyProp, new GUIContent("History"));
                EditorGUILayout.PropertyField(depthProp, new GUIContent("Depth"));
                EditorGUILayout.PropertyField(durabilityProp, new GUIContent("Durability"));
                EditorGUILayout.PropertyField(livelinessProp, new GUIContent("Liveliness"));
                break;
        }

        serializedObject.ApplyModifiedProperties();
    }
}
