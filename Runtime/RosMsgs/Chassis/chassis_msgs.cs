using System;
using std_msgs;
using std_msgs.msg;

namespace chassis_msgs
{
    namespace msg
    {
        public class ChassisStatus : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "chassis_msgs.msg.ChassisStatus";
            }

            public Header header { get; set; } = new Header();

            public float battery; // Íàïðÿæåíèå ÀÊÁ Âîëüò
            public float fuel_available; // Çàïàñ òîïëèâà ë
            public float fuel_consumption; // Òåêóùèé (óñðåäíåííûé çà ìèí) ðàñõîä òîïëèâà ë/÷
            public sbyte engine_state; // Ñòàòóñ ÄÂÑ
            public sbyte engine_temp; // Òåìïåðàòóðà ÄÂÑ ãðàäóñ
            public UInt16 engine_value; // Îáîðîòû ÄÂÑ îá/ìèí
            public sbyte transmission_state; // Ñòàòóñ ÀÊÏÏ
            public sbyte transmission_value; // Çíà÷åíèå ïåðåäà÷è ÀÊÏÏ
            public sbyte transmission_temp; // Òåìïåðàòóðà ÀÊÏÏ ãðàäóñ
            public sbyte transfer_value; // Çíà÷åíèå ðàçäàòî÷íîé êîðîáêè
            public sbyte main_brake_state; // Ñòàòóñ òîðìîçíîé ñèñòåìû
            public sbyte parking_brake_state; // Ñòàòóñ ïàðêîâî÷íîãî òîðìîçà
            public sbyte rail_state; // Ñòàòóñ ðóëåâîé ðåéêè
            public sbyte[] parts_temp = new sbyte[0]; // Òåìïåðàòóðà ñîñòàâíûõ ÷àñòåé ÐÒÑ ãðàäóñ
            public sbyte[] general_state = new sbyte[0]; // Ñòàòóñû äàò÷èêîâ, ïðèáîðîâ îñâåùåíèÿ, èíäèêàòîðû, è ò.ä.
            public bool sto; // Ðàçðåøåíèå äâèæåíèÿ
        }

        public class ChassisFeed : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "chassis_msgs.msg.ChassisFeed";
            }

            public Header header { get; set; } = new Header();
            public float speed; // Ïîêàçàíèÿ äàò÷èêà ñêîðîñòè [ì/ñåê]
            public Int16[] engine_value = new Int16[0]; // Îáîðîòû äâèãàòåëÿ [îá/ìèí]
            public float[] rail_value = new float[0]; // Óãîë ïîâîðîòà ðóëåâîé ðåéêè [ðàäèàí]
            public float[] rail_speed = new float[0]; // Óãîëîâàÿ ñêîðîñòü ðóëåâîé ðåéêè â [ðàä/ñåê]
            public float[] rail_target = new float[0]; // Çàäàíèå íà ðóëåâóþ ðåéêó [óñë. åä]
            public float[] accel_value = new float[0]; // Ïîëîæåíèå ïåäàëè ãàçà [óñë. åä]
            public float[] accel_target = new float[0]; // Çàäàíèå íà ïåäàëü ãàçà [óñë. åä]

            public float[]
                brake_value =
                    new float[0]; // Çíà÷åíèå äàò÷èêà îáðàòíîé ñâÿçè ïî òîðìîçíîé ñèñòåìå (îòðèöàòåëüíîå çíà÷åíèå - êîä íåèñïðàâíîñòè) [óñë. åä]

            public float[] brake_target = new float[0]; // Çàäàíèå äëÿ òîðìîçíîé ñèñòåìû [óñë. åä]
        }

        public class ChassisSignals : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "chassis_msgs.msg.ChassisSignals";
            }

            public Header header { get; set; } = new Header();
            public bool lights_side; // ñîñòîÿíèå ãàáàðèòíûõ îãíåé
            public bool lights_head; // ñîñòîÿíèå ôîíàðåé ãîëîâíîãî ñâåòà
            public bool lights_left_turn; // ñîñòîÿíèå ëåâîãî óêàçàòåëÿ ïîâîðîòà
            public bool lights_right_turn; // ñîñòîÿíèå ïðàâîãî óêàçàòåëÿ ïîâîðîòà
            public bool sound_signal; // ñîñòîÿíèå çâóêîâîãî ñèãíàëà
            public byte[] aux = new byte[0]; // ñîñòîÿíèå äîïîëíèòåëüíîãî ñèãíàëüíîãî îáîðóäîâàíèÿ
        }

        public class ChassisControl : IRosMsg, IStamped
        {
            string IRosMsg.GetRosType()
            {
                return "chassis_msgs.msg.ChassisControl";
            }

            public Header header { get; set; } = new Header();

            public float throttle; //  0. <= throttle <= 1.
            public float steer; //  -1. <= steer <= 1.
            public float brake; //  0. <= brake <= 1
            public bool hand_brake; //  0. <= throttle <= 1.
            public bool reverse; //  reverse 0 or 1
            public Int32 gear; //  gear
            public bool manual_gear_shift; //  manual_gear_shift
        }
    }
}