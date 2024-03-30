using System;

namespace ProBridge.ROS.Msgs.Chassis
{
    public class ChassisStatus : Std.IStamped
    {
        public Std.Header header { get; } = new Std.Header();

        public float battery;                           // Напряжение АКБ Вольт
        public float fuel_available;                    // Запас топлива л
        public float fuel_consumption;                  // Текущий (усредненный за мин) расход топлива л/ч
        public sbyte engine_state;                      // Статус ДВС
        public sbyte engine_temp;                       // Температура ДВС градус
        public UInt16 engine_value;                     // Обороты ДВС об/мин
        public sbyte transmission_state;                // Статус АКПП
        public sbyte transmission_value;                // Значение передачи АКПП
        public sbyte transmission_temp;                 // Температура АКПП градус
        public sbyte transfer_value;                    // Значение раздаточной коробки
        public sbyte main_brake_state;                  // Статус тормозной системы
        public sbyte parking_brake_state;               // Статус парковочного тормоза
        public sbyte rail_state;                        // Статус рулевой рейки
        public sbyte[] parts_temp = new sbyte[0];       // Температура составных частей РТС градус
        public sbyte[] general_state = new sbyte[0];    // Статусы датчиков, приборов освещения, индикаторы, и т.д.
        public bool sto;                                // Разрешение движения
    }

    public class ChassisFeed : Std.IStamped
    {
        public Std.Header header { get; } = new Std.Header();
        public float speed;                         // Показания датчика скорости [м/сек]
        public Int16[] engine_value = new Int16[0]; // Обороты двигателя [об/мин]
        public float[] rail_value = new float[0];   // Угол поворота рулевой рейки [радиан]
        public float[] rail_speed = new float[0];   // Уголовая скорость рулевой рейки в [рад/сек]
        public float[] rail_target = new float[0];  // Задание на рулевую рейку [усл. ед]
        public float[] accel_value = new float[0];  // Положение педали газа [усл. ед]
        public float[] accel_target = new float[0]; // Задание на педаль газа [усл. ед]
        public float[] brake_value = new float[0];  // Значение датчика обратной связи по тормозной системе (отрицательное значение - код неисправности) [усл. ед]
        public float[] brake_target = new float[0]; // Задание для тормозной системы [усл. ед]
    }

    public class ChassisSignals : Std.IStamped
    {
        public Std.Header header { get; } = new Std.Header();
        public bool lights_side;            // состояние габаритных огней
        public bool lights_head;            // состояние фонарей головного света
        public bool lights_left_turn;       // состояние левого указателя поворота
        public bool lights_right_turn;      // состояние правого указателя поворота
        public bool sound_signal;           // состояние звукового сигнала
        public byte[] aux = new byte[0];     // состояние дополнительного сигнального оборудования
    }
}