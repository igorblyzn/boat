
int pibBuzzer = 12; // пин для подключения зуммера
 
void setup()
{
    pinMode(pibBuzzer, OUTPUT); // включаем пин зуммера на вывод
        //byte i;
 
    // воспроизвести звук
    for (int i = 0; i < 50; i++)
    {
        digitalWrite(pibBuzzer, HIGH); // включить звук
        delay(1); // задержка в 1 мсек.
        digitalWrite(pibBuzzer, LOW); // выключить звук
        delay(1); // задержка в 1 мсек.
    }
    delay(100);
        for (int i = 0; i < 50; i++)
    {
        digitalWrite(pibBuzzer, HIGH); // включить звук
        delay(1); // задержка в 1 мсек.
        digitalWrite(pibBuzzer, LOW); // выключить звук
        delay(1); // задержка в 1 мсек.
    }

    delay(10000);
}
 
void loop()
{

 
    // Частота звука зависит от продолжительности задержки при включении и выклюении питания зуммера
    // аналогичный цикл, но с частотой в 2 раза меньше:

}
