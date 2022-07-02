# Дипломный стенд "Векторный частотный преобразователь"

### Дипломный стенд "Векторный частотный преобразователь" специально создавался для зашиты дипломной работы. Данный стенд является только демонстрационной и учебной моделью и не может служить на заводах и предприятиях.

## Характеристики:

- Питающее напряжение: 220-240V
- Напряжение нагрузки: 80V±5%
- Тип управления нагрузкой: «ШИМ»
- Допустимый максимальный ток долговременной нагрузки до: 2,5А
- Допустимый максимальный ток кратковременной нагрузки до: 3А
- Предельный порог измерения выходного датчика тока: 5А
- Предельный порог измерения выходного датчика оборотов: 800 обр/сек.
- MicroPython V1.17

## Режимы работы
##### Для переключения режимов используется тумблер
### * MANUAL(ручной)
#### - В данном режиме работы настроен плавный пуск двигателя, а также его плавная остановка
#### - Для включения двигателя используется кнопка Start
#### - Для смены вращения используется кнопка Revers
#### - Кнопка Stop останавливает двигатель
#### - Регулировка скорости осуществляется Энкодером. Установить скорость можно как до пуска двигателя, так и в работе
### * AUTO(Автоматический)
#### - Данный режим запускает пользовательскую программу
#### - Выбор программы осуществляется энкодером
#### - Запуск осуществляется кнопкой Start. Предусмотрен режим от случайного запуску
#### - Загрузка пользовательских программ

## Сторонние библиотеки:
- RPI-PICO-I2C-LCD -> https://github.com/T-622/RPI-PICO-I2C-LCD

## Google презентация
- https://docs.google.com/presentation/d/1gC0sEfEZTGtS3hWSy852PbZ25VCcBvIVAnEaVDbH2I0/edit?usp=sharing

##### (разработка кода Денис / разработка и пайка схем Никита)
