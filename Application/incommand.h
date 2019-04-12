

typedef enum InCommand
{
    CMD_NONE,               //test command
    CMD_GET_SETTINGS,       //пришли настройки на ѕ 
    CMD_SET_SETTINGS,       //установи настройки
    CMD_SET_TIME,           //установи врем€ часов
    CMD_GET_AKKVOLTAGE,     //пришли напр€жение на аккумул€торе
    CMD_SET_BLINK,          //помигай лампочками
    //CMD_WRITE_DATA,         //запиши данные
    CMD_READ_DATA,          //считай данные
    CMD_SET_MODE_RUN,
    CMD_SET_MODE_CONN,
    CMD_SET_MODE_SLEEP,
    CMD_NEXT
}InCommand;

#pragma pack(push, 1)
typedef struct stCommand
{
    uint8_t signature;  //стартова€ сигнатура 223
    uint8_t cmd;        //команда из InCommand
    uint16_t lenbuf;    //длинна буфера с данными
    uint8_t crc;
}stCommand;
#pragma pack(pop)

typedef enum OutAsk
{
    ASK_OK,
    ASK_ERROR,
    ASK_NEXT,
    ASK_MALLOC_ERROR,
    ASK_ERROR_CRC
}OutAsk;

//сигнатура команды
#define SIGNATURE_COMMAND                   223

typedef enum WORKMODE
{
    MODE_CONNECT,
    MODE_RUN,
    MODE_SLEEP
}WORKMODE;


#pragma pack(push, 1)
typedef struct SPORT_TAG_SETTINGS   //len=42
{
    WORKMODE    mode_tag;           //режим работы
    char        name_tag[20];       //им€ метки
    int         timeut_conn;        //врем€ через которое метка уснет в сек. в режиме соединени€
    int         timeut_run;         //врем€ через которое метка уснет в сек. в режиме забега
    uint8_t     powerble_tag;       //мощность BLE передатчика метки от 0 (-21 дЅ) до 12 (+5 дЅ) см ll.h строка 404
    int8_t      treshold_tag;       //порог чувствительности метки
    char        password_tag[10];   //пароль BLE

    //USER_SETTINGS        //len=103

    char        fam[20];        //фамили€
    char        imj[20];        //им€
    char        otch[20];       //отчество

    char        group[4];       //группа
    char        razr[4];        //разр€д
    uint16_t    godrojd;        //год рождени€
    char        colectiv[20];   //название коллектива
    uint8_t     zabeg;          //номер забега
    uint16_t    startnum;       //стартовый номер
    char        starttime[7];   //врем€ старта
    bool        lgota;          //есть ли льгота
    bool        arenda;         //свой чип или арендованный
    uint8_t     signature;      //сигнатура SIGNATURE_EPROM_SETTINGS
}SPORT_TAG_SETTINGS;
#pragma pack(pop)

// «арезервированный номер стартовой станции
#define START_STATION_NUM         240
// «арезервированный номер финишной станции
#define FINISH_STATION_NUM        245
// «арезервированный номер станции проверки
#define CHECK_STATION_NUM         248
// «арезервированный номер станции очистки
#define CLEAR_STATION_NUM         249
