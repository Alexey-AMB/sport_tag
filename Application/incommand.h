

typedef enum InCommand
{
    CMD_NONE,               //test command
    CMD_GET_SETTINGS,       //пришли настройки на ПК
    CMD_SET_SETTINGS,       //установи настройки
    CMD_SET_TIME,           //установи время часов
    CMD_GET_AKKVOLTAGE,     //пришли напряжение на аккумуляторе
    CMD_SET_BLINK,          //помигай лампочками
    CMD_WRITE_FOTO,         //запиши данные фото
    CMD_READ_FOTO,          //считай данные фото
    CMD_NEXT
}InCommand;

#pragma pack(push, 1)
typedef struct stCommand
{
    uint8_t signature;  //стартовая сигнатура 223
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
    char        name_tag[20];       //имя метки
    int         timeut_conn;        //время через которое метка уснет в сек. в режиме соединения
    int         timeut_run;         //время через которое метка уснет в сек. в режиме забега
    uint8_t     powerble_tag;       //мощность BLE передатчика метки от 0 (-21 дБ) до 12 (+5 дБ) см ll.h строка 404
    int8_t     treshold_tag;       //порог чувствительности метки
    char        password_tag[10];   //пароль BLE

    //USER_SETTINGS        //len=103

    char        fam[20];        //фамилия
    char        imj[20];        //имя
    char        otch[20];       //отчество

    char        group[4];       //группа
    char        razr[4];        //разряд
    uint16_t    godrojd;        //год рождения
    char        colectiv[20];   //название коллектива
    uint8_t     zabeg;          //номер забега
    uint16_t    startnum;       //стартовый номер
    char        starttime[7];   //время старта
    bool        lgota;          //есть ли льгота
    bool        arenda;         //свой чип или арендованный
    uint8_t     signature;      //сигнатура SIGNATURE_EPROM_SETTINGS
}SPORT_TAG_SETTINGS;
#pragma pack(pop)
